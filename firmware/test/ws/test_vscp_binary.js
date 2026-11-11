#!/usr/bin/env node

/*
  VSCP binary protocol smoke test over WebSocket ws1 endpoint.
  1. Connects in ws1 (text) mode
  2. Authenticates (USER/PASS/AUTH)
  3. Opens the channel (OPEN)
  4. Sends binary NOOP command frame and validates binary reply
*/

const crypto = require('crypto');
const WebSocket = require('ws');

const DEFAULT_URL = "ws://192.168.1.100:8884/ws1";
const DEFAULT_USERNAME = "vscp";
const DEFAULT_PASSWORD = "secret";
const DEFAULT_KEY_HEX = "A4A86F7D7E119BA3F0CD06881E371B98";
const TIMEOUT_MS = Number(process.env.WS_TIMEOUT_MS || "5000");
const ASYNC_EVENTS_TO_WAIT = Number(process.env.WS_ASYNC_EVENTS || "3");

const CMD_NOOP = 0x0000;
const CMD_QUIT = 0x0001;
const CMD_USER = 0x0002;
const CMD_PASS = 0x0003;
const CMD_CHALLENGE = 0x0004;
const CMD_SEND = 0x0005;
const CMD_RETR = 0x0006;
const CMD_OPEN = 0x0007;
const CMD_CLOSE = 0x0008;
const CMD_CHKDATA = 0x0009;
const CMD_CLEAR = 0x000A;
const CMD_STAT = 0x000B;
const CMD_INFO = 0x000C;
const CMD_GETCHID = 0x000D;
const CMD_SETGUID = 0x000E;
const CMD_GETGUID = 0x000F;
const CMD_VERSION = 0x0010;
const CMD_SETFILTER = 0x0011;
const CMD_SETMASK = 0x0012;
const CMD_INTERFACE = 0x0013;
const CMD_TEST = 0x001E;
const CMD_WCYD = 0x001F;
const CMD_SHUTDOWN = 0x0020;
const CMD_RESTART = 0x0021;
const CMD_TEXT = 0x0022;
const VSCP_ENCRYPTION_AES128 = 0x01;
const VSCP_BINARY_EVENT_HEADER_LENGTH = 35;
const VSCP_HEADER16_FRAME_VERSION_UNIX_NS = 0x0100;
const SAMPLE_EVENT_CLASS = 20;
const SAMPLE_EVENT_TYPE = 3;
const SAMPLE_EVENT_DATA = Buffer.from([0x01, 0x02, 0x03]);
const WS_RECEIVER_STATE = new WeakMap();

/**
 * CRC-CCITT lookup table (polynomial 0x1021, initial 0xFFFF, no final XOR)
 * Used for VSCP binary protocol frame validation
 */
const CRC_TABLE = (() => {
  const table = new Uint16Array(256);
  for (let i = 0; i < 256; i++) {
    let crc = i << 8;
    for (let j = 0; j < 8; j++) {
      crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
      crc &= 0xFFFF;
    }
    table[i] = crc;
  }
  return table;
})();

/**
 * Calculate CRC-CCITT for buffer (compatible with VSCP binary protocol)
 * @param {Buffer} data - Data to calculate CRC over
 * @returns {number} 16-bit CRC value
 */
function calculateCRC(data) {
  let crc = 0xFFFF; // Initial remainder
  for (let i = 0; i < data.length; i++) {
    const byte = data[i];
    const idx = ((crc >> 8) ^ byte) & 0xFF;
    crc = ((crc << 8) ^ CRC_TABLE[idx]) & 0xFFFF;
  }
  return crc; // No final XOR for VSCP
}

function parseCliArgs() {
  const args = process.argv.slice(2);
  const parsed = { url: null, help: false };

  for (let i = 0; i < args.length; i++) {
    const arg = args[i];
    if (arg === "--help" || arg === "-h") {
      parsed.help = true;
      continue;
    }

    if (arg === "--url" && args[i + 1]) {
      parsed.url = args[++i];
      continue;
    }

    if (arg.startsWith("--url=")) {
      parsed.url = arg.substring("--url=".length);
      continue;
    }

    if (!arg.startsWith("-") && !parsed.url) {
      parsed.url = arg;
    }
  }

  return parsed;
}

function toBuffer(payload) {
  if (Buffer.isBuffer(payload)) return payload;
  if (payload instanceof ArrayBuffer) return Buffer.from(payload);
  if (ArrayBuffer.isView(payload)) return Buffer.from(payload.buffer, payload.byteOffset, payload.byteLength);
  return Buffer.from(String(payload), "utf8");
}

function parseWs1Message(message) {
  if (!message || message.length < 1) {
    return { kind: 'unknown', raw: message };
  }

  const kind = message[0];
  const fields = message.split(';');

  if (kind === '+') {
    return {
      kind: 'positive',
      command: fields[1] || '',
      details: fields.slice(2),
      raw: message
    };
  }

  if (kind === '-') {
    return {
      kind: 'negative',
      command: fields[1] || '',
      errorCode: fields[2] || '',
      errorText: fields.slice(3).join(';'),
      raw: message
    };
  }

  return { kind: 'unknown', raw: message };
}

function encryptAuthCredentials(username, password, sidHex, keyHex) {
  if (!/^[0-9a-fA-F]{32}$/.test(sidHex)) {
    throw new Error(`Invalid SID for IV: expected 32 hex chars, got '${sidHex}'`);
  }
  if (!/^[0-9a-fA-F]{32}$/.test(keyHex)) {
    throw new Error('Invalid AES-128 key: expected 32 hex chars');
  }

  const iv = Buffer.from(sidHex, 'hex');
  const key = Buffer.from(keyHex, 'hex');
  const plainCredentials = Buffer.from(`${username}:${password}`, 'utf8');

  const cipher = crypto.createCipheriv('aes-128-cbc', key, iv);
  const encrypted = Buffer.concat([cipher.update(plainCredentials), cipher.final()]);
  return encrypted.toString('hex');
}

function buildBinaryCommandFrame(command, arg = Buffer.alloc(0)) {
  const frame = Buffer.alloc(1 + 2 + arg.length + 2, 0x00);
  frame[0] = 0xE0;
  frame[1] = (command >> 8) & 0xff;
  frame[2] = command & 0xff;
  if (arg.length > 0) {
    arg.copy(frame, 3);
  }
  const crc = calculateCRC(frame.slice(1, frame.length - 2));
  frame[frame.length - 2] = (crc >> 8) & 0xff;
  frame[frame.length - 1] = crc & 0xff;
  return frame;
}

function encryptBinaryFrame(frame, keyHex, encryption = VSCP_ENCRYPTION_AES128) {
  if (!Buffer.isBuffer(frame) || frame.length < 1) {
    throw new Error('Cannot encrypt empty frame');
  }
  if (encryption !== VSCP_ENCRYPTION_AES128) {
    throw new Error(`Unsupported binary frame encryption level: ${encryption}`);
  }
  if (!/^[0-9a-fA-F]{32}$/.test(keyHex)) {
    throw new Error('Invalid AES-128 key: expected 32 hex chars');
  }

  const key = Buffer.from(keyHex, 'hex');
  const iv = crypto.randomBytes(16);
  const payload = frame.subarray(1);
  const padLen = 16 - (payload.length % 16);
  const paddedPayload = Buffer.concat([payload, Buffer.alloc(padLen, 0x00)]);

  const cipher = crypto.createCipheriv('aes-128-cbc', key, iv);
  cipher.setAutoPadding(false);
  const ciphertext = Buffer.concat([cipher.update(paddedPayload), cipher.final()]);

  return Buffer.concat([
    Buffer.from([(frame[0] & 0xf0) | (encryption & 0x0f)]),
    ciphertext,
    iv
  ]);
}

function buildSampleEventFrame() {
  const frame = Buffer.alloc(1 + VSCP_BINARY_EVENT_HEADER_LENGTH + SAMPLE_EVENT_DATA.length + 2, 0x00);
  const timestampNs = BigInt(Date.now()) * 1000000n;

  frame[0] = 0x00;
  frame[1] = (VSCP_HEADER16_FRAME_VERSION_UNIX_NS >> 8) & 0xff;
  frame[2] = VSCP_HEADER16_FRAME_VERSION_UNIX_NS & 0xff;

  for (let idx = 0; idx < 8; idx++) {
    const shift = 56n - (BigInt(idx) * 8n);
    frame[3 + idx] = Number((timestampNs >> shift) & 0xffn);
  }

  frame[11] = 0x00;
  frame[12] = 0x00;
  frame[13] = 0x00;
  frame[14] = (SAMPLE_EVENT_CLASS >> 8) & 0xff;
  frame[15] = SAMPLE_EVENT_CLASS & 0xff;
  frame[16] = (SAMPLE_EVENT_TYPE >> 8) & 0xff;
  frame[17] = SAMPLE_EVENT_TYPE & 0xff;
  frame[33] = 0x01;
  frame[34] = (SAMPLE_EVENT_DATA.length >> 8) & 0xff;
  frame[35] = SAMPLE_EVENT_DATA.length & 0xff;
  SAMPLE_EVENT_DATA.copy(frame, 36);

  const crc = calculateCRC(frame.slice(1, frame.length - 2));
  frame[frame.length - 2] = (crc >> 8) & 0xff;
  frame[frame.length - 1] = crc & 0xff;
  return frame;
}

function parseBinaryReply(buf) {
  if (buf.length < 7) {
    throw new Error(`Reply too short (${buf.length} bytes), expected >= 7`);
  }

  const frameType = buf[0] & 0xf0;
  const command = (buf[1] << 8) | buf[2];
  const error = (buf[3] << 8) | buf[4];
  const receivedCrc = (buf[buf.length - 2] << 8) | buf[buf.length - 1];
  const expectedCrc = calculateCRC(buf.slice(1, buf.length - 2));

  if (frameType !== 0xf0) {
    throw new Error(`Unexpected reply frame type 0x${buf[0].toString(16)}`);
  }
  if (receivedCrc !== expectedCrc) {
    throw new Error(`CRC mismatch: received 0x${receivedCrc.toString(16)}, expected 0x${expectedCrc.toString(16)}`);
  }

  return { command, error };
}

function isBinaryPayload(raw) {
  return Buffer.isBuffer(raw) || raw instanceof ArrayBuffer || ArrayBuffer.isView(raw);
}

function toBinaryBuffer(raw) {
  if (Buffer.isBuffer(raw)) return raw;
  if (raw instanceof ArrayBuffer) return Buffer.from(raw);
  if (ArrayBuffer.isView(raw)) return Buffer.from(raw.buffer, raw.byteOffset, raw.byteLength);
  throw new Error('Payload is not binary');
}

async function waitForBinaryReply(ws, expectedCommand, stepName) {
  while (true) {
    const evt = await wsReceiveWithTimeout(ws, TIMEOUT_MS);

    if (!evt.isBinary) {
      const msgStr = toBuffer(evt.data).toString('utf8');
      console.log(`  <- ${msgStr}`);
      continue;
    }

    const buf = toBinaryBuffer(evt.data);
    const frameType = buf[0] & 0xf0;
    if (frameType === 0x00) {
      console.log(`  <- [async event while waiting for ${stepName}, ${buf.length} bytes]`);
      continue;
    }

    const { command, error } = parseBinaryReply(buf);
    console.log(`  <- [binary reply, ${buf.length} bytes] cmd=0x${command.toString(16).padStart(4, '0')} err=0x${error.toString(16).padStart(4, '0')}`);

    if (command !== expectedCommand) {
      continue;
    }
    if (error !== 0x0000) {
      throw new Error(`${stepName} failed, error=0x${error.toString(16).padStart(4, '0')}`);
    }
    return;
  }
}

async function waitForAsyncEvents(ws, count, label) {
  console.log(`  Waiting for ${count} asynchronous event(s) in ${label}...`);
  let received = 0;

  while (received < count) {
    const evt = await wsReceiveWithTimeout(ws, TIMEOUT_MS * 4);

    if (!evt.isBinary) {
      const msgStr = toBuffer(evt.data).toString('utf8').trim();
      console.log(`  <- ${msgStr}`);
      // WS1 may still deliver async events in text CSV form while binary commands are used.
      if (msgStr.length > 0 && !msgStr.startsWith('+') && !msgStr.startsWith('-')) {
        received += 1;
        console.log(`  <- [async event ${received}/${count}, text]`);
      }
      continue;
    }

    const buf = toBinaryBuffer(evt.data);
    const frameType = buf[0] & 0xf0;
    if (frameType === 0x00) {
      received += 1;
      console.log(`  <- [async event ${received}/${count}, ${buf.length} bytes]`);
    } else {
      const { command, error } = parseBinaryReply(buf);
      console.log(`  <- [binary reply while waiting events] cmd=0x${command.toString(16).padStart(4, '0')} err=0x${error.toString(16).padStart(4, '0')}`);
    }
  }
}

function wsReceiveWithTimeout(ws, timeoutMs) {
  let state = WS_RECEIVER_STATE.get(ws);
  if (!state) {
    state = { queue: [], waiters: [] };
    ws.on('message', (msg, isBinary) => {
      const payload = msg?.data !== undefined ? msg.data : msg;
      const item = { data: payload, isBinary: !!isBinary };
      if (state.waiters.length > 0) {
        const waiter = state.waiters.shift();
        waiter.resolve(item);
      } else {
        state.queue.push(item);
      }
    });
    ws.on('error', (err) => {
      while (state.waiters.length > 0) {
        const waiter = state.waiters.shift();
        waiter.reject(err);
      }
    });
    WS_RECEIVER_STATE.set(ws, state);
  }

  if (state.queue.length > 0) {
    return Promise.resolve(state.queue.shift());
  }

  return new Promise((resolve, reject) => {
    const waiter = {
      resolve: (payload) => {
        clearTimeout(timer);
        resolve(payload);
      },
      reject: (err) => {
        clearTimeout(timer);
        reject(err);
      }
    };

    const timer = setTimeout(() => {
      const idx = state.waiters.indexOf(waiter);
      if (idx >= 0) {
        state.waiters.splice(idx, 1);
      }
      reject(new Error('Timeout waiting for websocket frame'));
    }, timeoutMs);

    state.waiters.push(waiter);
  });
}

async function runScenarioTextThenBinary(url, username, password, keyHex) {
  console.log("\nScenario A: text AUTH/OPEN then binary NOOP");
  const ws = new WebSocket(url);
  await new Promise((resolve, reject) => {
    const t = setTimeout(() => reject(new Error('Timeout opening websocket')), TIMEOUT_MS);
    ws.once('open', () => {
      clearTimeout(t);
      resolve();
    });
    ws.once('error', reject);
  });

  let sid = null;
  let authenticated = false;
  let channelOpen = false;

  try {
    // Some servers don't emit initial AUTH0 automatically.
    console.log('  -> C;CHALLENGE');
    ws.send('C;CHALLENGE');

    while (!channelOpen) {
      const evt = await wsReceiveWithTimeout(ws, TIMEOUT_MS);
      if (evt.isBinary) {
        continue;
      }

      const msgStr = toBuffer(evt.data).toString('utf8');
      const parsed = parseWs1Message(msgStr);
      console.log(`  <- ${msgStr}`);

      if (!sid && parsed.kind === 'positive') {
        const sidCandidate = (parsed.details || []).find((field) => /^[0-9a-fA-F]{32}$/.test(field));
        if (sidCandidate) {
          sid = sidCandidate;
          const authCrypto = encryptAuthCredentials(username, password, sid, keyHex);
          const authCmd = `C;AUTH;${sid};${authCrypto}`;
          console.log(`  -> ${authCmd}`);
          ws.send(authCmd);
          continue;
        }
      }

      if (!authenticated && parsed.kind === 'positive' && (parsed.command || '').toUpperCase() === 'AUTH') {
        authenticated = true;
        console.log('  -> C;OPEN');
        ws.send('C;OPEN');
        continue;
      }

      if (parsed.kind === 'positive' && (parsed.command || '').toUpperCase() === 'OPEN') {
        channelOpen = true;
        continue;
      }

      if (parsed.kind === 'negative') {
        throw new Error(`Command failed: ${msgStr}`);
      }
    }

    ws.send(buildBinaryCommandFrame(CMD_NOOP));
    console.log('  -> [binary NOOP frame, 5 bytes]');
    await waitForBinaryReply(ws, CMD_NOOP, 'NOOP');
    await waitForAsyncEvents(ws, ASYNC_EVENTS_TO_WAIT, 'scenario A');

    ws.send(buildBinaryCommandFrame(CMD_QUIT));
    console.log('  -> [binary QUIT frame, 5 bytes]');
    await waitForBinaryReply(ws, CMD_QUIT, 'QUIT');
  } finally {
    try { ws.close(); } catch {}
  }
}

async function runScenarioBinaryOnly(url, username, password) {
  console.log("\nScenario B: binary USER/PASS/OPEN/NOOP/QUIT");
  const ws = new WebSocket(url);
  await new Promise((resolve, reject) => {
    const t = setTimeout(() => reject(new Error('Timeout opening websocket')), TIMEOUT_MS);
    ws.once('open', () => {
      clearTimeout(t);
      resolve();
    });
    ws.once('error', reject);
  });

  try {
    // Do not wait for initial greeting/challenge; proceed with binary-only commands.

    ws.send(buildBinaryCommandFrame(CMD_USER, Buffer.concat([Buffer.from(username, 'utf8'), Buffer.from([0x00])] )));
    console.log('  -> [binary USER]');
    await waitForBinaryReply(ws, CMD_USER, 'USER');

    ws.send(buildBinaryCommandFrame(CMD_PASS, Buffer.concat([Buffer.from(password, 'utf8'), Buffer.from([0x00])] )));
    console.log('  -> [binary PASS]');
    await waitForBinaryReply(ws, CMD_PASS, 'PASS');

    ws.send(buildBinaryCommandFrame(CMD_OPEN));
    console.log('  -> [binary OPEN]');
    await waitForBinaryReply(ws, CMD_OPEN, 'OPEN');

    ws.send(buildBinaryCommandFrame(CMD_SEND, buildSampleEventFrame()));
    console.log('  -> [binary SEND event]');
    await waitForBinaryReply(ws, CMD_SEND, 'SEND');

    ws.send(buildBinaryCommandFrame(CMD_NOOP));
    console.log('  -> [binary NOOP frame, 5 bytes]');
    await waitForBinaryReply(ws, CMD_NOOP, 'NOOP');
    await waitForAsyncEvents(ws, ASYNC_EVENTS_TO_WAIT, 'scenario B');

    ws.send(buildBinaryCommandFrame(CMD_QUIT));
    console.log('  -> [binary QUIT frame, 5 bytes]');
    await waitForBinaryReply(ws, CMD_QUIT, 'QUIT');
  } finally {
    try { ws.close(); } catch {}
  }
}

async function runScenarioBinaryOnlyEncrypted(url, username, password, keyHex) {
  console.log("\nScenario C: encrypted binary USER/PASS/OPEN + command sweep");
  const ws = new WebSocket(url);
  await new Promise((resolve, reject) => {
    const t = setTimeout(() => reject(new Error('Timeout opening websocket')), TIMEOUT_MS);
    ws.once('open', () => {
      clearTimeout(t);
      resolve();
    });
    ws.once('error', reject);
  });

  try {
    const first = await wsReceiveWithTimeout(ws, TIMEOUT_MS);
    if (!first.isBinary) {
      console.log(`  <- ${toBuffer(first.data).toString('utf8')}`);
    }

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_USER, Buffer.concat([Buffer.from(username, 'utf8'), Buffer.from([0x00])])), keyHex));
    console.log('  -> [encrypted binary USER]');
    await waitForBinaryReply(ws, CMD_USER, 'encrypted USER');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_PASS, Buffer.concat([Buffer.from(password, 'utf8'), Buffer.from([0x00])])), keyHex));
    console.log('  -> [encrypted binary PASS]');
    await waitForBinaryReply(ws, CMD_PASS, 'encrypted PASS');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_OPEN), keyHex));
    console.log('  -> [encrypted binary OPEN]');
    await waitForBinaryReply(ws, CMD_OPEN, 'encrypted OPEN');

    const zeroGuid = Buffer.alloc(16, 0x00);
    const zeroFilter = Buffer.alloc(21, 0x00);
    const ifaceCount = Buffer.from([0x00]);

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_GETCHID), keyHex));
    console.log('  -> [encrypted binary GETCHID]');
    await waitForBinaryReply(ws, CMD_GETCHID, 'encrypted GETCHID');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_GETGUID), keyHex));
    console.log('  -> [encrypted binary GETGUID]');
    await waitForBinaryReply(ws, CMD_GETGUID, 'encrypted GETGUID');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_SETGUID, zeroGuid), keyHex));
    console.log('  -> [encrypted binary SETGUID]');
    await waitForBinaryReply(ws, CMD_SETGUID, 'encrypted SETGUID');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_VERSION), keyHex));
    console.log('  -> [encrypted binary VERSION]');
    await waitForBinaryReply(ws, CMD_VERSION, 'encrypted VERSION');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_STAT), keyHex));
    console.log('  -> [encrypted binary STAT]');
    await waitForBinaryReply(ws, CMD_STAT, 'encrypted STAT');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_INFO), keyHex));
    console.log('  -> [encrypted binary INFO]');
    await waitForBinaryReply(ws, CMD_INFO, 'encrypted INFO');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_WCYD), keyHex));
    console.log('  -> [encrypted binary WCYD]');
    await waitForBinaryReply(ws, CMD_WCYD, 'encrypted WCYD');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_CHKDATA), keyHex));
    console.log('  -> [encrypted binary CHKDATA]');
    await waitForBinaryReply(ws, CMD_CHKDATA, 'encrypted CHKDATA');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_CLEAR), keyHex));
    console.log('  -> [encrypted binary CLEAR]');
    await waitForBinaryReply(ws, CMD_CLEAR, 'encrypted CLEAR');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_SETFILTER, zeroFilter), keyHex));
    console.log('  -> [encrypted binary SETFILTER]');
    await waitForBinaryReply(ws, CMD_SETFILTER, 'encrypted SETFILTER');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_SETMASK, zeroFilter), keyHex));
    console.log('  -> [encrypted binary SETMASK]');
    await waitForBinaryReply(ws, CMD_SETMASK, 'encrypted SETMASK');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_INTERFACE, ifaceCount), keyHex));
    console.log('  -> [encrypted binary INTERFACE]');
    await waitForBinaryReply(ws, CMD_INTERFACE, 'encrypted INTERFACE');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_TEST), keyHex));
    console.log('  -> [encrypted binary TEST]');
    await waitForBinaryReply(ws, CMD_TEST, 'encrypted TEST');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_CHALLENGE), keyHex));
    console.log('  -> [encrypted binary CHALLENGE]');
    await waitForBinaryReply(ws, CMD_CHALLENGE, 'encrypted CHALLENGE');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_SEND, buildSampleEventFrame()), keyHex));
    console.log('  -> [encrypted binary SEND event]');
    await waitForBinaryReply(ws, CMD_SEND, 'encrypted SEND');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_NOOP), keyHex));
    console.log('  -> [encrypted binary NOOP frame]');
    await waitForBinaryReply(ws, CMD_NOOP, 'encrypted NOOP');
    await waitForAsyncEvents(ws, ASYNC_EVENTS_TO_WAIT, 'scenario C');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_CLOSE), keyHex));
    console.log('  -> [encrypted binary CLOSE]');
    await waitForBinaryReply(ws, CMD_CLOSE, 'encrypted CLOSE');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_RETR), keyHex));
    console.log('  -> [encrypted binary RETR]');
    await waitForBinaryReply(ws, CMD_RETR, 'encrypted RETR');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_OPEN), keyHex));
    console.log('  -> [encrypted binary OPEN(reopen)]');
    await waitForBinaryReply(ws, CMD_OPEN, 'encrypted OPEN(reopen)');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_SHUTDOWN), keyHex));
    console.log('  -> [encrypted binary SHUTDOWN]');
    await waitForBinaryReply(ws, CMD_SHUTDOWN, 'encrypted SHUTDOWN');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_TEXT), keyHex));
    console.log('  -> [encrypted binary TEXT]');
    await waitForBinaryReply(ws, CMD_TEXT, 'encrypted TEXT');

    ws.send(encryptBinaryFrame(buildBinaryCommandFrame(CMD_RESTART), keyHex));
    console.log('  -> [encrypted binary RESTART]');
    await waitForBinaryReply(ws, CMD_RESTART, 'encrypted RESTART');
  } finally {
    try { ws.close(); } catch {}
  }
}

async function main() {
  const cli = parseCliArgs();

  if (cli.help) {
    console.log("Usage: node test_vscp_binary.js [url] [--url <url>]");
    console.log("Env: WS_BINARY_URL, WS1_URL, WS_TIMEOUT_MS");
    console.log("Auth env: VSCP_USERNAME, VSCP_PASSWORD, VSCP_KEY16");
    process.exit(0);
  }

  const url = cli.url || process.env.WS_BINARY_URL || process.env.WS1_URL || DEFAULT_URL;
  const username = process.env.VSCP_USERNAME || DEFAULT_USERNAME;
  const password = process.env.VSCP_PASSWORD || DEFAULT_PASSWORD;
  const keyHex = (process.env.VSCP_KEY16 || DEFAULT_KEY_HEX).trim();

  console.log(`Connecting to ${url}`);
  await runScenarioTextThenBinary(url, username, password, keyHex);
  await runScenarioBinaryOnly(url, username, password);
  await runScenarioBinaryOnlyEncrypted(url, username, password, keyHex);
  console.log("\nPASS: Binary NOOP test passed in all scenarios (text-assisted, binary-only, encrypted-binary)");
}

main().catch((err) => {
  console.error(`\nFAIL: ${err.message || err}`);
  process.exit(1);
});

