/*!
  Example code for node.js and the VSCP daemon ws1 websocket interface
  send 0,30,5,,,0,-,15,14,13,12,11,10,9,8,7,6,5,4,3,2,0,0,1,35
  Code from: https://github.com/websockets/ws

  Copyright 2020-2026 Ake Hedman, the VSCP project - MIT license
*/

WebSocket = require('ws');
const crypto = require('crypto');

const defaultUrl = 'ws://192.168.1.104:8884/ws1';
const DEFAULT_USERNAME = 'vscp';
const DEFAULT_PASSWORD = 'secret';
const DEFAULT_KEY_HEX = 'A4A86F7D7E119BA3F0CD06881E371B98';

function parseCliArgs() {
  const args = process.argv.slice(2);
  const parsed = {
    help: false,
    url: null,
    username: null,
    password: null,
    keyHex: null,
    positionals: []
  };

  for (let index = 0; index < args.length; index++) {
    const arg = args[index];

    if (arg === '--help' || arg === '-h') {
      parsed.help = true;
      continue;
    }

    if (arg === '--url' && args[index + 1]) {
      parsed.url = args[++index];
      continue;
    }

    if (arg.startsWith('--url=')) {
      parsed.url = arg.substring('--url='.length);
      continue;
    }

    if (arg === '--user' && args[index + 1]) {
      parsed.username = args[++index];
      continue;
    }

    if (arg.startsWith('--user=')) {
      parsed.username = arg.substring('--user='.length);
      continue;
    }

    if (arg === '--password' && args[index + 1]) {
      parsed.password = args[++index];
      continue;
    }

    if (arg.startsWith('--password=')) {
      parsed.password = arg.substring('--password='.length);
      continue;
    }

    if (arg === '--key' && args[index + 1]) {
      parsed.keyHex = args[++index];
      continue;
    }

    if (arg.startsWith('--key=')) {
      parsed.keyHex = arg.substring('--key='.length);
      continue;
    }

    if (!arg.startsWith('-')) {
      parsed.positionals.push(arg);
    }
  }

  return parsed;
}

function getAuthConfig() {
  const cli = parseCliArgs();

  if (cli.help) {
    console.log('Usage: node test_ws1.js [url] [--url <url>] [--user <username>] [--password <password>] [--key <aes128-hex>]');
    console.log('Example: node test_ws1.js --url ws://192.168.1.100:8884/ws1 --user admin --password secret --key A4A86F7D7E119BA3F0CD06881E371B98');
    console.log('Env: WS1_URL, WS1_USERNAME, WS1_PASSWORD, WS1_KEY (fallback: VSCP_USERNAME, VSCP_PASSWORD, VSCP_KEY16)');
    process.exit(0);
  }

  const wsUrl = cli.url || cli.positionals[0] || process.env.WS1_URL || defaultUrl;
  const username = cli.username || process.env.WS1_USERNAME || process.env.VSCP_USERNAME || DEFAULT_USERNAME;
  const password = cli.password || process.env.WS1_PASSWORD || process.env.VSCP_PASSWORD || DEFAULT_PASSWORD;
  const keyHex = (cli.keyHex || process.env.WS1_KEY || process.env.VSCP_KEY16 || DEFAULT_KEY_HEX).trim();

  return {
    wsUrl,
    username,
    password,
    keyHex
  };
}

function isValidHex(value, expectedLength) {
  return typeof value === 'string' && value.length === expectedLength && /^[0-9a-fA-F]+$/.test(value);
}

function encryptAuthCredentials(username, password, sidHex, keyHex) {
  if (!isValidHex(sidHex, 32)) {
    throw new Error(`Invalid SID for IV: expected 32 hex chars, got '${sidHex}'`);
  }

  if (!isValidHex(keyHex, 32)) {
    throw new Error('Invalid AES-128 key: expected 32 hex chars');
  }

  const iv = Buffer.from(sidHex, 'hex');
  const key = Buffer.from(keyHex, 'hex');
  const plainCredentials = Buffer.from(`${username}:${password}`, 'utf8');

  const cipher = crypto.createCipheriv('aes-128-cbc', key, iv);
  const encrypted = Buffer.concat([cipher.update(plainCredentials), cipher.final()]);
  return encrypted.toString('hex');
}

const authConfig = getAuthConfig();
const wsUrl = authConfig.wsUrl;
console.log(`Connecting to ${wsUrl}`);

const ws = new WebSocket(wsUrl, {
  perMessageDeflate: false
});

const REQUEST_TIMEOUT_MS = 5000;
let pendingRequest = null;
let pendingSidResolver = null;

function toTextMessage(payload) {
  if (typeof payload === 'string') {
    return payload;
  }

  if (Buffer.isBuffer(payload)) {
    return payload.toString('utf8');
  }

  if (payload instanceof ArrayBuffer) {
    return Buffer.from(payload).toString('utf8');
  }

  if (ArrayBuffer.isView(payload)) {
    return Buffer.from(payload.buffer, payload.byteOffset, payload.byteLength).toString('utf8');
  }

  return String(payload);
}

function parseWs1Event(eventPayload) {
  const parts = eventPayload.split(',');
  if (parts.length < 7) {
    return null;
  }

  return {
    head: parts[0],
    vscpClass: parts[1],
    vscpType: parts[2],
    obid: parts[3],
    datetime: parts[4],
    timestamp: parts[5],
    guid: parts[6],
    data: parts.slice(7)
  };
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

  if (kind === 'E') {
    return {
      kind: 'event',
      event: parseWs1Event(fields.slice(1).join(';')),
      raw: message
    };
  }

  return { kind: 'unknown', raw: message };
}

function getWs1CommandFromFrame(frame) {
  const fields = frame.split(';');
  if (fields.length >= 2 && fields[0] === 'C') {
    return (fields[1] || '').toUpperCase();
  }

  return '';
}

function matchesExpectedCommand(parsed, expectedCommand) {
  if (!expectedCommand) {
    return true;
  }

  return (parsed.command || '').toUpperCase() === expectedCommand.toUpperCase();
}

function sendAndWait(commandFrame, expectedCommand) {
  if (pendingRequest) {
    return Promise.reject(new Error('A command is already waiting for response'));
  }

  const expected = expectedCommand || getWs1CommandFromFrame(commandFrame);

  return new Promise((resolve, reject) => {
    const timeoutId = setTimeout(() => {
      pendingRequest = null;
      reject(new Error(`Timeout waiting for response to ${commandFrame}`));
    }, REQUEST_TIMEOUT_MS);

    pendingRequest = {
      expected,
      resolve: (parsed) => {
        clearTimeout(timeoutId);
        pendingRequest = null;
        resolve(parsed);
      },
      reject: (error) => {
        clearTimeout(timeoutId);
        pendingRequest = null;
        reject(error);
      }
    };

    ws.send(commandFrame);
  });
}

function extractSid(parsed) {
  if (!parsed || parsed.kind !== 'positive') {
    return null;
  }

  const sidCandidate = (parsed.details || []).find((field) => /^[0-9a-fA-F]{32}$/.test(field));
  return sidCandidate || null;
}

function waitForSid() {
  if (pendingSidResolver) {
    return Promise.reject(new Error('Already waiting for SID'));
  }

  return new Promise((resolve, reject) => {
    let challengeSent = false;
    let timeoutId;

    function armTimeout() {
      timeoutId = setTimeout(() => {
        if (!challengeSent) {
          challengeSent = true;
          console.log('* * * SID wait timed out, requesting CHALLENGE...');
          ws.send('C;CHALLENGE');
          armTimeout();
          return;
        }

        pendingSidResolver = null;
        reject(new Error('Timeout waiting for SID (including CHALLENGE retry)'));
      }, REQUEST_TIMEOUT_MS);
    }

    pendingSidResolver = {
      resolve: (sid) => {
        clearTimeout(timeoutId);
        pendingSidResolver = null;
        resolve(sid);
      },
      reject: (error) => {
        clearTimeout(timeoutId);
        pendingSidResolver = null;
        reject(error);
      }
    };

    armTimeout();
  });
}

ws.on('open', async function open() {
  const cmdnoop = "C;NOOP";
  const cmdopen = "C;OPEN";

  // 'E';head,vscp_class,vscp_type,obid,datetime,timestamp,GUID,data
  const sendEvent = "E;0,30,5,0,2020-01-29T23:05:59Z,0,FF:FF:FF:FF:FF:FF:FF:F5:00:00:00:00:00:02:00:00,1,2,3,4,5,6";

  // Receive only CLASS1.CONTROL, TurnOn

  // C;SF;filter-priority, filter-class, 
  //  filter-type, filter-GUID;mask-priority, 
  //  mask-class, mask-type, mask-GUID”
  const cmdFilter = "C;SF;0,30,5,00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00;0,0xffff,0xffff,00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00";

  try {
    console.log("\n* * * Waiting for SID...");
    const sid = await waitForSid();
    console.log(`SID       : ${sid}`);

    const authCrypto = encryptAuthCredentials(authConfig.username, authConfig.password, sid, authConfig.keyHex);
    const cmdauth = `C;AUTH;${sid};${authCrypto}`;

    console.log(`\n* * * Logging in as '${authConfig.username}'.`);
    console.log(`AUTH IV   : ${sid}`);
    console.log(`AUTH CRYPT: ${authCrypto}`);
    await sendAndWait(cmdauth, 'AUTH');

    console.log("\n* * * Sending NOOP command.");
    console.log(cmdnoop);
    await sendAndWait(cmdnoop, 'NOOP');

    console.log("\n* * * Open communication channel.");
    console.log(cmdopen);
    await sendAndWait(cmdopen, 'OPEN');

    console.log("\n* * * Set filter.");
    console.log(cmdFilter);
    await sendAndWait(cmdFilter, 'SF');

    console.log("\n* * * Send event.");
    console.log(sendEvent);
    ws.send(sendEvent);
  }
  catch (error) {
    console.error(`Command sequence failed: ${error.message}`);
    ws.close();
    return;
  }

  console.log("---------------------------------------------------------");
  console.log("Waiting for events (Abort with ctrl+c)")

  console.log("Will only receive CLASS1.CONTROL, TurnOn now after filter");
  console.log("is set");
  console.log("---------------------------------------------------------");

});
  
ws.on('message', function incoming(data) {
  const message = toTextMessage(data);
  const parsed = parseWs1Message(message);

  console.log("\n* * * Raw message received:", message);

  if (parsed.kind === 'positive') {
    console.log("\n* * * Positive response");
    console.log(`Command   : ${parsed.command}`);
    if (parsed.details.length) {
      console.log(`Details   : ${parsed.details.join(' ; ')}`);
    }
  }
  else if (parsed.kind === 'negative') {
    console.log("\n* * * Negative response");
    console.log(`Command   : ${parsed.command}`);
    console.log(`ErrorCode : ${parsed.errorCode}`);
    console.log(`ErrorText : ${parsed.errorText}`);
  }
  else if (parsed.kind === 'event') {
    console.log("\n* * * Event received");
    if (parsed.event) {
      console.log(`Class/Type: ${parsed.event.vscpClass}/${parsed.event.vscpType}`);
      console.log(`Head/ObId : ${parsed.event.head}/${parsed.event.obid}`);
      console.log(`Date/Time : ${parsed.event.datetime}`);
      console.log(`Timestamp : ${parsed.event.timestamp}`);
      console.log(`GUID      : ${parsed.event.guid}`);
      console.log(`Data      : [${parsed.event.data.join(', ')}]`);
    }
    else {
      console.log(`Event raw : ${parsed.raw}`);
    }
  }
  else {
    console.log("\n* * * Message received");
    console.log(parsed.raw);
  }

  if (pendingSidResolver) {
    const sid = extractSid(parsed);
    if (sid) {
      pendingSidResolver.resolve(sid);
      return;
    }

    if (parsed.kind === 'negative' && (parsed.command || '').toUpperCase() === 'CHALLENGE') {
      pendingSidResolver.reject(
        new Error(`Failed to get SID: ${parsed.errorCode} ${parsed.errorText}`)
      );
      return;
    }
  }

  if (pendingRequest && (parsed.kind === 'positive' || parsed.kind === 'negative')) {
    if (!matchesExpectedCommand(parsed, pendingRequest.expected)) {
      return;
    }

    if (parsed.kind === 'positive') {
      pendingRequest.resolve(parsed);
    }
    else {
      pendingRequest.reject(
        new Error(`Negative response for ${parsed.command}: ${parsed.errorCode} ${parsed.errorText}`)
      );
    }
  }
});
