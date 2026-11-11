/*!
  Example code for node.js and the VSCP daemon ws2 websocket interface
  send 0,30,5,,,0,-,15,14,13,12,11,10,9,8,7,6,5,4,3,2,0,0,1,35
  Code from: https://github.com/websockets/ws

  Copyright 2020 Ake Hedman, the VSCP project - MIT license
*/

WebSocket = require('ws');
const crypto = require('crypto');

const defaultUrl = 'ws://localhost:8884/ws2';
const DEFAULT_USERNAME = 'admin';
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
    console.log('Usage: node test_ws2.js [url] [--url <url>] [--user <username>] [--password <password>] [--key <aes128-hex>]');
    console.log('Example: node test_ws2.js --url ws://192.168.1.100:8884/ws2 --user admin --password secret --key A4A86F7D7E119BA3F0CD06881E371B98');
    console.log('Env: WS2_URL, WS2_USERNAME, WS2_PASSWORD, WS2_KEY (fallback: VSCP_USERNAME, VSCP_PASSWORD, VSCP_KEY16)');
    process.exit(0);
  }

  const wsUrl = cli.url || cli.positionals[0] || process.env.WS2_URL || defaultUrl;
  const username = cli.username || process.env.WS2_USERNAME || process.env.VSCP_USERNAME || DEFAULT_USERNAME;
  const password = cli.password || process.env.WS2_PASSWORD || process.env.VSCP_PASSWORD || DEFAULT_PASSWORD;
  const keyHex = (cli.keyHex || process.env.WS2_KEY || process.env.VSCP_KEY16 || DEFAULT_KEY_HEX).trim();

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
  const plainCredentials = Buffer.from(`${username};${password}`, 'utf8');

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

function printWs2Message(reply, rawMessage) {
  if (!reply || typeof reply !== 'object') {
    console.log("\n* * * Message received");
    console.log(rawMessage);
    return;
  }

  const msgType = reply.type || 'unknown';
  console.log(`\n* * * WS2 message (${msgType})`);

  if (msgType === 'cmd') {
    console.log(`Command   : ${reply.command || '(none)'}`);
    if (reply.result !== undefined) {
      console.log(`Result    : ${reply.result}`);
    }
    if (reply.code !== undefined) {
      console.log(`Code      : ${reply.code}`);
    }
    if (reply.message) {
      console.log(`Message   : ${reply.message}`);
    }
    if (reply.args !== undefined && reply.args !== null) {
      console.log(`Args      : ${JSON.stringify(reply.args)}`);
    }
    return;
  }

  if (msgType === 'event') {
    const event = reply.event || {};
    console.log(`Class/Type: ${event.vscpClass ?? event.class ?? '?'}/${event.vscpType ?? event.type ?? '?'}`);
    console.log(`Head/ObId : ${event.vscpHead ?? event.head ?? '?'}/${event.vscpObId ?? event.obid ?? '?'}`);
    console.log(`Date/Time : ${event.vscpDateTime ?? event.datetime ?? '?'}`);
    console.log(`Timestamp : ${event.vscpTimeStamp ?? event.timestamp ?? '?'}`);
    console.log(`GUID      : ${event.vscpGuid ?? event.guid ?? '?'}`);
    if (Array.isArray(event.vscpData || event.data)) {
      console.log(`Data      : [${(event.vscpData || event.data).join(', ')}]`);
    }
    return;
  }

  if (msgType === 'error') {
    console.log(`Code      : ${reply.code ?? '(none)'}`);
    console.log(`Message   : ${reply.message ?? '(none)'}`);
    return;
  }

  console.log(JSON.stringify(reply, null, 2));
}

function getCommandFromFrame(frame) {
  if (!frame || frame.type !== 'cmd') {
    return '';
  }

  return String(frame.command || '').toLowerCase();
}

function commandMatches(reply, expectedCommand) {
  if (!expectedCommand) {
    return true;
  }

  if (!reply || reply.type !== 'cmd') {
    return false;
  }

  return String(reply.command || '').toLowerCase() === expectedCommand.toLowerCase();
}

function findSidValue(value) {
  if (typeof value === 'string' && /^[0-9a-fA-F]{32}$/.test(value)) {
    return value;
  }

  if (Array.isArray(value)) {
    for (const item of value) {
      const sid = findSidValue(item);
      if (sid) {
        return sid;
      }
    }
  }
  else if (value && typeof value === 'object') {
    for (const objectValue of Object.values(value)) {
      const sid = findSidValue(objectValue);
      if (sid) {
        return sid;
      }
    }
  }

  return null;
}

function extractSidFromReply(reply) {
  if (!reply || typeof reply !== 'object') {
    return null;
  }

  return findSidValue(reply);
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
          ws.send(JSON.stringify({
            type: 'cmd',
            command: 'challenge',
            args: null
          }));
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

function sendAndWait(commandFrame, expectedCommand) {
  if (pendingRequest) {
    return Promise.reject(new Error('A command is already waiting for response'));
  }

  const expected = expectedCommand || getCommandFromFrame(commandFrame);

  return new Promise((resolve, reject) => {
    const timeoutId = setTimeout(() => {
      pendingRequest = null;
      reject(new Error(`Timeout waiting for response to command: ${expected || '(unknown)'}`));
    }, REQUEST_TIMEOUT_MS);

    pendingRequest = {
      expected,
      resolve: (reply) => {
        clearTimeout(timeoutId);
        pendingRequest = null;
        resolve(reply);
      },
      reject: (error) => {
        clearTimeout(timeoutId);
        pendingRequest = null;
        reject(error);
      }
    };

    ws.send(JSON.stringify(commandFrame));
  });
}

ws.on('open', async function open() {
  const cmdnoop = {
    "type": "cmd",
    "command": "noop",
    "args": null
  };

  const cmdver = {
    "type": "cmd",
    "command": "version",
    "args": null
  };

  const cmdcopy = {
    "type": "cmd",
    "command": "copyright",
    "args": null
  };

  const cmdopen = {
    "type": "cmd",
    "command": "open",
    "args": null
  };

  const sendEvent = {
    "type": "event",
    "event" : {
        "vscpHead" : 0,
        "vscpObId": 0,
        "vscpDateTime": "2020-01-29T23:05:59Z",
        "vscpTimeStamp": 0,
        "vscpClass": 30,
        "vscpType": 5,
        "vscpGuid": "FF:FF:FF:FF:FF:FF:FF:F5:00:00:00:00:00:02:00:00",
        "vscpData": [1,2,3,4,5,6]
    }
  };

  // Receive only CLASS1.CONTROL, TurnOn
  const cmdFilter = {
    "type": "cmd",
    "command": "setfilter",
    "args": {                                                                  
        "mask_priority": 0,
        "mask_class": 65535,
        "mask_type": 65535,                                                     
        "mask_guid": "00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00",                                               
        "filter_priority": 0,                                             
        "filter_class": 30,                                                  
        "filter_type": 5,                                                   
        "filter_guid": "00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00",
    }
  };

  try {
    console.log("\n* * * Waiting for SID...");
    const sid = await waitForSid();
    console.log(`SID       : ${sid}`);

    const authCrypto = encryptAuthCredentials(authConfig.username, authConfig.password, sid, authConfig.keyHex);
    const cmdauth = {
      "type": "cmd",
      "command": "auth",
      "args": {
         "iv": sid,
         "crypto": authCrypto
      }
    };

    console.log(`\n* * * Logging in as '${authConfig.username}'.`);
    console.log(`AUTH IV   : ${sid}`);
    console.log(`AUTH CRYPT: ${authCrypto}`);
    console.log(JSON.stringify(cmdauth, null, 2 ));
    await sendAndWait(cmdauth, 'auth');

    console.log("\n* * * Sending NOOP command.");
    console.log(JSON.stringify(cmdnoop, null, 2 ));
    await sendAndWait(cmdnoop, 'noop');

    console.log("\n* * * Sending VERSION command.");
    console.log(JSON.stringify(cmdver, null, 2 ));
    await sendAndWait(cmdver, 'version');

    console.log("\n* * * Sending COPYRIGHT command.");
    console.log(JSON.stringify(cmdcopy, null, 2 ));
    await sendAndWait(cmdcopy, 'copyright');

    console.log("\n* * * Open communication channel.");
    console.log(JSON.stringify(cmdopen, null, 2 ));
    await sendAndWait(cmdopen, 'open');

    console.log("\n* * * Set filter.");
    console.log(JSON.stringify(cmdFilter, null, 2 ));
    await sendAndWait(cmdFilter, 'setfilter');

    console.log("\n* * * Send event.");
    console.log(JSON.stringify(sendEvent, null, 2 ));
    ws.send(JSON.stringify(sendEvent));
  }
  catch (error) {
    console.error(`Command sequence failed: ${error.message}`);
    ws.close();
    return;
  }

  console.log("---------------------------------------------------------");
  console.log("Waiting for events (Abort with ctrl+c)")

  console.log("Will only receive CLASS1.CONTROL(30), TurnOn(5) now after filter");
  console.log("is set");
  console.log("---------------------------------------------------------");

});
  
ws.on('message', function incoming(data) {
  const message = toTextMessage(data);

  try {
    const reply = JSON.parse(message);
    printWs2Message(reply, message);

    if (pendingSidResolver) {
      const sid = extractSidFromReply(reply);
      if (sid) {
        pendingSidResolver.resolve(sid);
        return;
      }
    }

    if (pendingRequest) {
      if (reply.type === 'error') {
        pendingRequest.reject(new Error(`Error response: ${reply.code ?? ''} ${reply.message ?? ''}`.trim()));
        return;
      }

      if (commandMatches(reply, pendingRequest.expected)) {
        if (reply.code !== undefined && Number(reply.code) !== 0) {
          pendingRequest.reject(new Error(`Command failed (${reply.command}): code=${reply.code} message=${reply.message ?? ''}`.trim()));
        }
        else {
          pendingRequest.resolve(reply);
        }
      }
    }
  }
  catch (error) {
    console.log("\n* * * Non-JSON response:");
    console.log(message);
  }
});