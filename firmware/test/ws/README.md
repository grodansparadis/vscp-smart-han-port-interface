# Test code for the Websocket interface

This folder contains files for websocket tests

See also the top-level firmware README: [../../README.md](../../README.md)

## mintest_ws1.html
Simplest possible login test for VSCP ws1 websocket interface.

## test_ws1.py
Python (ver 3) code to login on the websocket interface and perform
some VSCP ws1 websocket commands and then wait for incoming events. User, password and key should be set to default values.

## test_ws2.py
Python (ver 3) code to login on the websocket interface and perform
some VSCP ws2 websocket commands and then wait for incoming events. User, password and key should be set to default values.

## test_ws1.js
node.js code to login on the websocket interface and perform
some VSCP ws1 websocket commands and then wait for incoming events. User, password and key should be set to default values.

## test_ws2.js
node.js code to login on the websocket interface and perform
some VSCP ws2 websocket commands and then wait for incoming events. User, password and key should be set to default values.

## test_vscp_binary.js
Node.js smoke test for VSCP binary protocol over `ws1`. Authenticates on ws1 (text mode), opens the channel, then tests binary NOOP command with CRC-CCITT validation.

**Flow:**
1. Receive CHALLENGE from server (extract SID)
2. Send AUTH command with AES-128-CBC encrypted credentials
3. Send OPEN command to open communication channel
4. Send binary NOOP frame (Frame format=14)
5. Receive and validate binary reply (Frame format=15) with CRC-CCITT

**Frame Format:**
- Command: `[type(0xE0) | command(2 bytes) | CRC-CCITT(2 bytes)]`
- Reply: `[type(0xF0) | command(2 bytes) | error(2 bytes) | CRC-CCITT(2 bytes)]`
- CRC: CRC-CCITT (polynomial 0x1021, initial 0xFFFF, no final XOR)

**Environment Variables:**
- `WS_BINARY_URL` or `WS1_URL`: WebSocket endpoint (default: ws://192.168.1.100:8884/ws1)
- `WS_TIMEOUT_MS`: Timeout in milliseconds (default: 5000)
- `VSCP_USERNAME`: Authentication username (default: vscp)
- `VSCP_PASSWORD`: Authentication password (default: secret)
- `VSCP_KEY16`: AES-128 encryption key in hex (default: A4A86F7D7E119BA3F0CD06881E371B98)

## test_vscp_binary.py
Python 3 smoke test for VSCP binary protocol over `ws1`. Same authentication and binary protocol flow as the JavaScript version.

**Dependencies:**
- `websockets` (websocket client)
- `cryptography` (AES-128-CBC encryption)

Install with: `pip install websockets cryptography`

**Environment Variables:** Same as JavaScript test (WS_BINARY_URL, VSCP_USERNAME, etc.)

## test_vscp_binary.c
Standalone C smoke test for VSCP binary protocol over `ws1`, mirroring the Python test scenarios:

1. Scenario A: text `CHALLENGE` + `AUTH` + `OPEN`, then binary `NOOP`/`QUIT`
2. Scenario B: binary `USER`/`PASS`/`OPEN`/`SEND`/`NOOP`/`QUIT`
3. Scenario C: encrypted binary `USER`/`PASS`/`OPEN`/`SEND`/`NOOP`/`QUIT`

It implements:

- RFC6455 websocket handshake/framing over `ws://`
- CRC-CCITT validation for VSCP binary replies
- AES-128-CBC auth encryption (`AUTH`)
- AES-128-CBC binary frame encryption with zero padding + appended IV (same frame model as Python test)

Build (requires OpenSSL dev package):

```bash
gcc -std=c11 -O2 -Wall -Wextra -o test_vscp_binary_c test_vscp_binary.c -lcrypto
```

Run:

```bash
./test_vscp_binary_c --url ws://192.168.1.104:8884/ws1
```

Environment variables:

- `WS_BINARY_URL` or `WS1_URL`
- `WS_TIMEOUT_S`
- `WS_ASYNC_EVENTS`
- `VSCP_USERNAME`
- `VSCP_PASSWORD`
- `VSCP_KEY16`

## AUTH encryption (JS and Python scripts)

`test_ws1.js`, `test_ws2.js`, `test_ws1.py`, and `test_ws2.py` now build `AUTH` dynamically using:

- AES-128-CBC
- IV = received SID (from connect response / challenge)
- Encrypted payload = `username;password`

Credential/key sources (highest to lowest):

1. CLI options: `--user`, `--password`, `--key`
2. WS-specific env vars: `WS1_USERNAME`/`WS2_USERNAME`, `WS1_PASSWORD`/`WS2_PASSWORD`, `WS1_KEY`/`WS2_KEY`
3. Generic env vars: `VSCP_USERNAME`, `VSCP_PASSWORD`, `VSCP_KEY16`
4. Defaults: `admin`, `secret`, `A4A86F7D7E119BA3F0CD06881E371B98`

Examples:

- `node test_ws1.js --url ws://192.168.1.50:8884/ws1 --user admin --password secret --key A4A86F7D7E119BA3F0CD06881E371B98`
- `WS2_URL=ws://192.168.1.50:8884/ws2 WS2_USERNAME=admin WS2_PASSWORD=secret WS2_KEY=A4A86F7D7E119BA3F0CD06881E371B98 node test_ws2.js`
- `python test_ws1.py --url ws://192.168.1.50:8884/ws1 --user admin --password secret --key A4A86F7D7E119BA3F0CD06881E371B98`
- `WS2_URL=ws://192.168.1.50:8884/ws2 WS2_USERNAME=admin WS2_PASSWORD=secret WS2_KEY=A4A86F7D7E119BA3F0CD06881E371B98 python test_ws2.py`

### Quick start auth test

```bash
# WS1 (Node.js)
node test_ws1.js --url ws://192.168.1.50:8884/ws1 --user admin --password secret --key A4A86F7D7E119BA3F0CD06881E371B98

# WS2 (Node.js)
node test_ws2.js --url ws://192.168.1.50:8884/ws2 --user admin --password secret --key A4A86F7D7E119BA3F0CD06881E371B98

# WS1 (Python)
python test_ws1.py --url ws://192.168.1.50:8884/ws1 --user admin --password secret --key A4A86F7D7E119BA3F0CD06881E371B98

# WS2 (Python)
python test_ws2.py --url ws://192.168.1.50:8884/ws2 --user admin --password secret --key A4A86F7D7E119BA3F0CD06881E371B98
```

### Python AES backend note

The Python scripts print the AES backend at startup:

- `AES backend : cryptography`
- `AES backend : pycryptodome`
- `AES backend : missing (install cryptography or pycryptodome)`

If backend is missing, install one of:

- `pip install cryptography`
- `pip install pycryptodome`

## URL override for all CLI test scripts

The following scripts support URL override:

- `test_ws1.js`
- `test_ws2.js`
- `test_ws1.py`
- `test_ws2.py`

URL source precedence (highest to lowest):

1. `--url <url>`
2. `--url=<url>`
3. Positional URL argument
4. Environment variable (`WS1_URL` or `WS2_URL`)
5. Script built-in default URL

Examples:

- `node test_ws1.js --url ws://192.168.1.50:8884/ws1`
- `WS2_URL=ws://192.168.1.50:8884/ws2 node test_ws2.js`
- `python test_ws1.py ws://192.168.1.50:8884/ws1`
- `WS2_URL=ws://192.168.1.50:8884/ws2 python test_ws2.py`

Binary protocol smoke test examples:

- `node test_vscp_binary.js --url ws://192.168.1.50:8884/ws1`
- `WS_BINARY_URL=ws://192.168.1.50:8884/ws1 node test_vscp_binary.js`
- `python test_vscp_binary.py --url ws://192.168.1.50:8884/ws1`
- `WS_BINARY_URL=ws://192.168.1.50:8884/ws1 python test_vscp_binary.py`

Binary protocol test with custom credentials:

- `VSCP_USERNAME=admin VSCP_PASSWORD=admin123 VSCP_KEY16=A4A86F7D7E119BA3F0CD06881E371B98 node test_vscp_binary.js`
- `VSCP_USERNAME=admin VSCP_PASSWORD=admin123 python test_vscp_binary.py ws://192.168.1.50:8884/ws1`

Expected output when test succeeds:

```
Connecting to ws://192.168.1.100:8884/ws1
  Connected, waiting for CHALLENGE from server...
  <- +;CHALLENGE;<sid>
  SID received: <sid>
  -> C;AUTH;<sid>;<encrypted>
  <- +;AUTH
  Authenticated successfully
  -> C;OPEN
  <- +;OPEN
  Channel opened successfully
  Switching to binary protocol...
  -> [binary NOOP frame, 5 bytes]
  <- [binary reply, 7 bytes]
     Type: 0xF0, Command: 0x0000, Error: 0x0000

PASS: Binary NOOP protocol test successful
```