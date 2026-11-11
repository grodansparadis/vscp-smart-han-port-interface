#!/usr/bin/env python

# VSCP ws2 client example  (Need python3)
# Demonstrates the use of the ws2 websocket interface of the VSCP daemon
# Sample event to send
#    send 0,20,3,,,0,-,15,14,13,12,11,10,9,8,7,6,5,4,3,2,0,0,1,35
# Original sample from: https://websockets.readthedocs.io/en/stable/intro.html
# Copyright 2020 Ake Hedman, the VSCP project - MIT license

from signal import signal, SIGINT
from sys import exit
import sys
import os
import asyncio
import pathlib
import ssl
import websockets
import base64
import json
import re

DEFAULT_URL = 'ws://localhost:8884/ws2'
DEFAULT_USERNAME = 'admin'
DEFAULT_PASSWORD = 'secret'
DEFAULT_KEY_HEX = 'A4A86F7D7E119BA3F0CD06881E371B98'

HEX_32_RE = re.compile(r'^[0-9a-fA-F]{32}$')


def get_aes_backend_name() -> str:
    try:
        from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
        _ = (Cipher, algorithms, modes)
        return 'cryptography'
    except ImportError:
        pass

    try:
        from Crypto.Cipher import AES
        _ = AES
        return 'pycryptodome'
    except ImportError:
        return 'missing'


def _pkcs7_pad(data: bytes, block_size: int = 16) -> bytes:
    pad_len = block_size - (len(data) % block_size)
    return data + bytes([pad_len]) * pad_len


def encrypt_auth_credentials(username: str, password: str, sid_hex: str, key_hex: str) -> str:
    if not HEX_32_RE.fullmatch(sid_hex or ''):
        raise ValueError(f"Invalid SID for IV: expected 32 hex chars, got '{sid_hex}'")

    if not HEX_32_RE.fullmatch(key_hex or ''):
        raise ValueError('Invalid AES-128 key: expected 32 hex chars')

    iv = bytes.fromhex(sid_hex)
    key = bytes.fromhex(key_hex)
    plaintext = _pkcs7_pad(f"{username};{password}".encode('utf-8'))

    try:
        from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
        encryptor = Cipher(algorithms.AES(key), modes.CBC(iv)).encryptor()
        ciphertext = encryptor.update(plaintext) + encryptor.finalize()
        return ciphertext.hex()
    except ImportError:
        try:
            from Crypto.Cipher import AES
            cipher = AES.new(key, AES.MODE_CBC, iv)
            return cipher.encrypt(plaintext).hex()
        except ImportError as ex:
            raise RuntimeError('AES dependency missing. Install `cryptography` or `pycryptodome`.') from ex


def find_sid_value(value):
    if isinstance(value, str) and HEX_32_RE.fullmatch(value):
        return value

    if isinstance(value, list):
        for item in value:
            sid = find_sid_value(item)
            if sid:
                return sid

    if isinstance(value, dict):
        for item in value.values():
            sid = find_sid_value(item)
            if sid:
                return sid

    return None

def get_cli_config():
    args = sys.argv[1:]

    if '--help' in args or '-h' in args:
        print('Usage: python test_ws2.py [url] [--url <url>] [--user <username>] [--password <password>] [--key <aes128-hex>]')
        print('Example: python test_ws2.py --url ws://192.168.1.50:8884/ws2 --user admin --password secret --key A4A86F7D7E119BA3F0CD06881E371B98')
        print('Env: WS2_URL, WS2_USERNAME, WS2_PASSWORD, WS2_KEY (fallback: VSCP_USERNAME, VSCP_PASSWORD, VSCP_KEY16)')
        exit(0)

    url = None
    username = None
    password = None
    key_hex = None
    positionals = []

    index = 0
    while index < len(args):
        arg = args[index]

        if arg == '--url' and index + 1 < len(args):
            url = args[index + 1]
            index += 2
            continue

        if arg.startswith('--url='):
            url = arg[len('--url='):]
            index += 1
            continue

        if arg == '--user' and index + 1 < len(args):
            username = args[index + 1]
            index += 2
            continue

        if arg.startswith('--user='):
            username = arg[len('--user='):]
            index += 1
            continue

        if arg == '--password' and index + 1 < len(args):
            password = args[index + 1]
            index += 2
            continue

        if arg.startswith('--password='):
            password = arg[len('--password='):]
            index += 1
            continue

        if arg == '--key' and index + 1 < len(args):
            key_hex = args[index + 1]
            index += 2
            continue

        if arg.startswith('--key='):
            key_hex = arg[len('--key='):]
            index += 1
            continue

        if not arg.startswith('-'):
            positionals.append(arg)

        index += 1

    if url is None and positionals:
        url = positionals[0]

    if not url:
        url = os.environ.get('WS2_URL', DEFAULT_URL)

    if not username:
        username = os.environ.get('WS2_USERNAME') or os.environ.get('VSCP_USERNAME') or DEFAULT_USERNAME

    if not password:
        password = os.environ.get('WS2_PASSWORD') or os.environ.get('VSCP_PASSWORD') or DEFAULT_PASSWORD

    if not key_hex:
        key_hex = os.environ.get('WS2_KEY') or os.environ.get('VSCP_KEY16') or DEFAULT_KEY_HEX

    return {
        'url': url,
        'username': username,
        'password': password,
        'key_hex': key_hex.strip(),
    }

def handler(signal_received, frame):
    print('SIGINT or CTRL-C detected. Exiting')
    exit(0)

async def connect(config):
    async with websockets.connect(
                 config['url'], ping_interval=20, ping_timeout=20, close_timeout=100) as websocket:

        # Get initial server response
        response = await websocket.recv()
        wsrply = json.loads(response)
        print(f"Initial response from server: {json.dumps(wsrply, indent=2)}")

        sid = find_sid_value(wsrply)
        if not sid:
            print('No SID in initial response, requesting challenge...')
            challenge = {
                "type": "cmd",
                "command": "challenge",
                "args": None
            }
            await websocket.send(json.dumps(challenge))
            response = await websocket.recv()
            wsrply = json.loads(response)
            print(f"Challenge response from server: {json.dumps(wsrply, indent=2)}")
            sid = find_sid_value(wsrply)

        if not sid:
            raise RuntimeError('No SID found in server response')

        auth_crypto = encrypt_auth_credentials(config['username'], config['password'], sid, config['key_hex'])

        # Log in as admin user

        cmdauth = {
            "type": "cmd",
            "command": "auth",
            "args": {
               "iv": sid,
               "crypto": auth_crypto
            }
        }

        print(f"\nLogging in as {config['username']}")
        print(f"AUTH IV   : {sid}")
        print(f"AUTH CRYPT: {auth_crypto}")
        await websocket.send(json.dumps(cmdauth))
        response = await websocket.recv()
        wsrply = json.loads(response)
        print(f"Response from server: {json.dumps(wsrply, indent=2)}")

        # NOOP

        cmdnoop = {
            "type": "cmd",
            "command": "noop",
            "args": None
        }

        print("\nNOOP command")
        await websocket.send(json.dumps(cmdnoop))
        response = await websocket.recv()
        wsrply = json.loads(response)
        print(f"Response from server: {json.dumps(wsrply, indent=2)}")

        # VERSION

        cmdver = {
            "type": "cmd",
            "command": "version",
            "args": None
        }

        print("\nVERSION command")
        await websocket.send(json.dumps(cmdver))
        response = await websocket.recv()
        wsrply = json.loads(response)
        print(f"Response from server: {json.dumps(wsrply, indent=2)}")

        # COPYRIGHT

        cmdcopy = {
            "type": "cmd",
            "command": "copyright",
            "args": None
        }

        print("\nCOPYRIGHT command")
        await websocket.send(json.dumps(cmdcopy))
        response = await websocket.recv()
        wsrply = json.loads(response)
        print(f"Response from server: {json.dumps(wsrply, indent=2)}")

        # Open Channel

        cmdopen = {
            "type": "cmd",
            "command": "open",
            "args": None
        }

        print(f"\nOpen channel  {json.dumps(cmdopen)}")
        await websocket.send(json.dumps(cmdopen))
        response = await websocket.recv()
        wsrply = json.loads(response)
        print(f"Response from server: {json.dumps(wsrply, indent=2)}")



        # Send event  - CLASS1.CONTROL, TurnOn
        sendEvent = {
            "type": "event",
            "event" : {
                "head" : 0,
                "obid": 0,
                "datetime": "2020-01-29T23:05:59Z",
                "timestamp": 0,
                "class": 30,
                "type": 5,
                "guid": "FF:FF:FF:FF:FF:FF:FF:F5:00:00:00:00:00:02:00:00",
                "data": [1,2,3,4,5,6]
            }
        }

        print(f"\nSend event  {json.dumps(sendEvent)}")
        await websocket.send(json.dumps(sendEvent))
        response = await websocket.recv()
        wsrply = json.loads(response)
        print(f"Response from server: {json.dumps(wsrply, indent=2)}")

        # Set filter
        # Receive only CLASS1.CONTROL, TurnOn
        cmdFilter = {
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
        }

        # print(f"\nSet filter  {json.dumps(cmdFilter)}")
        # await websocket.send(json.dumps(cmdFilter))
        # response = await websocket.recv()
        # wsrply = json.loads(response)
        # print(f"Response from server: {json.dumps(wsrply, indent=2)}")

        print("Waiting for class=30, Type=5 as events (Active FILTER) (Abort with ctrl+c)")
        while True:
            response = await websocket.recv()
            wsrply = json.loads(response)
            print(f"Response from server: {json.dumps(wsrply, indent=2)}")
            e = wsrply["event"] # Get event
            # get without extracted event
            print(f"VSCP Class={wsrply['event']['vscpClass']}")
            # use extracted event 
            print(f"VSCP Type={e['vscpType']}")
            # Event can be edited and changed
            e['vscpType'] = 99
            print(f"Modified event: {json.dumps(e, indent=2)}")

if __name__ == '__main__':
    # Tell Python to run the handler() function when SIGINT is recieved
    signal(SIGINT, handler)
    cli_config = get_cli_config()
    backend = get_aes_backend_name()
    if backend == 'missing':
        print('AES backend : missing (install cryptography or pycryptodome)')
    else:
        print(f'AES backend : {backend}')
    print(f"Connecting to {cli_config['url']}")
    asyncio.get_event_loop().run_until_complete(connect(cli_config))
