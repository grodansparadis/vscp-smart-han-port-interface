#!/usr/bin/env python

# VSCP ws1 client example  (Need python3)
# Demonstrates the use of the ws1 websocket interface of the VSCP daemon
# Sample event to send
#    send 0,20,3,,,0,-,15,14,13,12,11,10,9,8,7,6,5,4,3,2,0,0,1,35
# Original sample from: https://websockets.readthedocs.io/en/stable/intro.html
# Copyright 2020-2026 Ake Hedman, the VSCP project - MIT license

from signal import signal, SIGINT
from sys import exit
import sys
import os
import asyncio
import websockets
import re

DEFAULT_URL = 'ws://192.168.1.100:8884/ws1'
DEFAULT_USERNAME = 'vscp'
DEFAULT_PASSWORD = 'secret'
DEFAULT_KEY_HEX = 'A4A86F7D7E119BA3F0CD06881E371B98'

HEX_32_RE = re.compile(r'^[0-9a-fA-F]{32}$')
HEX_32_ANY_RE = re.compile(r'\b([0-9a-fA-F]{32})\b')


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
    plaintext = _pkcs7_pad(f"{username}:{password}".encode('utf-8'))

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


def extract_sid_from_text(text: str):
    if not text:
        return None

    match = HEX_32_ANY_RE.search(text)
    if match:
        return match.group(1)

    return None


def get_cli_config():
    args = sys.argv[1:]

    if '--help' in args or '-h' in args:
        print('Usage: python test_ws1.py [url] [--url <url>] [--user <username>] [--password <password>] [--key <aes128-hex>]')
        print('Example: python test_ws1.py --url ws://192.168.1.50:8884/ws1 --user admin --password secret --key A4A86F7D7E119BA3F0CD06881E371B98')
        print('Env: WS1_URL, WS1_USERNAME, WS1_PASSWORD, WS1_KEY (fallback: VSCP_USERNAME, VSCP_PASSWORD, VSCP_KEY16)')
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
        url = os.environ.get('WS1_URL', DEFAULT_URL)

    if not username:
        username = os.environ.get('WS1_USERNAME') or os.environ.get('VSCP_USERNAME') or DEFAULT_USERNAME

    if not password:
        password = os.environ.get('WS1_PASSWORD') or os.environ.get('VSCP_PASSWORD') or DEFAULT_PASSWORD

    if not key_hex:
        key_hex = os.environ.get('WS1_KEY') or os.environ.get('VSCP_KEY16') or DEFAULT_KEY_HEX

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
                 config['url'], ping_interval=None, ping_timeout=1, close_timeout=100) as websocket:

        await websocket.send("C;CHALLENGE")
        greeting = await websocket.recv()
        print(f"Response from server: {greeting}")

        sid = extract_sid_from_text(greeting)
        if not sid:
            raise RuntimeError('No SID found in challenge response')

        auth_crypto = encrypt_auth_credentials(config['username'], config['password'], sid, config['key_hex'])
        cmdauth = f"C;AUTH;{sid};{auth_crypto}"

        print(f"Logging in as {config['username']}")
        print(f"AUTH IV   : {sid}")
        print(f"AUTH CRYPT: {auth_crypto}")
        await websocket.send(cmdauth)
        greeting = await websocket.recv()
        print(f"Response from server: {greeting}")

        await websocket.send("C;OPEN")
        greeting = await websocket.recv()
        print(f"Response from server: {greeting}")

        print("Waiting for events")
        while True:
            greeting = await websocket.recv()
            print(f"Response from server: {greeting}")


if __name__ == '__main__':
    signal(SIGINT, handler)
    cli_config = get_cli_config()
    backend = get_aes_backend_name()
    if backend == 'missing':
        print('AES backend : missing (install cryptography or pycryptodome)')
    else:
        print(f'AES backend : {backend}')
    print(f"Connecting to {cli_config['url']}")
    asyncio.get_event_loop().run_until_complete(connect(cli_config))
