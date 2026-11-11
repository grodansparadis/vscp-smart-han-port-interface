#!/usr/bin/env python3

"""
VSCP binary protocol smoke test over WebSocket ws1 endpoint.
1. Connects in ws1 (text) mode
2. Authenticates (USER/PASS/AUTH)
3. Opens the channel (OPEN)
4. Sends binary NOOP frame and validates binary reply
"""

import asyncio
import os
import sys
import time
from typing import Optional, Tuple

import websockets
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives import padding
from cryptography.hazmat.backends import default_backend

DEFAULT_URL = "ws://192.168.1.104:8884/ws1"
DEFAULT_USERNAME = "vscp"
DEFAULT_PASSWORD = "secret"
DEFAULT_KEY_HEX = "A4A86F7D7E119BA3F0CD06881E371B98"
TIMEOUT_S = float(os.environ.get("WS_TIMEOUT_S", "5"))
ASYNC_EVENTS_TO_WAIT = int(os.environ.get("WS_ASYNC_EVENTS", "3"))

CMD_NOOP = 0x0000
CMD_QUIT = 0x0001
CMD_USER = 0x0002
CMD_PASS = 0x0003
CMD_CHALLENGE = 0x0004
CMD_SEND = 0x0005
CMD_RETR = 0x0006
CMD_OPEN = 0x0007
CMD_CLOSE = 0x0008
CMD_CHKDATA = 0x0009
CMD_CLEAR = 0x000A
CMD_STAT = 0x000B
CMD_INFO = 0x000C
CMD_GETCHID = 0x000D
CMD_SETGUID = 0x000E
CMD_GETGUID = 0x000F
CMD_VERSION = 0x0010
CMD_SETFILTER = 0x0011
CMD_SETMASK = 0x0012
CMD_INTERFACE = 0x0013
CMD_TEST = 0x001E
CMD_WCYD = 0x001F
CMD_SHUTDOWN = 0x0020
CMD_RESTART = 0x0021
CMD_TEXT = 0x0022
VSCP_ENCRYPTION_AES128 = 0x01
VSCP_BINARY_EVENT_HEADER_LENGTH = 35
VSCP_HEADER16_FRAME_VERSION_UNIX_NS = 0x0100
SAMPLE_EVENT_CLASS = 20
SAMPLE_EVENT_TYPE = 3
SAMPLE_EVENT_DATA = b"\x01\x02\x03"


def calculate_crc_ccitt(data: bytes) -> int:
  """
  Calculate CRC-CCITT for VSCP binary protocol (polynomial 0x1021, initial 0xFFFF).
  Compatible with firmware CRC validation.
  
  Args:
    data: Bytes to calculate CRC over
  Returns:
    16-bit CRC value
  """
  crc = 0xFFFF  # Initial remainder
  for byte in data:
    crc ^= (byte << 8)
    for _ in range(8):
      if crc & 0x8000:
        crc = ((crc << 1) ^ 0x1021) & 0xFFFF
      else:
        crc = (crc << 1) & 0xFFFF
  return crc  # No final XOR for VSCP


def parse_ws1_message(message: str) -> dict:
  """Parse a ws1 text protocol message."""
  if not message:
    return {"kind": "unknown", "raw": message}
  
  kind = message[0]
  fields = message.split(';')
  
  if kind == '+':
    return {
      "kind": "positive",
      "command": fields[1] if len(fields) > 1 else "",
      "details": fields[2:] if len(fields) > 2 else [],
      "raw": message
    }
  
  if kind == '-':
    return {
      "kind": "negative",
      "command": fields[1] if len(fields) > 1 else "",
      "error_code": fields[2] if len(fields) > 2 else "",
      "error_text": ";".join(fields[3:]) if len(fields) > 3 else "",
      "raw": message
    }
  
  return {"kind": "unknown", "raw": message}


def encrypt_auth_credentials(username: str, password: str, sid_hex: str, key_hex: str) -> str:
  """Encrypt authentication credentials using AES-128-CBC."""
  if not all(c in '0123456789abcdefABCDEF' for c in sid_hex) or len(sid_hex) != 32:
    raise ValueError(f"Invalid SID for IV: expected 32 hex chars, got '{sid_hex}'")
  if not all(c in '0123456789abcdefABCDEF' for c in key_hex) or len(key_hex) != 32:
    raise ValueError("Invalid AES-128 key: expected 32 hex chars")
  
  iv = bytes.fromhex(sid_hex)
  key = bytes.fromhex(key_hex)
  plaintext = f"{username}:{password}".encode('utf-8')

  # AES-CBC requires input length to be a multiple of 16 bytes.
  padder = padding.PKCS7(128).padder()
  padded = padder.update(plaintext) + padder.finalize()
  
  cipher = Cipher(algorithms.AES(key), modes.CBC(iv), backend=default_backend())
  encryptor = cipher.encryptor()
  ciphertext = encryptor.update(padded) + encryptor.finalize()
  
  return ciphertext.hex()


async def authenticate_and_switch_to_binary(url: str, username: str, password: str, key_hex: str) -> bytes:
  """
  Authenticate on ws1 and switch to binary protocol.
  Returns the binary reply from NOOP command.
  """
  async with websockets.connect(url, ping_interval=None, ping_timeout=1, close_timeout=5) as ws:
    print("  Connected, waiting for CHALLENGE from server...")
    
    sid = None
    authenticated = False
    channel_open = False
    binary_reply = None
    
    while True:
      # Receive message
      raw = await asyncio.wait_for(ws.recv(), timeout=TIMEOUT_S)
      
      if isinstance(raw, bytes):
        # Binary frame - must be reply to NOOP
        if channel_open:
          binary_reply = raw
          break
        else:
          raise RuntimeError("Received binary data before channel was opened")
      
      # Text message
      msg_str = raw
      parsed = parse_ws1_message(msg_str)
      print(f"  <- {msg_str}")
      
      # Look for SID in CHALLENGE response
      if not sid and parsed["kind"] == "positive":
        for detail in parsed.get("details", []):
          if len(detail) == 32 and all(c in '0123456789abcdefABCDEF' for c in detail):
            sid = detail
            print(f"  SID received: {sid}")
            
            # Send AUTH
            auth_crypto = encrypt_auth_credentials(username, password, sid, key_hex)
            auth_cmd = f"C;AUTH;{sid};{auth_crypto}"
            print(f"  -> {auth_cmd}")
            await ws.send(auth_cmd)
            break
      
      # Check for AUTH success
      elif not authenticated and parsed["kind"] == "positive" and (parsed.get("command") or "").upper() == "AUTH":
        authenticated = True
        print("  Authenticated successfully")
        
        # Send OPEN
        open_cmd = "C;OPEN"
        print(f"  -> {open_cmd}")
        await ws.send(open_cmd)
      
      # Check for OPEN success
      elif not channel_open and parsed["kind"] == "positive" and (parsed.get("command") or "").upper() == "OPEN":
        channel_open = True
        print("  Channel opened successfully")
        print("  Switching to binary protocol...")
        
        # Build a type-14 command frame: [type][cmd msb][cmd lsb][crc msb][crc lsb].
        noop = bytearray(5)
        noop[0] = 0xE0  # Frame type 14 (command), no encryption
        noop[1] = 0x00  # NOOP command MSB
        noop[2] = 0x00  # NOOP command LSB

        # CRC for command/reply is over command + argument (skip type byte).
        crc = calculate_crc_ccitt(bytes(noop[1:-2]))
        noop[-2] = (crc >> 8) & 0xFF
        noop[-1] = crc & 0xFF
        await ws.send(noop)
        print(f"  -> [binary NOOP frame, {len(noop)} bytes]")
      
      # Error response
      elif parsed["kind"] == "negative":
        raise RuntimeError(f"Command failed: {msg_str}")
    
    return binary_reply


def handle_binary_reply(buf: bytes) -> None:
  """Validate binary protocol reply (Frame format 15)."""
  if len(buf) < 7:
    raise RuntimeError(f"Reply too short ({len(buf)} bytes), expected >= 7")

  # Frame format=15 (Protocol Reply):
  # Byte 0: Frame type & encryption settings (should be 0xF0 = reply, no encryption)
  # Byte 1-2: Command code (echo of command)
  # Byte 3-4: Error code (0x0000 = success)
  # Byte N-2..N-1: CRC-CCITT over command + error + args (excluding byte 0)
  frame_type = buf[0] & 0xF0
  command = (buf[1] << 8) | buf[2]
  error = (buf[3] << 8) | buf[4]
  received_crc = (buf[-2] << 8) | buf[-1]

  # Validate CRC over command + error + args (skip type byte and trailing CRC bytes)
  frame_data_for_crc = buf[1:-2]
  expected_crc = calculate_crc_ccitt(frame_data_for_crc)
  
  print(f"  <- [binary reply, {len(buf)} bytes]")
  print(f"     Type: 0x{buf[0]:02X}, Command: 0x{command:04X}, Error: 0x{error:04X}")
  
  if frame_type != 0xF0:
    raise RuntimeError(f"Unexpected reply frame type: 0x{buf[0]:02X}")
  
  if command != 0x0000:
    raise RuntimeError(f"Unexpected command in reply: 0x{command:04X}")
  
  if error != 0x0000:
    raise RuntimeError(f"NOOP failed, error=0x{error:04X}")
  
  if received_crc != expected_crc:
    raise RuntimeError(f"CRC mismatch: received 0x{received_crc:04X}, expected 0x{expected_crc:04X}")


def build_binary_command_frame(command: int, arg: bytes = b"") -> bytes:
  """Build frame type 14 command frame: [type][cmd][arg...][crc]."""
  frame = bytearray(1 + 2 + len(arg) + 2)
  frame[0] = 0xE0
  frame[1] = (command >> 8) & 0xFF
  frame[2] = command & 0xFF
  if arg:
    frame[3:3 + len(arg)] = arg

  crc = calculate_crc_ccitt(bytes(frame[1:-2]))
  frame[-2] = (crc >> 8) & 0xFF
  frame[-1] = crc & 0xFF
  return bytes(frame)


def encrypt_binary_frame(frame: bytes, key_hex: str, encryption: int = VSCP_ENCRYPTION_AES128) -> bytes:
  """Encrypt a VSCP binary frame using the same framing as vscp_fwhlp_encryptFrame."""
  if len(frame) < 1:
    raise ValueError("Cannot encrypt empty frame")
  if encryption != VSCP_ENCRYPTION_AES128:
    raise ValueError(f"Unsupported binary frame encryption level: {encryption}")
  if not all(c in '0123456789abcdefABCDEF' for c in key_hex) or len(key_hex) != 32:
    raise ValueError("Invalid AES-128 key: expected 32 hex chars")

  key = bytes.fromhex(key_hex)
  iv = os.urandom(16)
  payload = frame[1:]
  pad_len = 16 - (len(payload) % 16)
  padded_payload = payload + (b"\x00" * pad_len)

  cipher = Cipher(algorithms.AES(key), modes.CBC(iv), backend=default_backend())
  encryptor = cipher.encryptor()
  ciphertext = encryptor.update(padded_payload) + encryptor.finalize()

  return bytes([(frame[0] & 0xF0) | (encryption & 0x0F)]) + ciphertext + iv


def build_sample_event_frame() -> bytes:
  """Build a small VSCP binary event frame for SEND command tests."""
  frame = bytearray(1 + VSCP_BINARY_EVENT_HEADER_LENGTH + len(SAMPLE_EVENT_DATA) + 2)
  timestamp_ns = time.time_ns()

  frame[0] = 0x00
  frame[1] = (VSCP_HEADER16_FRAME_VERSION_UNIX_NS >> 8) & 0xFF
  frame[2] = VSCP_HEADER16_FRAME_VERSION_UNIX_NS & 0xFF

  for idx in range(8):
    shift = 56 - (idx * 8)
    frame[3 + idx] = (timestamp_ns >> shift) & 0xFF

  frame[11] = 0
  frame[12] = 0
  frame[13] = 0
  frame[14] = (SAMPLE_EVENT_CLASS >> 8) & 0xFF
  frame[15] = SAMPLE_EVENT_CLASS & 0xFF
  frame[16] = (SAMPLE_EVENT_TYPE >> 8) & 0xFF
  frame[17] = SAMPLE_EVENT_TYPE & 0xFF
  frame[33] = 0x01
  frame[34] = (len(SAMPLE_EVENT_DATA) >> 8) & 0xFF
  frame[35] = len(SAMPLE_EVENT_DATA) & 0xFF
  frame[36:36 + len(SAMPLE_EVENT_DATA)] = SAMPLE_EVENT_DATA

  crc = calculate_crc_ccitt(bytes(frame[1:-2]))
  frame[-2] = (crc >> 8) & 0xFF
  frame[-1] = crc & 0xFF
  return bytes(frame)


def parse_binary_reply(buf: bytes) -> Tuple[int, int]:
  """Parse and validate frame type 15 reply and return (command, error)."""
  if len(buf) < 7:
    raise RuntimeError(f"Reply too short ({len(buf)} bytes), expected >= 7")

  frame_type = buf[0] & 0xF0
  command = (buf[1] << 8) | buf[2]
  error = (buf[3] << 8) | buf[4]
  received_crc = (buf[-2] << 8) | buf[-1]
  expected_crc = calculate_crc_ccitt(buf[1:-2])

  if frame_type != 0xF0:
    raise RuntimeError(f"Unexpected reply frame type: 0x{buf[0]:02X}")
  if received_crc != expected_crc:
    raise RuntimeError(f"CRC mismatch: received 0x{received_crc:04X}, expected 0x{expected_crc:04X}")

  return command, error


async def wait_for_binary_reply(ws, expected_command: int, step_name: str) -> None:
  """Wait for expected binary reply. Ignore text and async event frames."""
  while True:
    raw = await asyncio.wait_for(ws.recv(), timeout=TIMEOUT_S)

    if isinstance(raw, str):
      print(f"  <- {raw}")
      continue

    frame_type = raw[0] & 0xF0
    if frame_type == 0x00:
      print(f"  <- [async event while waiting for {step_name}, {len(raw)} bytes]")
      continue

    command, error = parse_binary_reply(raw)
    print(f"  <- [binary reply, {len(raw)} bytes] cmd=0x{command:04X} err=0x{error:04X}")
    if command != expected_command:
      continue
    if error != 0x0000:
      raise RuntimeError(f"{step_name} failed, error=0x{error:04X}")
    return


async def wait_for_async_events(ws, count: int, label: str) -> None:
  """Wait for count asynchronous event frames before continuing."""
  print(f"  Waiting for {count} asynchronous event(s) in {label}...")
  received = 0

  while received < count:
    raw = await asyncio.wait_for(ws.recv(), timeout=TIMEOUT_S * 4)
    if isinstance(raw, str):
      msg = raw.strip()
      print(f"  <- {msg}")
      # WS1 may still deliver async events in text CSV form while binary commands are used.
      if msg and not msg.startswith('+') and not msg.startswith('-'):
        received += 1
        print(f"  <- [async event {received}/{count}, text]")
      continue

    frame_type = raw[0] & 0xF0
    if frame_type == 0x00:
      received += 1
      print(f"  <- [async event {received}/{count}, {len(raw)} bytes]")
    else:
      command, error = parse_binary_reply(raw)
      print(f"  <- [binary reply while waiting events] cmd=0x{command:04X} err=0x{error:04X}")


async def run_scenario_text_then_binary(url: str, username: str, password: str, key_hex: str) -> None:
  """Scenario A: text AUTH/OPEN then binary NOOP/QUIT with async wait."""
  print("\nScenario A: text AUTH/OPEN then binary NOOP")
  async with websockets.connect(url, ping_interval=None, ping_timeout=1, close_timeout=5) as ws:
    print("  Connected, waiting for CHALLENGE from server...")

    # Some servers do not emit initial AUTH0 automatically.
    print("  -> C;CHALLENGE")
    await ws.send("C;CHALLENGE")

    sid = None
    authenticated = False
    channel_open = False

    while not channel_open:
      raw = await asyncio.wait_for(ws.recv(), timeout=TIMEOUT_S)
      if isinstance(raw, bytes):
        continue

      parsed = parse_ws1_message(raw)
      print(f"  <- {raw}")

      if not sid and parsed["kind"] == "positive":
        for detail in parsed.get("details", []):
          if len(detail) == 32 and all(c in '0123456789abcdefABCDEF' for c in detail):
            sid = detail
            auth_crypto = encrypt_auth_credentials(username, password, sid, key_hex)
            auth_cmd = f"C;AUTH;{sid};{auth_crypto}"
            print(f"  -> {auth_cmd}")
            await ws.send(auth_cmd)
            break
      elif not authenticated and parsed["kind"] == "positive" and (parsed.get("command") or "").upper() == "AUTH":
        authenticated = True
        print("  -> C;OPEN")
        await ws.send("C;OPEN")
      elif parsed["kind"] == "positive" and (parsed.get("command") or "").upper() == "OPEN":
        channel_open = True
      elif parsed["kind"] == "negative":
        raise RuntimeError(f"Command failed: {raw}")

    await ws.send(build_binary_command_frame(CMD_NOOP))
    print("  -> [binary NOOP frame, 5 bytes]")
    await wait_for_binary_reply(ws, CMD_NOOP, "NOOP")
    await wait_for_async_events(ws, ASYNC_EVENTS_TO_WAIT, "scenario A")

    await ws.send(build_binary_command_frame(CMD_QUIT))
    print("  -> [binary QUIT frame, 5 bytes]")
    await wait_for_binary_reply(ws, CMD_QUIT, "QUIT")


async def run_scenario_binary_only(url: str, username: str, password: str) -> None:
  """Scenario B: binary USER/PASS/OPEN/NOOP/QUIT with async wait."""
  print("\nScenario B: binary USER/PASS/OPEN/NOOP/QUIT")
  async with websockets.connect(url, ping_interval=None, ping_timeout=1, close_timeout=5) as ws:
    print("  Connected, waiting for initial text greeting/challenge...")

    first = await asyncio.wait_for(ws.recv(), timeout=TIMEOUT_S)
    if isinstance(first, str):
      print(f"  <- {first}")

    await ws.send(build_binary_command_frame(CMD_USER, username.encode("utf-8") + b"\x00"))
    print("  -> [binary USER]")
    await wait_for_binary_reply(ws, CMD_USER, "USER")

    await ws.send(build_binary_command_frame(CMD_PASS, password.encode("utf-8") + b"\x00"))
    print("  -> [binary PASS]")
    await wait_for_binary_reply(ws, CMD_PASS, "PASS")

    await ws.send(build_binary_command_frame(CMD_OPEN))
    print("  -> [binary OPEN]")
    await wait_for_binary_reply(ws, CMD_OPEN, "OPEN")

    await ws.send(build_binary_command_frame(CMD_SEND, build_sample_event_frame()))
    print("  -> [binary SEND event]")
    await wait_for_binary_reply(ws, CMD_SEND, "SEND")

    await ws.send(build_binary_command_frame(CMD_NOOP))
    print("  -> [binary NOOP frame, 5 bytes]")
    await wait_for_binary_reply(ws, CMD_NOOP, "NOOP")
    await wait_for_async_events(ws, ASYNC_EVENTS_TO_WAIT, "scenario B")

    await ws.send(build_binary_command_frame(CMD_QUIT))
    print("  -> [binary QUIT frame, 5 bytes]")
    await wait_for_binary_reply(ws, CMD_QUIT, "QUIT")


async def run_scenario_binary_only_encrypted(url: str, username: str, password: str, key_hex: str) -> None:
  """Scenario C: encrypted binary command sweep with async wait."""
  print("\nScenario C: encrypted binary USER/PASS/OPEN + command sweep")
  async with websockets.connect(url, ping_interval=None, ping_timeout=1, close_timeout=5) as ws:
    print("  Connected, waiting for initial text greeting/challenge...")

    first = await asyncio.wait_for(ws.recv(), timeout=TIMEOUT_S)
    if isinstance(first, str):
      print(f"  <- {first}")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_USER, username.encode("utf-8") + b"\x00"), key_hex))
    print("  -> [encrypted binary USER]")
    await wait_for_binary_reply(ws, CMD_USER, "encrypted USER")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_PASS, password.encode("utf-8") + b"\x00"), key_hex))
    print("  -> [encrypted binary PASS]")
    await wait_for_binary_reply(ws, CMD_PASS, "encrypted PASS")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_OPEN), key_hex))
    print("  -> [encrypted binary OPEN]")
    await wait_for_binary_reply(ws, CMD_OPEN, "encrypted OPEN")

    zero_guid = bytes(16)
    zero_filter = bytes(21)
    iface_count = b"\x00"

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_GETCHID), key_hex))
    print("  -> [encrypted binary GETCHID]")
    await wait_for_binary_reply(ws, CMD_GETCHID, "encrypted GETCHID")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_GETGUID), key_hex))
    print("  -> [encrypted binary GETGUID]")
    await wait_for_binary_reply(ws, CMD_GETGUID, "encrypted GETGUID")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_SETGUID, zero_guid), key_hex))
    print("  -> [encrypted binary SETGUID]")
    await wait_for_binary_reply(ws, CMD_SETGUID, "encrypted SETGUID")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_VERSION), key_hex))
    print("  -> [encrypted binary VERSION]")
    await wait_for_binary_reply(ws, CMD_VERSION, "encrypted VERSION")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_STAT), key_hex))
    print("  -> [encrypted binary STAT]")
    await wait_for_binary_reply(ws, CMD_STAT, "encrypted STAT")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_INFO), key_hex))
    print("  -> [encrypted binary INFO]")
    await wait_for_binary_reply(ws, CMD_INFO, "encrypted INFO")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_WCYD), key_hex))
    print("  -> [encrypted binary WCYD]")
    await wait_for_binary_reply(ws, CMD_WCYD, "encrypted WCYD")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_CHKDATA), key_hex))
    print("  -> [encrypted binary CHKDATA]")
    await wait_for_binary_reply(ws, CMD_CHKDATA, "encrypted CHKDATA")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_CLEAR), key_hex))
    print("  -> [encrypted binary CLEAR]")
    await wait_for_binary_reply(ws, CMD_CLEAR, "encrypted CLEAR")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_SETFILTER, zero_filter), key_hex))
    print("  -> [encrypted binary SETFILTER]")
    await wait_for_binary_reply(ws, CMD_SETFILTER, "encrypted SETFILTER")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_SETMASK, zero_filter), key_hex))
    print("  -> [encrypted binary SETMASK]")
    await wait_for_binary_reply(ws, CMD_SETMASK, "encrypted SETMASK")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_INTERFACE, iface_count), key_hex))
    print("  -> [encrypted binary INTERFACE]")
    await wait_for_binary_reply(ws, CMD_INTERFACE, "encrypted INTERFACE")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_TEST), key_hex))
    print("  -> [encrypted binary TEST]")
    await wait_for_binary_reply(ws, CMD_TEST, "encrypted TEST")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_CHALLENGE), key_hex))
    print("  -> [encrypted binary CHALLENGE]")
    await wait_for_binary_reply(ws, CMD_CHALLENGE, "encrypted CHALLENGE")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_SEND, build_sample_event_frame()), key_hex))
    print("  -> [encrypted binary SEND event]")
    await wait_for_binary_reply(ws, CMD_SEND, "encrypted SEND")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_NOOP), key_hex))
    print("  -> [encrypted binary NOOP frame]")
    await wait_for_binary_reply(ws, CMD_NOOP, "encrypted NOOP")
    await wait_for_async_events(ws, ASYNC_EVENTS_TO_WAIT, "scenario C")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_CLOSE), key_hex))
    print("  -> [encrypted binary CLOSE]")
    await wait_for_binary_reply(ws, CMD_CLOSE, "encrypted CLOSE")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_RETR), key_hex))
    print("  -> [encrypted binary RETR]")
    await wait_for_binary_reply(ws, CMD_RETR, "encrypted RETR")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_OPEN), key_hex))
    print("  -> [encrypted binary OPEN(reopen)]")
    await wait_for_binary_reply(ws, CMD_OPEN, "encrypted OPEN(reopen)")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_SHUTDOWN), key_hex))
    print("  -> [encrypted binary SHUTDOWN]")
    await wait_for_binary_reply(ws, CMD_SHUTDOWN, "encrypted SHUTDOWN")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_TEXT), key_hex))
    print("  -> [encrypted binary TEXT]")
    await wait_for_binary_reply(ws, CMD_TEXT, "encrypted TEXT")

    await ws.send(encrypt_binary_frame(build_binary_command_frame(CMD_RESTART), key_hex))
    print("  -> [encrypted binary RESTART]")
    await wait_for_binary_reply(ws, CMD_RESTART, "encrypted RESTART")


def get_url() -> str:
  """Parse command line arguments for URL."""
  args = sys.argv[1:]

  if "--help" in args or "-h" in args:
    print("Usage: python test_vscp_binary.py [url] [--url <url>]")
    print("Env: WS_BINARY_URL, WS1_URL, WS_TIMEOUT_S")
    print("Auth env: VSCP_USERNAME, VSCP_PASSWORD, VSCP_KEY16")
    sys.exit(0)

  url = None
  idx = 0
  while idx < len(args):
    arg = args[idx]

    if arg == "--url" and idx + 1 < len(args):
      url = args[idx + 1]
      idx += 2
      continue

    if arg.startswith("--url="):
      url = arg[len("--url="):]
      idx += 1
      continue

    if not arg.startswith("-") and url is None:
      url = arg
      idx += 1
      continue

    idx += 1

  return url or os.environ.get("WS_BINARY_URL") or os.environ.get("WS1_URL") or DEFAULT_URL


async def main():
  """Main test function."""
  target_url = get_url()
  username = os.environ.get("VSCP_USERNAME") or DEFAULT_USERNAME
  password = os.environ.get("VSCP_PASSWORD") or DEFAULT_PASSWORD
  key_hex = (os.environ.get("VSCP_KEY16") or DEFAULT_KEY_HEX).strip()
  
  print(f"Connecting to {target_url}")
  await run_scenario_text_then_binary(target_url, username, password, key_hex)
  await run_scenario_binary_only(target_url, username, password)
  await run_scenario_binary_only_encrypted(target_url, username, password, key_hex)
  print("\nPASS: Binary NOOP test passed in all scenarios (text-assisted, binary-only, encrypted-binary)")


if __name__ == "__main__":
  try:
    asyncio.run(main())
  except Exception as err:
    print(f"\nFAIL: {err}", file=sys.stderr)
    sys.exit(1)

