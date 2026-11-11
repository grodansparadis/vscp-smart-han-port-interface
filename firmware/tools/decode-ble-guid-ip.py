#!/usr/bin/env python3

"""Decode VSCP BLE manufacturer payload (GUID + IPv4).

Expected payload layouts:
- 20 bytes: GUID(16) + IPv4(4)
- 22 bytes: CompanyID(2) + GUID(16) + IPv4(4)
"""

from __future__ import annotations

import argparse
import re
import sys


def normalize_hex_bytes(raw: str) -> bytes:
    cleaned = re.sub(r"[^0-9a-fA-F]", "", raw)
    if len(cleaned) == 0:
        raise ValueError("no hex data found")
    if len(cleaned) % 2 != 0:
        raise ValueError("odd number of hex digits")
    return bytes.fromhex(cleaned)


def decode_payload(payload: bytes) -> tuple[bytes, bytes, bytes]:
    if len(payload) == 20:
        company = b""
        guid = payload[:16]
        ip = payload[16:20]
        return company, guid, ip

    if len(payload) == 22:
        company = payload[:2]
        guid = payload[2:18]
        ip = payload[18:22]
        return company, guid, ip

    raise ValueError(
        f"unexpected payload length {len(payload)} bytes; expected 20 (GUID+IP) or 22 (CompanyID+GUID+IP)"
    )


def format_guid(guid: bytes) -> str:
    return ":".join(f"{b:02X}" for b in guid)


def format_ip(ip: bytes) -> str:
    return ".".join(str(b) for b in ip)


def main() -> int:
    parser = argparse.ArgumentParser(description="Decode BLE manufacturer payload to GUID and IPv4")
    parser.add_argument(
        "payload",
        nargs="?",
        help="hex payload string (for example: 01 02 ... or 0102...)",
    )
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read payload text from stdin",
    )
    args = parser.parse_args()

    source = ""
    if args.stdin:
        source = sys.stdin.read()
    elif args.payload:
        source = args.payload
    else:
        parser.error("provide payload argument or use --stdin")

    try:
        payload = normalize_hex_bytes(source)
        company, guid, ip = decode_payload(payload)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    print(f"raw_len={len(payload)}")
    if company:
        print(f"company_id=0x{company[0]:02X}{company[1]:02X}")
    else:
        print("company_id=(not present)")
    print(f"guid={format_guid(guid)}")
    print(f"ipv4={format_ip(ip)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
