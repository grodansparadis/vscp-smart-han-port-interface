#!/usr/bin/env python3

"""Extract and decode VSCP BLE GUID+IPv4 payloads from btmon/bluetoothctl logs.

This script scans input text for hex byte sequences and tries to decode payloads
with these layouts:
- 20 bytes: GUID(16) + IPv4(4)
- 22 bytes: CompanyID(2) + GUID(16) + IPv4(4)
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys
from typing import Iterable

HEX_PAIR_RE = re.compile(r"\b[0-9a-fA-F]{2}\b")
BYTE_LINE_RE = re.compile(r"^\s*(?:Data:)?\s*([0-9a-fA-F]{2}(?:\s+[0-9a-fA-F]{2})+)\s*$")
COMPANY_RE = re.compile(r"manufacturer\s+data\s*\(0x([0-9a-fA-F]{4})\)", re.IGNORECASE)
COMPANY_LINE_RE = re.compile(r"\bcompany:.*\((\d+)\)", re.IGNORECASE)
DATA_BRACKET_RE = re.compile(r"\bData\[(\d+)\]:\s*([0-9a-fA-F]+)", re.IGNORECASE)
ADDRESS_RE = re.compile(r"\bAddress:\s*([0-9A-F]{2}(?::[0-9A-F]{2}){5})\b", re.IGNORECASE)


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

    raise ValueError("unsupported payload length")


def format_guid(guid: bytes) -> str:
    return ":".join(f"{b:02X}" for b in guid)


def format_ip(ip: bytes) -> str:
    return ".".join(str(b) for b in ip)


def finalize_mfg_bytes(raw: bytes, expected_company: int | None) -> bytes | None:
    if len(raw) in (20, 22):
        return raw

    if len(raw) < 20:
        return None

    # If btmon reported the company ID in the heading, prefer matching 22-byte sequence.
    if expected_company is not None and len(raw) >= 22:
        c0 = expected_company & 0xFF
        c1 = (expected_company >> 8) & 0xFF
        for i in range(0, len(raw) - 22 + 1):
            if raw[i] == c0 and raw[i + 1] == c1:
                return raw[i : i + 22]

    # Otherwise keep strict behavior and do not guess with sliding windows.
    return None


def parse_manufacturer_candidates(text: str, mac_filter: str | None = None) -> Iterable[bytes]:
    in_mfg_block = False
    expected_company: int | None = None
    collected: list[int] = []
    current_addr: str | None = None
    block_addr: str | None = None

    def flush_current() -> Iterable[bytes]:
        nonlocal collected, expected_company
        if not collected:
            expected_company = None
            return []

        raw = bytes(collected)
        payload = finalize_mfg_bytes(raw, expected_company)
        collected = []
        expected_company = None
        if payload is None:
            return []
        return [payload]

    for line in text.splitlines():
        lower = line.lower()

        m_addr = ADDRESS_RE.search(line)
        if m_addr:
            current_addr = m_addr.group(1).upper()

        if "manufacturer data" in lower:
            # Start a new block; flush any previous one first.
            for payload in flush_current():
                if (mac_filter is None) or (block_addr == mac_filter):
                    yield payload

            in_mfg_block = True
            block_addr = current_addr
            m = COMPANY_RE.search(line)
            expected_company = int(m.group(1), 16) if m else None

            # Some tools include bytes on the same line.
            mm = BYTE_LINE_RE.match(line)
            if mm:
                collected.extend(int(x, 16) for x in mm.group(1).split())
            continue

        if not in_mfg_block:
            continue

        mm = BYTE_LINE_RE.match(line)
        if mm:
            collected.extend(int(x, 16) for x in mm.group(1).split())
            continue

        # End of manufacturer block.
        for payload in flush_current():
            if (mac_filter is None) or (block_addr == mac_filter):
                yield payload
        in_mfg_block = False
        block_addr = None

    # Flush at EOF.
    for payload in flush_current():
        if (mac_filter is None) or (block_addr == mac_filter):
            yield payload


def parse_company_data_candidates(text: str, mac_filter: str | None = None) -> Iterable[bytes]:
    expected_company: int | None = None
    current_addr: str | None = None

    for line in text.splitlines():
        m_addr = ADDRESS_RE.search(line)
        if m_addr:
            current_addr = m_addr.group(1).upper()

        m_company = COMPANY_LINE_RE.search(line)
        if m_company:
            try:
                expected_company = int(m_company.group(1), 10) & 0xFFFF
            except ValueError:
                expected_company = None
            continue

        m_data = DATA_BRACKET_RE.search(line)
        if not m_data:
            continue

        if (mac_filter is not None) and (current_addr != mac_filter):
            continue

        try:
            data_len = int(m_data.group(1), 10)
        except ValueError:
            continue

        hex_blob = m_data.group(2)
        if len(hex_blob) != data_len * 2:
            continue

        raw = bytes.fromhex(hex_blob)

        # btmon 'Company: ... Data[20]' means company id is separate from blob.
        if (data_len == 20) and (expected_company is not None):
            c0 = expected_company & 0xFF
            c1 = (expected_company >> 8) & 0xFF
            yield bytes([c0, c1]) + raw
            continue

        if data_len in (20, 22):
            yield raw


def unique_payloads(candidates: Iterable[bytes]) -> list[bytes]:
    seen: set[bytes] = set()
    out: list[bytes] = []
    for payload in candidates:
        if payload in seen:
            continue
        try:
            decode_payload(payload)
        except ValueError:
            continue
        seen.add(payload)
        out.append(payload)
    return out


def read_input(args: argparse.Namespace) -> str:
    if args.file:
        return pathlib.Path(args.file).read_text(encoding="utf-8", errors="replace")
    if args.stdin:
        return sys.stdin.read()
    raise ValueError("provide --file or --stdin")


def main() -> int:
    parser = argparse.ArgumentParser(description="Decode GUID+IPv4 BLE payloads from btmon-style logs")
    parser.add_argument("--file", help="path to btmon/bluetoothctl log file")
    parser.add_argument("--stdin", action="store_true", help="read log text from stdin")
    parser.add_argument("--mac", help="optional BLE MAC address filter (AA:BB:CC:DD:EE:FF)")
    args = parser.parse_args()

    if not args.file and not args.stdin:
        parser.error("provide --file <path> or --stdin")

    text = read_input(args)
    mac_filter = args.mac.upper() if args.mac else None

    payloads = unique_payloads(parse_manufacturer_candidates(text, mac_filter))
    if not payloads:
        payloads = unique_payloads(parse_company_data_candidates(text, mac_filter))

    if not payloads:
        print("No decodable 20-byte/22-byte GUID+IPv4 payload found.", file=sys.stderr)
        return 2

    for idx, payload in enumerate(payloads, start=1):
        company, guid, ip = decode_payload(payload)
        print(f"payload[{idx}] len={len(payload)}")
        if company:
            print(f"  company_id=0x{company[0]:02X}{company[1]:02X}")
        else:
            print("  company_id=(not present)")
        print(f"  guid={format_guid(guid)}")
        print(f"  ipv4={format_ip(ip)}")
        print(f"  raw={payload.hex()}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
