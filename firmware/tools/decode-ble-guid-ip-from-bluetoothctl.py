#!/usr/bin/env python3

"""Decode VSCP GUID+IPv4 BLE payloads using bluetoothctl output.

Supports two modes:
1) Active scan + info query (no root privileges needed in many setups)
2) Parse existing bluetoothctl text from stdin/file

Expected payload layouts:
- 20 bytes: GUID(16) + IPv4(4)
- 22 bytes: CompanyID(2) + GUID(16) + IPv4(4)
"""

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys
from typing import Iterable

HEX_PAIR_RE = re.compile(r"\b[0-9a-fA-F]{2}\b")
MAC_RE = re.compile(r"\b([0-9A-F]{2}(?::[0-9A-F]{2}){5})\b", re.IGNORECASE)


def decode_payload(payload: bytes) -> tuple[bytes, bytes, bytes]:
    if len(payload) == 20:
        return b"", payload[:16], payload[16:20]
    if len(payload) == 22:
        return payload[:2], payload[2:18], payload[18:22]
    raise ValueError("unsupported payload length")


def format_guid(guid: bytes) -> str:
    return ":".join(f"{b:02X}" for b in guid)


def format_ip(ip: bytes) -> str:
    return ".".join(str(b) for b in ip)


def extract_macs_from_scan(scan_output: str) -> list[str]:
    macs: list[str] = []
    seen: set[str] = set()
    for line in scan_output.splitlines():
        if "Device" not in line:
            continue
        m = MAC_RE.search(line)
        if not m:
            continue
        mac = m.group(1).upper()
        if mac not in seen:
            seen.add(mac)
            macs.append(mac)
    return macs


def run_cmd(cmd: list[str]) -> tuple[int, str, str]:
    proc = subprocess.run(cmd, capture_output=True, text=True)
    return proc.returncode, proc.stdout, proc.stderr


def collect_info_text(macs: list[str]) -> str:
    out_parts: list[str] = []
    for mac in macs:
        rc, out, err = run_cmd(["bluetoothctl", "info", mac])
        if rc == 0 and out.strip():
            out_parts.append(out)
        elif err.strip():
            out_parts.append(err)
    return "\n".join(out_parts)


def parse_manufacturer_blocks(text: str) -> Iterable[bytes]:
    lines = text.splitlines()
    in_mfg_value = False
    for line in lines:
        lower = line.lower()

        if "manufacturerdata value" in lower:
            in_mfg_value = True
            continue

        if "manufacturerdata" in lower and "value" not in lower:
            # Keep scanning, sometimes value appears on the same line.
            pairs = HEX_PAIR_RE.findall(line)
            if len(pairs) >= 20:
                raw = bytes.fromhex("".join(pairs))
                if len(raw) >= 22:
                    yield raw[-22:]
                elif len(raw) >= 20:
                    yield raw[-20:]
            continue

        if in_mfg_value:
            pairs = HEX_PAIR_RE.findall(line)
            if not pairs:
                in_mfg_value = False
                continue
            raw = bytes.fromhex("".join(pairs))
            if len(raw) >= 22:
                yield raw[-22:]
            elif len(raw) >= 20:
                yield raw[-20:]


def unique_decodable(candidates: Iterable[bytes]) -> list[bytes]:
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


def print_results(payloads: list[bytes]) -> int:
    if not payloads:
        print("No decodable 20-byte/22-byte GUID+IPv4 payload found.", file=sys.stderr)
        return 2

    for i, payload in enumerate(payloads, start=1):
        company, guid, ip = decode_payload(payload)
        print(f"payload[{i}] len={len(payload)}")
        if company:
            print(f"  company_id=0x{company[0]:02X}{company[1]:02X}")
        else:
            print("  company_id=(not present)")
        print(f"  guid={format_guid(guid)}")
        print(f"  ipv4={format_ip(ip)}")
        print(f"  raw={payload.hex()}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Decode GUID+IPv4 BLE payloads from bluetoothctl")
    parser.add_argument("--mac", help="specific MAC to query with bluetoothctl info")
    parser.add_argument("--scan-time", type=int, default=12, help="bluetoothctl scan duration in seconds")
    parser.add_argument("--file", help="parse existing bluetoothctl output file")
    parser.add_argument("--stdin", action="store_true", help="parse bluetoothctl output from stdin")
    args = parser.parse_args()

    if args.file or args.stdin:
        if args.file:
            text = pathlib.Path(args.file).read_text(encoding="utf-8", errors="replace")
        else:
            text = sys.stdin.read()

        payloads = unique_decodable(parse_manufacturer_blocks(text))
        return print_results(payloads)

    scan_cmd = ["bluetoothctl", "--timeout", str(args.scan_time), "scan", "on"]
    rc, scan_out, scan_err = run_cmd(scan_cmd)
    if rc != 0 and not scan_out:
        print(scan_err.strip() or "bluetoothctl scan failed", file=sys.stderr)
        return 2

    macs = extract_macs_from_scan(scan_out + "\n" + scan_err)
    if args.mac:
        mac = args.mac.upper()
        if mac not in macs:
            macs.insert(0, mac)

    if not macs:
        print("No BLE devices discovered. Try increasing --scan-time.", file=sys.stderr)
        return 2

    info_text = collect_info_text(macs)
    payloads = unique_decodable(parse_manufacturer_blocks(info_text))

    if payloads:
        return print_results(payloads)

    print("No decodable payload from bluetoothctl info output.", file=sys.stderr)
    print("Tip: try --mac <device-mac> and a longer --scan-time.", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
