# BLE + Wi-Fi Coexistence and GUID/IP Advertising

This document describes the project-specific behavior implemented in this firmware for running BLE and Wi-Fi at the same time on ESP32-C3, and how GUID + IPv4 are broadcast over BLE advertising payloads.

## Summary

The firmware keeps BLE available after provisioning and advertises custom manufacturer payload data while Wi-Fi station mode is active.

- Coexistence is enabled in sdkconfig
- BLE remains available after provisioning
- Advertising payload contains GUID + IPv4
- Advertising payload is refreshed when a new STA IP is obtained

## What Is Advertised

Manufacturer payload format used by this firmware:

- 22 bytes total
- bytes [0..1]: Company ID (16-bit, little-endian)
- bytes [2..17]: node GUID (16 bytes)
- bytes [18..21]: IPv4 address (4 bytes, A.B.C.D order)

Example:

- GUID: 01:02:03:04:05:06:07:08:09:0A:0B:0C:0D:0E:0F:10
- IPv4: 192.168.1.42
- Company ID: 0xFFFF (default placeholder)
- Payload hex:
  FF FF 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 C0 A8 01 2A

## Lifecycle

### During Provisioning

- BLE provisioning transport is active (as before)
- custom provisioning endpoint VSCP-WCANG is available

### After Provisioning

- provisioning manager is deinitialized
- app BLE advertising is started through post-provision hook
- Wi-Fi stays in station mode

### On IP Change

- on IP_EVENT_STA_GOT_IP, advertising payload is rebuilt
- advertised IPv4 always follows current station IP

## Runtime Wi-Fi Power Save Control

Wi-Fi power save mode can be changed at runtime using the custom provisioning endpoint payload:

- WIFIPS:none
- WIFIPS:min
- WIFIPS:max

The selected mode is persisted in NVS key wifiPsMode and applied on next boot.

## Decoder Tools

### Decode raw manufacturer payload bytes

Use:

- tools/decode-ble-guid-ip.py "AA BB CC ..."
- echo "AA BB CC ..." | tools/decode-ble-guid-ip.py --stdin

Supported payload sizes:

- 22 bytes: CompanyID(2) + GUID(16) + IPv4(4) (default firmware format)
- 20 bytes: GUID(16) + IPv4(4) (legacy/compatibility)

### Decode directly from btmon/bluetoothctl logs

Use:

- btmon | tee /tmp/btmon.log
- tools/decode-ble-guid-ip-from-btmon.py --file /tmp/btmon.log

The btmon parser auto-extracts candidate payloads and prints decoded GUID/IP.

If btmon monitor mode is not permitted on your host, use the bluetoothctl fallback:

- tools/decode-ble-guid-ip-from-bluetoothctl.py --mac A0:76:4E:45:9E:7E --scan-time 15

You can also parse saved bluetoothctl text:

- tools/decode-ble-guid-ip-from-bluetoothctl.py --file /tmp/bluetoothctl-info.txt

## End-to-End Validation

Use the helper script:

- tools/validate-coex-flow.sh --port /dev/ttyUSB0 --monitor

The script performs build + flash and prints a coexistence checklist, including payload decode steps.

## Relevant Source Files

- main/main.c
- main/main.h
- main/CMakeLists.txt
- tools/validate-coex-flow.sh
- tools/decode-ble-guid-ip.py
- tools/decode-ble-guid-ip-from-btmon.py

## Notes

- Current advertising data is encoded as 22-byte manufacturer data with CompanyID(2) + GUID(16) + IPv4(4).
- Default Company ID in this firmware is 0xFFFF and should be replaced with an assigned value for production devices.
