# Changelog

## 2026-04-10

### Added

- BLE post-provision advertising implementation for ESP32-C3 using NimBLE.
- Manufacturer advertisement payload with device identity and network address.
- Runtime Wi-Fi power-save control via provisioning endpoint commands:
  - WIFIPS:none
  - WIFIPS:min
  - WIFIPS:max
- Validation helpers:
  - tools/validate-coex-flow.sh
  - tools/decode-ble-guid-ip.py
  - tools/decode-ble-guid-ip-from-btmon.py
  - tools/decode-ble-guid-ip-from-bluetoothctl.py
- Documentation:
  - docs/ble-wifi-coexistence-guid-ip.md

### Changed

- BLE manufacturer payload format updated to 22 bytes:
  - CompanyID(2, little-endian) + GUID(16) + IPv4(4)
- Advertising payload refresh now occurs after IP_EVENT_STA_GOT_IP.
- BLE advertising is kept available after provisioning (coexists with Wi-Fi STA).
- README updated to point to detailed coexistence documentation.

### Fixed

- NimBLE startup path hardened to avoid startup failure when BLE host state is already initialized.
- Header macro conflicts with NimBLE logging defines resolved by namespacing project log-level constants.
