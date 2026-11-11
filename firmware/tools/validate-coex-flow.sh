#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$script_dir/.." && pwd)"

port=""
monitor=0
skip_build=0

usage() {
  cat <<'USAGE'
Usage:
  tools/validate-coex-flow.sh --port <serial-port> [--monitor] [--skip-build]

Example:
  tools/validate-coex-flow.sh --port /dev/ttyUSB0 --monitor

What this does:
  1) Builds ESP32-C3 firmware
  2) Flashes over serial
  3) Prints a concise BLE+Wi-Fi coexistence validation checklist
  4) Optionally starts monitor
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port)
      port="${2:-}"
      shift 2
      ;;
    --monitor)
      monitor=1
      shift
      ;;
    --skip-build)
      skip_build=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

if [[ -z "$port" ]]; then
  echo "Error: --port is required"
  usage
  exit 2
fi

cd "$workspace_dir"

echo "[1/4] Target check"
if ! grep -q 'CONFIG_IDF_TARGET="esp32c3"' sdkconfig; then
  echo "Warning: sdkconfig target is not esp32c3. Continue only if this is intentional."
fi

echo "[2/4] Build"
if [[ "$skip_build" -eq 0 ]]; then
  ./tools/idf-safe.sh build
else
  echo "Skipped build by request"
fi

echo "[3/4] Flash"
./tools/idf-safe.sh -p "$port" flash

echo "[4/4] Coexistence validation checklist"
cat <<'CHECKLIST'
A. Boot and provisioning
- Confirm logs include provisioning start (if not provisioned).
- Confirm logs include "No app BLE advertising hook implemented" or "Started app BLE advertising" after provisioning end.

B. Wi-Fi + BLE concurrency
- Keep Wi-Fi traffic active (MQTT publish flood or continuous ping).
- In a second shell, run a BLE scan from a nearby Linux host:
    bluetoothctl
    scan on
- Verify device remains visible for at least 5 minutes during Wi-Fi traffic.

C. Decode advertised GUID + IPv4 payload
- Copy manufacturer payload bytes from your scanner output.
- Decode with:
  tools/decode-ble-guid-ip.py "AA BB CC ..."
- Or pipe text from stdin:
  echo "AA BB CC ..." | tools/decode-ble-guid-ip.py --stdin
- For btmon logs (auto-extract):
  btmon | tee /tmp/btmon.log
  tools/decode-ble-guid-ip-from-btmon.py --file /tmp/btmon.log
- For non-root fallback using bluetoothctl:
  tools/decode-ble-guid-ip-from-bluetoothctl.py --mac A0:76:4E:45:9E:7E --scan-time 15

D. Runtime Wi-Fi PS command (during provisioning custom endpoint)
- Send payload: WIFIPS:none
- Send payload: WIFIPS:min
- Send payload: WIFIPS:max
- Verify logs report the selected mode.

E. Reliability
- Reboot AP while device is running; verify reconnect succeeds.
- Leave soak test running for >= 1 hour with BLE scanning and Wi-Fi traffic.
CHECKLIST

if [[ "$monitor" -eq 1 ]]; then
  echo "Starting monitor on $port (Ctrl+] to exit)..."
  ./tools/idf-safe.sh -p "$port" monitor
else
  echo "Done. Re-run with --monitor to stream runtime logs."
fi
