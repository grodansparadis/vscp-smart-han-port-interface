#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$script_dir/.." && pwd)"
build_dir="$workspace_dir/build"

port=""
run_monitor=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -p|--port)
      if [[ $# -lt 2 ]]; then
        echo "Missing value for $1" >&2
        exit 2
      fi
      port="$2"
      shift 2
      ;;
    --monitor)
      run_monitor=1
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      echo "Usage: $0 -p <serial-port> [--monitor]" >&2
      exit 2
      ;;
  esac
done

if [[ -z "$port" ]]; then
  echo "Serial port is required. Use -p <serial-port>." >&2
  exit 2
fi

if [[ ! -f "$build_dir/flash_args" ]]; then
  echo "Missing $build_dir/flash_args. Build the project first." >&2
  exit 1
fi

source /home/akhe/development/esp/esp-idf/export.sh

echo "Flashing existing build artifacts (no build step)..."
cd "$build_dir"
read -ra all_flash_args < <(tr '\n' ' ' < flash_args) || true
python -m esptool --chip esp32c3 -p "$port" --before default-reset --after hard-reset write-flash "${all_flash_args[@]}"

if [[ $run_monitor -eq 1 ]]; then
  cd "$workspace_dir"
  exec idf.py -p "$port" monitor
fi
