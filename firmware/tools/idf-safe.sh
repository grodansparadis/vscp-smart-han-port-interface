#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "$script_dir/.." && pwd)"
build_dir="$workspace_dir/build"

source /home/akhe/development/esp/esp-idf/export.sh

preferred_generator="${IDF_CMAKE_GENERATOR:-}"
if [[ -z "$preferred_generator" ]]; then
  if command -v ninja >/dev/null 2>&1; then
    preferred_generator="Ninja"
  else
    preferred_generator="Unix Makefiles"
  fi
fi

needs_fullclean=0
reason=""

if [[ -f "$build_dir/CMakeCache.txt" ]]; then
  generator="$(awk -F= '/^CMAKE_GENERATOR:INTERNAL=/{print $2; exit}' "$build_dir/CMakeCache.txt")"
  if [[ -n "$generator" && "$generator" != "$preferred_generator" ]]; then
    needs_fullclean=1
    reason="CMakeCache.txt generator is '$generator' (expected '$preferred_generator')"
  fi
fi

if [[ $needs_fullclean -eq 0 && -f "$build_dir/Makefile" && "$preferred_generator" == "Ninja" ]]; then
  needs_fullclean=1
  reason="top-level Makefile generator artifacts are present"
fi

if [[ $needs_fullclean -eq 0 && -f "$build_dir/CMakeFiles/Makefile.cmake" && "$preferred_generator" == "Ninja" ]]; then
  if grep -q 'Unix Makefiles' "$build_dir/CMakeFiles/Makefile.cmake"; then
    needs_fullclean=1
    reason="CMakeFiles/Makefile.cmake still references Unix Makefiles"
  fi
fi

if [[ $needs_fullclean -eq 1 ]]; then
  echo "Detected stale CMake generator metadata in $build_dir ($reason). Running idf.py fullclean before continuing."
  idf.py fullclean
fi

# ESP-IDF v6.0 idf.py flash invokes esptool with relative paths but from the project
# root, causing "No such file or directory: bootloader/bootloader.bin" errors.
# Detect flash/flash monitor invocations and run esptool from build/ directly.
has_flash=0
has_monitor=0
port_val=""
extra_args=()
skip_next=0
for arg in "$@"; do
  if [[ $skip_next -eq 1 ]]; then port_val="$arg"; skip_next=0; continue; fi
  if [[ "$arg" == "-p" ]]; then skip_next=1; continue; fi
  if [[ "$arg" == "flash" ]]; then has_flash=1; continue; fi
  if [[ "$arg" == "monitor" ]]; then has_monitor=1; continue; fi
  extra_args+=("$arg")
done

if [[ $has_flash -eq 0 ]]; then
  exec idf.py -G "$preferred_generator" "$@"
fi

# Flash: cd into build/ and invoke esptool directly with the generated flash_args
cd "$build_dir"
read -ra all_flash_args < <(tr '\n' ' ' < flash_args) || true
port_opts=(); [[ -n "$port_val" ]] && port_opts=(-p "$port_val")
esptool_cmd="esptool --chip esp32c3 ${port_opts[*]} --before default-reset --after hard-reset write-flash ${all_flash_args[*]}"
# Use sg to activate dialout group membership in case the session hasn't been refreshed
if id -nG "$USER" 2>/dev/null | grep -qw dialout && ! id -G 2>/dev/null | tr ' ' '\n' | grep -qx "$(getent group dialout | cut -d: -f3)"; then
  sg dialout -c "$esptool_cmd"
else
  eval "$esptool_cmd"
fi

if [[ $has_monitor -eq 1 ]]; then
  cd "$workspace_dir"
  exec idf.py "${port_opts[@]/#-p/-p}" monitor
fi