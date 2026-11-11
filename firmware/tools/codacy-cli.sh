#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
launcher="${CODACY_LAUNCHER:-${repo_root}/build/_deps/vscp_firmware-src/.codacy/cli.sh}"

if [[ ! -x "${launcher}" ]]; then
  echo "Codacy launcher not found at ${launcher}" >&2
  echo "Run 'idf.py reconfigure' first so CMake fetches vscp-firmware, or set CODACY_LAUNCHER." >&2
  exit 1
fi

# Common defaults for this firmware workspace.
export CODACY_PROJECT_TOKEN="${CODACY_PROJECT_TOKEN:-}"

cd "${repo_root}"
exec "${launcher}" "$@"
