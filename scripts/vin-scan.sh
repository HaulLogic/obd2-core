#!/usr/bin/env bash
# Stop-ship: scan tracked files for VIN-like strings not on the synthetic allowlist.
# Never put production VINs in this script — only known test/fake values.
#
# Compatible with bash 3.2+ (macOS) and bash 5 (CI).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

# 17-char VINs allowed in this public repo (mocks, fixtures, unit tests only).
# Synthetic/test VINs already used in mocks and unit tests (all …000001 style serials).
ALLOWLIST_REGEX='^(1GCHK23224F000001|1GCHK23124F000001|1GCHK23114F000001|1GCHK23164F000001|1GCHK2312LF000001|1GCHK2322LF000001|1FTHK23124F000001|1G1YY22G465000001|JH4KA7660PC000001|1HGCM82633A004352|1GTESTVNTEST00001|WMWRE33546T000001|1FUJGLDR6DS000001)$'

is_allowed() {
  echo "$1" | grep -Eq "$ALLOWLIST_REGEX"
}

fail=0
tmp="$(mktemp)"
trap 'rm -f "$tmp"' EXIT

# git grep may exit 1 when no matches — that is OK.
set +e
git grep -I -n -E '[A-HJ-NPR-Z0-9]{17}' -- \
  ':!**/target/**' \
  ':!**/*.lock' \
  ':!**/*.png' \
  ':!**/*.jpg' \
  ':!**/*.pdf' \
  >"$tmp" 2>/dev/null
set -e

while IFS= read -r line || [[ -n "${line:-}" ]]; do
  [[ -z "${line:-}" ]] && continue
  tokens="$(printf '%s\n' "$line" | grep -oE '[A-HJ-NPR-Z0-9]{17}' || true)"
  while IFS= read -r token || [[ -n "${token:-}" ]]; do
    [[ -z "${token:-}" ]] && continue
    if is_allowed "$token"; then
      continue
    fi
    # Hex payload lines in synthetic fixtures
    case "$line" in
      *payload_hex*|*raw_hex*) continue ;;
    esac
    echo "FORBIDDEN VIN-like token (not on allowlist): $token"
    echo "  at: $line"
    fail=1
  done <<<"$tokens"
done <"$tmp"

# Filenames under raw-captures must not look like VIN-prefixed partner dumps
if [[ -d raw-captures ]]; then
  while IFS= read -r f; do
    [[ -z "$f" ]] && continue
    base="$(basename "$f")"
    case "$base" in
      [A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9][A-HJ-NPR-Z0-9]-*)
        vin="${base:0:17}"
        if ! is_allowed "$vin"; then
          echo "FORBIDDEN partner-style capture filename: $f"
          fail=1
        fi
        ;;
    esac
  done < <(find raw-captures -type f 2>/dev/null || true)
fi

if [[ "$fail" -ne 0 ]]; then
  echo ""
  echo "vin-scan FAILED. Add only synthetic/test VINs to ALLOWLIST_REGEX in scripts/vin-scan.sh."
  echo "Never commit production VINs (including in docs/examples)."
  exit 1
fi

echo "vin-scan OK (only allowlisted synthetic VINs found)"
