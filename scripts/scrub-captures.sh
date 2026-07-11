#!/usr/bin/env bash
# Scrub real VINs from a *private* capture directory before graduation.
#
# Usage (never pass a real VIN on a shared shell history if avoidable):
#   REAL_VIN='…' FAKE_VIN='1GTESTVNTEST00001' \
#     ./scripts/scrub-captures.sh /path/to/private/captures /path/to/scrubbed/out
#
# Does not commit anything. Review output with: rg -n '[A-HJ-NPR-Z0-9]{17}' OUTDIR
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: REAL_VIN=... [FAKE_VIN=1GTESTVNTEST00001] $0 SRC_DIR OUT_DIR" >&2
  exit 2
fi

SRC="$(cd "$1" && pwd)"
OUT="$2"
REAL_VIN="${REAL_VIN:-}"
FAKE_VIN="${FAKE_VIN:-1GTESTVNTEST00001}"

if [[ -z "$REAL_VIN" ]]; then
  echo "REAL_VIN env var is required (do not hardcode production VINs in scripts)." >&2
  exit 2
fi

if [[ ${#REAL_VIN} -ne 17 ]]; then
  echo "REAL_VIN must be exactly 17 characters" >&2
  exit 2
fi
if [[ ${#FAKE_VIN} -ne 17 ]]; then
  echo "FAKE_VIN must be exactly 17 characters (got ${#FAKE_VIN}: $FAKE_VIN)" >&2
  exit 2
fi

export REAL_VIN FAKE_VIN

mkdir -p "$OUT"
# Copy tree then scrub
if command -v rsync >/dev/null 2>&1; then
  rsync -a --exclude '.git' "$SRC/" "$OUT/"
else
  cp -R "$SRC/." "$OUT/"
fi

# Content scrub
find "$OUT" -type f \( \
    -name '*.obd2raw' -o -name '*.obd2rec' -o -name '*.json' -o -name '*.csv' -o -name '*.txt' -o -name '*.md' \
  \) -print0 \
  | while IFS= read -r -d '' f; do
      perl -pi -e 's/\Q$ENV{REAL_VIN}\E/$ENV{FAKE_VIN}/g' "$f"
    done

# Filename scrub
find "$OUT" -depth -name "*${REAL_VIN}*" -print0 \
  | while IFS= read -r -d '' f; do
      dir="$(dirname "$f")"
      base="$(basename "$f")"
      new="${base//$REAL_VIN/scrubbed}"
      mv "$f" "$dir/$new"
    done

echo "Scrubbed copy at: $OUT"
echo "Spot-check (should only show FAKE_VIN if any 17-char tokens):"
if command -v rg >/dev/null 2>&1; then
  rg -n '[A-HJ-NPR-Z0-9]{17}' "$OUT" || true
else
  grep -REn '[A-HJ-NPR-Z0-9]{17}' "$OUT" || true
fi
