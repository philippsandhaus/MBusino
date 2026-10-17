#!/usr/bin/env bash
# Build all MBusino firmware variants with arduino-cli.
#
#   tools/build-all.sh [--release] [variant ...]
#
# All variants are built from the one sketch in src/MBusino, the board selects
# shield or Nano, -DMBUSINO_SLAVES selects 1 or 5 slaves (see src/MBusino/config.h).
# Without --release the binaries go to firmware-build/<variant>/<board>/ only.
# With --release the release files are also copied into src/<name>/build/<board>/
# under their old names (e.g. MBusinoNano5S.ino.bin), the paths the tutorials link to.
# Variants: MBusino-d1, MBusino-s2, MBusino5S-d1, MBusino5S-s2, MBusinoNano, MBusinoNano5S (default: all)
#
# Requirements: arduino-cli with the esp32 and esp8266 cores and the libraries
# listed in the README.

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/firmware-build"
RELEASE=0

SKETCH=MBusino

# variant -> "release name, fqbn, M-Bus slaves"
variant_info() {
  case "$1" in
    MBusino-d1)    echo "MBusino esp8266:esp8266:d1_mini 1" ;;
    MBusino-s2)    echo "MBusino esp32:esp32:lolin_s2_mini 1" ;;
    MBusino5S-d1)  echo "MBusino5S esp8266:esp8266:d1_mini 5" ;;
    MBusino5S-s2)  echo "MBusino5S esp32:esp32:lolin_s2_mini 5" ;;
    MBusinoNano)   echo "MBusinoNano esp32:esp32:makergo_c3_supermini 1" ;;
    MBusinoNano5S) echo "MBusinoNano5S esp32:esp32:makergo_c3_supermini 5" ;;
    *) return 1 ;;
  esac
}
ALL_VARIANTS="MBusino-d1 MBusino-s2 MBusino5S-d1 MBusino5S-s2 MBusinoNano MBusinoNano5S"

VARIANTS=""
for arg in "$@"; do
  case "$arg" in
    --release) RELEASE=1 ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) variant_info "$arg" >/dev/null || { echo "unknown variant: $arg" >&2; exit 1; }
       VARIANTS="$VARIANTS $arg" ;;
  esac
done
[ -z "$VARIANTS" ] && VARIANTS="$ALL_VARIANTS"

FAILED=""
for v in $VARIANTS; do
  read -r name fqbn slaves <<< "$(variant_info "$v")"
  board="${fqbn//:/.}"
  bp="$OUT/$v/$board"
  echo "=== $v ($fqbn)"
  if ! arduino-cli compile -b "$fqbn" --build-path "$bp" \
       --build-property "compiler.cpp.extra_flags=-DMBUSINO_SLAVES=$slaves" "$ROOT/src/$SKETCH" 2>&1 | grep -E "error|Sketch uses|Der Sketch|Globale|Global variables|^\. (Variables|Code in flash)" ; then
    :
  fi
  if [ ! -f "$bp/$SKETCH.ino.bin" ]; then
    FAILED="$FAILED $v"
    continue
  fi
  if [ "$RELEASE" = 1 ]; then
    dest="$ROOT/src/$name/build/$board"
    mkdir -p "$dest"
    for ext in ino.bin ino.bootloader.bin ino.partitions.bin ino.merged.bin ino.elf; do
      [ -f "$bp/$SKETCH.$ext" ] && cp "$bp/$SKETCH.$ext" "$dest/$name.$ext"
    done
    echo "    release files copied to src/$name/build/$board/"
  fi
done

if [ -n "$FAILED" ]; then
  echo "FAILED:$FAILED" >&2
  exit 1
fi
echo "all builds OK, output in ${OUT#$ROOT/}/"
