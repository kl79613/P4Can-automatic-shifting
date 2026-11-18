#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
CXX=${CXX:-c++}
FLAGS="-std=c++17 -Wall -Wextra -Wno-unused-function"
case "$(uname -s)" in
  Darwin)
    SDK=${SDKROOT:-/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk}
    if [ -d "$SDK/usr/include/c++/v1" ]; then
      FLAGS="$FLAGS -isysroot $SDK -isystem $SDK/usr/include/c++/v1"
    fi
    ;;
esac
for test in test_gear_shift_control test_v12_live_decision test_gear_hint_display; do
  echo "==> $test"
  "$CXX" $FLAGS -I"$ROOT/src" "$ROOT/tests/$test.cpp" -o "/tmp/$test"
  "/tmp/$test"
done
