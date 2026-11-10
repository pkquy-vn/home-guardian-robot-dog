#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Builds and runs the firmware host tests (posture state machine) with g++ and sanitizers.
# Usage, from anywhere: firmware/test_host/run_tests.sh
set -eu
root="$(cd "$(dirname "$0")/../.." && pwd)"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
"${CXX:-g++}" -std=c++17 -O1 -g -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion \
  -Wshadow -Werror -fsanitize=address,undefined -fno-sanitize-recover=all \
  -I"$root/common/include" -I"$root/firmware/src" \
  "$root/firmware/src/posture.cpp" "$root/firmware/test_host/test_posture.cpp" \
  -o "$out/test_posture"
"$out/test_posture"
