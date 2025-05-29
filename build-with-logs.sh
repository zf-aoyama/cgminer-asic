#!/usr/bin/env bash
set -euo pipefail

# Use first argument as logfile name or default to build.log
LOGFILE="${1:-build.log}"

echo "Logging build output to ${LOGFILE}"

# Redirect all stdout and stderr to tee so output is on console and in file
exec > >(tee "$LOGFILE") 2>&1

./autogen.sh
./configure --enable-bitaxe
make -j"$(nproc)"

