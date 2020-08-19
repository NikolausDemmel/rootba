#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
BIN_DIR="$SCRIPT_DIR/../../bin"
TEST_DATA="$SCRIPT_DIR/../../data/rootba/test/bal-ladybug-problem-49-7776-pre-shrink-1800.txt"

set -x
set -e

cd "$SCRIPT_DIR"
rm -f ba_log.json
time "$BIN_DIR"/bal --input "$TEST_DATA" --max-num-iterations 2
[ -f ba_log.json ] && echo ok
rm -f ba_log.json
