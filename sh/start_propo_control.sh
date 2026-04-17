#!/bin/sh
set -eu

PYTHON="/usr/bin/python3"
SCRIPT="/home/pi/src/cugo_rs485_motor_control/scripts/propo_control.py"
LOG_DIR="/home/pi/src/cugo_rs485_motor_control/log"
LOG="$LOG_DIR/propo_control.log"
PATTERN="$SCRIPT"

if pgrep -f "$PATTERN" >/dev/null 2>&1; then
  echo "propo_control.py is already running"
  exit 0
fi

mkdir -p "$LOG_DIR"

if [ "$#" -gt 0 ]; then
  nohup "$PYTHON" "$SCRIPT" "$@" >>"$LOG" 2>&1 &
else
  nohup "$PYTHON" "$SCRIPT" >>"$LOG" 2>&1 &
fi

echo "Started propo_control.py"
