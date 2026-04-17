#!/bin/sh
set -eu

PATTERN="/home/pi/src/cugo_rs485_motor_control/scripts/propo_control.py"

if pgrep -f "$PATTERN" >/dev/null 2>&1; then
  pkill -f "$PATTERN"
  echo "Stopped propo_control.py"
else
  echo "propo_control.py is not running"
fi
