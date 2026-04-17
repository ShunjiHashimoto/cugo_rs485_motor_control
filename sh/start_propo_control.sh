#!/bin/sh
set -eu

PYTHON="/usr/bin/python3"
SCRIPT="/home/pi/src/cugo_rs485_motor_control/scripts/propo_control.py"
LOG_DIR="/home/pi/src/cugo_rs485_motor_control/log"
LOG="$LOG_DIR/propo_control.log"
PATTERN="$SCRIPT"

timestamp() {
  date '+%Y-%m-%d %H:%M:%S'
}

log_line() {
  mkdir -p "$LOG_DIR"
  printf '%s [start_propo_control] %s\n' "$(timestamp)" "$1" >>"$LOG"
}

trap '
  status=$?
  if [ "$status" -ne 0 ]; then
    log_line "launcher failed with exit code ${status}"
  fi
' EXIT

if pgrep -f "$PATTERN" >/dev/null 2>&1; then
  log_line "start skipped: propo_control.py is already running"
  echo "propo_control.py is already running"
  exit 0
fi

mkdir -p "$LOG_DIR"
log_line "launch requested: $PYTHON $SCRIPT $*"

if [ "$#" -gt 0 ]; then
  nohup "$PYTHON" "$SCRIPT" "$@" >>"$LOG" 2>&1 &
else
  nohup "$PYTHON" "$SCRIPT" >>"$LOG" 2>&1 &
fi

PID=$!
sleep 1

if kill -0 "$PID" >/dev/null 2>&1; then
  log_line "launch succeeded: pid=$PID"
  echo "Started propo_control.py (pid=$PID)"
  exit 0
fi

log_line "launch failed: process exited immediately"
echo "Failed to start propo_control.py; see $LOG" >&2
exit 1
