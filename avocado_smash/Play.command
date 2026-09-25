#!/bin/zsh
# Double-click in Finder or run from Terminal. Optional: --sensor-port /dev/cu.usbserial-XXXX
set -e
cd "$(dirname "$0")"
if [[ ! -x .venv/bin/python ]]; then
  python3 -m venv .venv
  .venv/bin/python -m pip install -r requirements.txt
fi
exec .venv/bin/python main.py "$@"
