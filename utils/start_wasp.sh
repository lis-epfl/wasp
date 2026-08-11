#!/bin/bash
cd "$(dirname "$(readlink -f "$0")")/.."
VENV=${VENV:-venv}
source "$VENV/bin/activate" || { echo "Could not activate virtual environment '$VENV'"; exec bash; }
"$VENV/bin/python3" main.py
exec bash
