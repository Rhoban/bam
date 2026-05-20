#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"

if ! command -v inotifywait >/dev/null 2>&1; then
    echo "inotifywait not found. Install inotify-tools (e.g. sudo apt install inotify-tools)."
    exit 1
fi

if [[ ! -x ../.venv/bin/sphinx-build ]] && ! command -v sphinx-build >/dev/null 2>&1; then
    echo "sphinx-build not found. Run: uv venv ../.venv && uv pip install --python ../.venv/bin/python -r requirements.txt"
    exit 1
fi

make html
echo "Starting watching..."
pkill -f "python3 -m http.server 8080" >/dev/null 2>&1 || true
cd _build/html
python3 -m http.server 8080 --bind 127.0.0.1 &
cd ../..

while true
do
    inotifywait *.rst */ --recursive
    make html
    sleep 0.5
done
