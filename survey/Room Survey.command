#!/bin/zsh
set -eu
cd "${0:A:h}"
python_bin=../.venv/bin/python   # LiftLab's environment at the repository root
if [[ ! -x "$python_bin" ]]; then
  python3 -m venv ../.venv
fi
if ! "$python_bin" -c 'import cv2, pycolmap, numba, skimage, fast_simplification' >/dev/null 2>&1; then
  "$python_bin" -m pip install -r requirements.txt
fi
url=http://127.0.0.1:8779
if curl -fsS --max-time 1 "$url/state" >/dev/null 2>&1; then
  open "$url"
  exit
fi
"$python_bin" server.py &
survey_pid=$!
trap 'kill "$survey_pid" 2>/dev/null || true' EXIT
until curl -fsS --max-time 1 "$url/state" >/dev/null 2>&1; do
  kill -0 "$survey_pid" || exit 1
  sleep 1
done
open "$url"
wait "$survey_pid"
