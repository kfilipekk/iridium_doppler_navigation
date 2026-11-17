#!/usr/bin/env bash
#start a field session: the recorder and the live view, both detached so closing the terminal
#(or Claude) does not stop them
#
#  tools/soop_field.sh NAME [PLAN]      PLAN as soop_session.py takes it (default: record 4 h)
#  tools/soop_field.sh stop
#
#Files go to ~/navcore-sessions/NAME.csv; the live view is http://localhost:8818.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
VENV=${VENV_DIR:-$HOME/.cache/navcore/fieldvenv}
OUT=${SESSION_DIR:-$HOME/navcore-sessions}
PORT=${PORT:-8818}
if [ ! -x "$VENV/bin/python" ]; then
    echo "=== creating the field venv at $VENV ==="
    python3 -m venv "$VENV"
    "$VENV/bin/pip" install --quiet pymavlink pyserial numpy sgp4
fi

if [ "${1:-}" = "stop" ]; then
    pkill -f "$HERE/soop_session.py" && echo "recorder stopped" || echo "no recorder running"
    pkill -f "$HERE/soop_live.py" && echo "live view stopped" || echo "no live view running"
    exit 0
fi
[ $# -ge 1 ] || { sed -n '4,8p' "$0"; exit 2; }
NAME=$1
PLAN=${2:-on:14400}
mkdir -p "$OUT"
CSV="$OUT/$NAME.csv"
TLE="$OUT/$NAME.tle"

#the sky view needs today's orbits; fetch them beside the session, never into the repo's copy
if [ ! -s "$TLE" ]; then
    tmp=$(mktemp -d)
    cp "$HERE/../sitl/tle/fetch_iridium_tle.sh" "$tmp/"
    bash "$tmp/fetch_iridium_tle.sh" && cp "$tmp/iridium-next.tle" "$TLE" || echo "no TLE fetched: the live view runs without its sky"
    rm -rf "$tmp"
fi

setsid nohup "$VENV/bin/python" "$HERE/soop_session.py" "$CSV" "$PLAN" > "$OUT/$NAME.recorder.log" 2>&1 < /dev/null &
sleep 2
LIVE=("$VENV/bin/python" "$HERE/soop_live.py" "$CSV" --port "$PORT")
[ -s "$TLE" ] && LIVE+=(--tle "$TLE")
setsid nohup "${LIVE[@]}" > "$OUT/$NAME.live.log" 2>&1 < /dev/null &
sleep 3
echo "recording to $CSV (plan $PLAN); recorder output in $OUT/$NAME.recorder.log"
echo "live view: http://localhost:$PORT"
grep -v "could not get home" "$OUT/$NAME.recorder.log" | tail -3 || true
