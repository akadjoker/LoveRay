#!/usr/bin/env bash
# Runs the conformance tests and every example headless for a few frames.
# Usage: tests/smoke.sh [path/to/love]
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOVE="${1:-$ROOT/bin/love}"
OUT="$ROOT/tests/out"
mkdir -p "$OUT"

if [[ -z "${DISPLAY:-}" ]] && command -v xvfb-run >/dev/null; then
    exec xvfb-run -a -s "-screen 0 1024x768x24" "$0" "$LOVE"
fi

run() {
    local name="$1"
    shift
    echo "== $name"
    local log status=0
    log="$("$LOVE" "$@" 2>&1)" || status=$?
    printf '%s\n' "$log" | grep -v -e '^ALSA' -e 'Screenshot saved' || true
    if [[ $status -ne 0 ]]; then
        echo "FAILED ($status): $name"
        exit 1
    fi
}

run "version" --version
run "api tests" "$ROOT/tests/api" --frames 10
run "physics tests" "$ROOT/tests/physics" --frames 10
run "shader tests" "$ROOT/tests/shader" --frames 10
run "particle tests" "$ROOT/tests/particles" --frames 10

if command -v python3 >/dev/null; then
    python3 "$ROOT/tests/make_archives.py" "$OUT"
    run "archive tests (directory)" "$ROOT/tests/archive" "$OUT/data.zip" dir
    run "archive tests (.love)" "$OUT/archive.love" "$OUT/data.zip" archive
    cat "$LOVE" "$OUT/archive.love" > "$OUT/fused"
    chmod +x "$OUT/fused"
    LOVE="$OUT/fused" run "archive tests (fused executable)" "$OUT/data.zip" fused
else
    echo "python3 not found, skipping the archive tests"
fi

for example in "$ROOT"/examples/*/; do
    name="$(basename "$example")"
    run "example $name" "$example" --frames 60 --screenshot "$name.png"
done

run "nogame" --frames 30

echo "All smoke tests passed"
