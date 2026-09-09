#!/usr/bin/env bash
#
# Thread-safety regression runner for the SITL TCP serial bridge.
#
# Runs test/serial_tcp_race_harness.c in four builds:
#
#   1. plain -O2             fixed sources in the tree — must pass byte-perfect
#   2. -fsanitize=thread     fixed sources — must run with zero reports
#   3. -fsanitize=address    fixed sources — must run with zero reports
#   4. -fsanitize=thread -DOLD_UNSAFE_PATTERN, compiled against the PRE-FIX
#      serial_tcp.c/h extracted from git history — the harness must DETECT it
#      (TSan reports and/or non-zero exit), proving it catches this bug class.
#
# The overall script exits non-zero unless 1-3 pass cleanly AND 4 is detected.
#
# Run from the repository root:  test/run-serial-tcp-race.sh
# Override the pre-fix baseline with OLD_BASELINE=<rev>.

set -u

cd "$(dirname "$0")/.."
OUT=obj/test
mkdir -p "$OUT"

# The last commit before the single-owner fix went in. Any earlier commit
# with the racy serial_tcp.c works as a demonstration baseline too.
OLD_BASELINE="${OLD_BASELINE:-d49fa7177495b6839c9f7e7bb82abfc23c485c3f}"

# Sanitizer builds need a compiler with the sanitizer runtime installed.
# Fedora splits those out of gcc (libtsan/libasan packages); clang ships them,
# so prefer clang for the sanitizer builds when it is available.
PLAINCC="${CC:-gcc}"
if [ -n "${SANCC:-}" ]; then
    :
elif command -v clang >/dev/null 2>&1; then
    SANCC=clang
else
    SANCC="$PLAINCC"
fi

# Same include set `make TARGET=SITL` uses for serial_tcp.c (see the harness
# file header for the canonical gcc line; TARGET-specific -D flags are not
# needed by the headers the harness pulls in).
INCLUDES="-Isrc/main -Isrc/platform/SIMULATOR -Isrc/platform/SIMULATOR/include \
-Ilib/main/dyad -Isrc/platform/SIMULATOR/target/SITL"

HARNESS=test/serial_tcp_race_harness.c
REAL_SRC="src/main/drivers/serial_tcp.c lib/main/dyad/dyad.c"

pass=0
fail=0

report() { # label ok
    if [ "$2" = ok ]; then
        echo "  -> $1: PASS"
        pass=$((pass+1))
    else
        echo "  -> $1: FAIL"
        fail=$((fail+1))
    fi
}

run_fixed() { # label extra_cflags outname cc
    local label="$1" flags="$2" name="$3" cc="$4"
    local log="$OUT/${name}.log"
    echo "== fixed mode: $label ($cc)"
    # shellcheck disable=SC2086
    if ! "$cc" $INCLUDES -std=gnu17 -Wall -Wextra -O2 -g $flags \
         "$HARNESS" $REAL_SRC -lpthread -o "$OUT/${name}" >"$log" 2>&1; then
        echo "  -> build failed (see $log)"
        fail=$((fail+1)); return
    fi
    if "$OUT/${name}" >>"$log" 2>&1; then
        if grep -q "WARNING: ThreadSanitizer\|ERROR: AddressSanitizer" "$log"; then
            report "$label (sanitizer reported)" fail
        else
            report "$label" ok
        fi
    else
        report "$label (non-zero exit)" fail
    fi
    tail -2 "$log" | sed 's/^/     /'
}

echo "== pre-fix sources (${OLD_BASELINE:0:9}) under ThreadSanitizer ($SANCC): must be DETECTED"
ORIG="$OUT/orig"
mkdir -p "$ORIG/drivers"
if ! git show "$OLD_BASELINE:src/main/drivers/serial_tcp.c" >"$ORIG/drivers/serial_tcp.c" \
   || ! git show "$OLD_BASELINE:src/main/drivers/serial_tcp.h" >"$ORIG/drivers/serial_tcp.h"; then
    echo "  -> could not extract pre-fix sources (bad OLD_BASELINE=$OLD_BASELINE?)"
    fail=$((fail+1))
else
    OLDLOG="$OUT/old_pattern_tsan.log"
    # -I"$ORIG" first so the harness picks up the pre-fix header.
    # shellcheck disable=SC2086
    if ! "$SANCC" -I"$ORIG" $INCLUDES -std=gnu17 -Wall -Wextra -O1 -g -fsanitize=thread \
         -DOLD_UNSAFE_PATTERN "$HARNESS" "$ORIG/drivers/serial_tcp.c" lib/main/dyad/dyad.c \
         -lpthread -o "$OUT/old_pattern_tsan" >"$OLDLOG.compile" 2>&1; then
        echo "  -> build failed (see $OLDLOG.compile)"
        fail=$((fail+1))
    else
        TSAN_OPTIONS="exitcode=88 halt_on_error=0" timeout 300 "$OUT/old_pattern_tsan" >"$OLDLOG" 2>&1
        rc=$?
        if grep -q "WARNING: ThreadSanitizer" "$OLDLOG"; then
            n=$(grep -c "WARNING: ThreadSanitizer" "$OLDLOG")
            echo "  -> pre-fix pattern: DETECTED ($n TSan warnings, exit $rc) — harness catches the bug class"
            pass=$((pass+1))
        elif [ "$rc" -ne 0 ]; then
            echo "  -> pre-fix pattern: DETECTED (exit $rc without byte-perfect run) — see $OLDLOG"
            pass=$((pass+1))
        else
            echo "  -> pre-fix pattern: NOT detected this run — see $OLDLOG"
            fail=$((fail+1))
        fi
    fi
fi

run_fixed "plain -O2" "" plain "$PLAINCC"
run_fixed "ThreadSanitizer" "-fsanitize=thread" tsan "$SANCC"
run_fixed "AddressSanitizer" "-fsanitize=address" asan "$SANCC"

echo
echo "summary: $pass pass, $fail fail"
[ "$fail" -eq 0 ]
