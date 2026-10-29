# Sourced: the one way the night pipeline makes a derived file ([SdA], decision [gI8]). A builder run in the background
# and collected with a bare `wait` (always 0), or piped through `| grep | tail` (the filter's status, not the builder's),
# passed a crash silently, and an output not removed first then stood in for the rebuild (pipeline critique, 3 Oct 2026).
#   build [--log F] [--show RE] [--tail N] [--cd DIR] [--stdout] [--clear DIR]... <out>... -- <command...>
# removes each <out> (and each --clear DIR) first, runs the command (one program, not a pipeline: nothing filters its
# status; pipefail in its subshell) with its output in its own log (F, or <first out>.log), and fails, the log's last
# lines shown, unless the command exits 0 and every <out> is there again and not empty: removed first, so one that is
# there is this run's. On success it prints the log's lines matching RE, or its
# last N (1). --stdout: the command's standard output is the first <out> (its errors go to the log).
#   build_bg <build args>: the same, in the background; build_wait: waits on each of those by its own PID, 1 if any failed.
build() {
  local log="" show="" tail_n=1 dir="" to_out=0 outs=() clear=() o rc=0
  while [ $# -gt 0 ] && [ "$1" != -- ]; do
    case $1 in
      --log) log=$2; shift 2;; --show) show=$2; shift 2;; --tail) tail_n=$2; shift 2;; --cd) dir=$2; shift 2;;
      --stdout) to_out=1; shift;; --clear) clear+=("$2"); shift 2;;
      *) outs+=("$1"); shift;;
    esac
  done
  if [ "${1:-}" != -- ] || [ $# -lt 2 ] || [ ${#outs[@]} = 0 ]; then
    echo "build: usage: build [--log F] [--show RE] [--tail N] [--cd DIR] [--stdout] [--clear DIR]... <out>... -- <command...>" >&2; return 2; fi
  shift
  [ -n "$log" ] || log="${outs[0]}.log"
  for o in "${clear[@]}"; do rm -rf -- "$o"; done
  for o in "${outs[@]}"; do rm -f -- "$o"; done
  mkdir -p "$(dirname "$log")"
  if [ $to_out = 1 ]; then
    ( set -o pipefail; [ -z "$dir" ] || cd "$dir" || exit 1; "$@" ) > "${outs[0]}" 2> "$log" || rc=$?
  else
    ( set -o pipefail; [ -z "$dir" ] || cd "$dir" || exit 1; "$@" ) > "$log" 2>&1 || rc=$?
  fi
  local missing=()
  for o in "${outs[@]}"; do [ -s "$o" ] || missing+=("$o"); done
  if [ $rc != 0 ] || [ ${#missing[@]} != 0 ]; then
    [ $to_out = 1 ] && [ $rc != 0 ] && rm -f -- "${outs[0]}"   # a half-written stdout is not an output
    local what; what=$(printf '%s\n' "$@" | grep -m1 -E '\.(py|sh)$' || echo "$1")
    echo "FAILED: $(basename "$what") (exit $rc)${missing[*]:+; not made: ${missing[*]}}; log $log"
    { grep -v Zenoh "$log" 2>/dev/null || true; } | tail -6
    return 1
  fi
  if [ -n "$show" ]; then { grep -v Zenoh "$log" || true; } | { grep -E -- "$show" || true; }
  else { grep -v Zenoh "$log" || true; } | tail -"$tail_n"; fi
  return 0
}
_BUILD_PIDS=()
build_bg() { build "$@" & _BUILD_PIDS+=($!); }
build_wait() {
  local p rc=0
  for p in "${_BUILD_PIDS[@]}"; do wait "$p" || rc=1; done
  _BUILD_PIDS=(); return $rc
}
