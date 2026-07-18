#!/usr/bin/env bash
set -euo pipefail
ulimit -c 0

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
work_dir=${1:-"$repo_root/build/native-safety-sanitizers"}
evidence_dir=${2:-"$repo_root/build/test-results"}
mkdir -p "$work_dir" "$evidence_dir"

real_cxx=${CXX:-c++}
asan_library=$("$real_cxx" -print-file-name=libasan.so)
ubsan_library=$("$real_cxx" -print-file-name=libubsan.so)
for library in "$asan_library" "$ubsan_library"; do
  if [[ ! -f "$library" ]]; then
    echo "sanitizer runtime not found: $library" >&2
    exit 1
  fi
done

compiler_wrapper="$work_dir/sanitized-cxx"
cat > "$compiler_wrapper" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
exec "${CPPYY_KIT_SANITIZER_REAL_CXX:?}" \
  -fsanitize=address,undefined -fno-omit-frame-pointer "$@"
EOF
chmod +x "$compiler_wrapper"
export CPPYY_KIT_SANITIZER_REAL_CXX="$real_cxx"

probe="$work_dir/native-safety-sanitizer-probe"
"$compiler_wrapper" -O0 -g \
  "$repo_root/rclcpp_kit/tests/native_safety_sanitizer_probe.cpp" \
  -o "$probe"

preload="$asan_library"
if [[ -n "${LD_PRELOAD:-}" ]]; then
  preload="$preload:$LD_PRELOAD"
fi

lsan_scope_library="$work_dir/native-generated-lsan-scope.so"
"$compiler_wrapper" -shared -fPIC -O0 -g \
  "$repo_root/rclcpp_kit/tests/native_generated_lsan_scope.cpp" \
  -o "$lsan_scope_library"
readelf -d "$lsan_scope_library" | grep -q 'Shared library: \[libasan\.so'

expect_failure() {
  local mode=$1
  local pattern=$2
  local log="$work_dir/${mode}-probe.log"
  set +e
  LD_PRELOAD="$preload" \
    ASAN_OPTIONS="detect_leaks=$([[ $mode == lsan ]] && echo 1 || echo 0):halt_on_error=1" \
    UBSAN_OPTIONS="halt_on_error=1:print_stacktrace=1" \
    "$probe" "$mode" > "$log" 2>&1
  local status=$?
  set -e
  if [[ $status -eq 0 ]] || ! grep -E -q "$pattern" "$log"; then
    echo "$mode failure probe did not prove its runtime" >&2
    cat "$log" >&2
    exit 1
  fi
}

expect_failure asan "AddressSanitizer: heap-buffer-overflow"
expect_failure ubsan "runtime error: signed integer overflow"
expect_failure lsan "LeakSanitizer: detected memory leaks"

cache_home="$work_dir/cache-home"
base_cache="$work_dir/base-cache"
rm -rf "$cache_home" "$base_cache"
mkdir -p "$cache_home" "$base_cache"
common_env=(
  "CXX=$compiler_wrapper"
  "CPPYY_KIT_CACHE_DIR=$base_cache"
  "CPPYY_KIT_NO_AUTOPCH=1"
  "XDG_CACHE_HOME=$cache_home"
  "LD_PRELOAD=$preload"
  "ASAN_OPTIONS=detect_leaks=0:halt_on_error=1"
  "UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1"
)

helper="$repo_root/rclcpp_kit/tests/_native_safety_stress_helper.py"
env "${common_env[@]}" timeout 300 python "$helper" --compile-only \
  > "$work_dir/prebuild.log" 2>&1

mapfile -t native_libraries < <(
  find "$cache_home/cppyy_kit" -type f -name '*.so' | sort
)
if [[ ${#native_libraries[@]} -lt 5 ]]; then
  echo "expected at least five cached native glue libraries" >&2
  printf '%s\n' "${native_libraries[@]}" >&2
  exit 1
fi
for library in "${native_libraries[@]}"; do
  readelf -d "$library" | grep -q 'Shared library: \[libasan\.so'
  readelf -d "$library" | grep -q 'Shared library: \[libubsan\.so'
done

stress_evidence="$evidence_dir/native-safety-sanitized.json"
env "${common_env[@]}" timeout 300 python "$helper" \
  --evidence "$stress_evidence" > "$work_dir/stress.log" 2>&1
grep -q "NATIVE_SAFETY_STRESS_OK" "$work_dir/stress.log"
if grep -E -q 'AddressSanitizer|runtime error:' "$work_dir/stress.log"; then
  cat "$work_dir/stress.log" >&2
  exit 1
fi
python - "$stress_evidence" "${#native_libraries[@]}" <<'PY'
import json
import sys

with open(sys.argv[1]) as stream:
    evidence = json.load(stream)
if len(evidence["loaded_native_glue"]) != int(sys.argv[2]):
    raise SystemExit("sanitized cached glue was not loaded by the stress run")
PY

lsan_cache_home="$work_dir/lsan-cache-home"
lsan_base_cache="$work_dir/lsan-base-cache"
rm -rf "$lsan_cache_home" "$lsan_base_cache"
mkdir -p "$lsan_cache_home" "$lsan_base_cache"
lsan_helper="$repo_root/rclcpp_kit/tests/_native_generated_lsan_helper.py"
env \
  "CXX=$compiler_wrapper" \
  "CPPYY_KIT_CACHE_DIR=$lsan_base_cache" \
  "CPPYY_KIT_NO_AUTOPCH=1" \
  "XDG_CACHE_HOME=$lsan_cache_home" \
  "LD_PRELOAD=$preload" \
  "ASAN_OPTIONS=detect_leaks=0:halt_on_error=1" \
  "UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1" \
  timeout 300 python "$lsan_helper" --prebuild \
  > "$work_dir/lsan-prebuild.log" 2>&1
grep -q "NATIVE_GENERATED_LSAN_PREBUILD_OK" "$work_dir/lsan-prebuild.log"

mapfile -t lsan_libraries < <(
  find "$lsan_cache_home/cppyy_kit/native-pipelines" -type f -name '*.so' | sort
)
if [[ ${#lsan_libraries[@]} -ne 2 ]]; then
  echo "expected exactly two generated LSan pipeline libraries" >&2
  printf '%s\n' "${lsan_libraries[@]}" >&2
  exit 1
fi
for library in "${lsan_libraries[@]}"; do
  readelf -d "$library" | grep -q 'Shared library: \[libasan\.so'
  readelf -d "$library" | grep -q 'Shared library: \[libubsan\.so'
done

lsan_env=(
  "CXX=$real_cxx"
  "CPPYY_KIT_CACHE_DIR=$lsan_base_cache"
  "CPPYY_KIT_NO_AUTOPCH=1"
  "XDG_CACHE_HOME=$lsan_cache_home"
  "LD_PRELOAD=$preload:$lsan_scope_library"
  "ASAN_OPTIONS=detect_leaks=1:leak_check_at_exit=0:halt_on_error=1"
  "LSAN_OPTIONS=report_objects=1"
  "UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1"
)

clean_lsan_evidence="$evidence_dir/native-generated-lsan-clean.json"
env "${lsan_env[@]}" timeout 300 python "$lsan_helper" \
  --case clean --after-session-close --evidence "$clean_lsan_evidence" \
  > "$work_dir/lsan-clean.log" 2>&1
grep -q "NATIVE_GENERATED_LSAN_CHECK case=clean result=0" \
  "$work_dir/lsan-clean.log"
if grep -q 'LeakSanitizer: detected memory leaks' "$work_dir/lsan-clean.log"; then
  cat "$work_dir/lsan-clean.log" >&2
  exit 1
fi

leak_lsan_evidence="$evidence_dir/native-generated-lsan-intentional-leak.json"
set +e
env "${lsan_env[@]}" timeout 300 python "$lsan_helper" \
  --case intentional-leak --after-session-close \
  --evidence "$leak_lsan_evidence" \
  > "$work_dir/lsan-intentional-leak.log" 2>&1
leak_status=$?
set -e
if [[ $leak_status -ne 1 ]] || \
   ! grep -q "NATIVE_GENERATED_LSAN_CHECK case=intentional-leak result=1" \
     "$work_dir/lsan-intentional-leak.log" || \
   ! grep -q 'LeakSanitizer: detected memory leaks' \
     "$work_dir/lsan-intentional-leak.log" || \
   ! grep -q 'Direct leak of 65537 byte(s) in 1 object(s)' \
     "$work_dir/lsan-intentional-leak.log"; then
  echo "intentional generated-glue leak did not prove the LSan gate" >&2
  cat "$work_dir/lsan-intentional-leak.log" >&2
  exit 1
fi

python - "$clean_lsan_evidence" "$leak_lsan_evidence" \
  "${lsan_libraries[@]}" <<'PY'
import json
from pathlib import Path
import sys

clean = json.loads(Path(sys.argv[1]).read_text())
leak = json.loads(Path(sys.argv[2]).read_text())
libraries = {str(Path(value).resolve()) for value in sys.argv[3:]}
if clean["lsan_check_result"] != 0 or leak["lsan_check_result"] != 1:
    raise SystemExit("explicit generated-glue LSan results are inconsistent")
for evidence in (clean, leak):
    if evidence["artifact"] not in libraries:
        raise SystemExit("LSan evidence did not identify a prebuilt generated library")
    for field in (
            "artifact_loaded", "compile_cache_hit",
            "explicit_check_before_interpreter_shutdown",
            "session_closed_before_check",
            "python_proxies_dropped_before_check"):
        if evidence[field] is not True:
            raise SystemExit("generated-glue LSan evidence did not prove %s" % field)
    if evidence["generated_messages"] != 1:
        raise SystemExit("generated-glue LSan path did not process exactly one message")
    if evidence["python_boundary_crossings"] != 0:
        raise SystemExit("generated-glue LSan path crossed the Python hot boundary")
PY

report="$evidence_dir/native-safety-sanitizers.txt"
{
  echo "architecture=$(uname -m)"
  echo "compiler=$("$real_cxx" --version | head -n 1)"
  echo "asan_runtime=$asan_library"
  echo "ubsan_runtime=$ubsan_library"
  echo "asan_failure_probe=verified"
  echo "ubsan_failure_probe=verified"
  echo "lsan_failure_probe=verified"
  echo "native_glue_asan=passed"
  echo "native_glue_ubsan=passed"
  echo "native_glue_lsan=passed"
  echo "native_glue_lsan_clean=explicit_recoverable_check_zero"
  echo "native_glue_lsan_intentional_leak=explicit_recoverable_check_one"
  echo "lsan_scope=generated pipeline allocations checked after managed session teardown"
  echo "lsan_scope_library=$lsan_scope_library"
  printf 'instrumented_library=%s\n' "${native_libraries[@]}"
  printf 'lsan_instrumented_library=%s\n' "${lsan_libraries[@]}"
} > "$report"

echo "NATIVE_SAFETY_SANITIZERS_OK"
