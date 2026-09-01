#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
matlab="${MATLAB_EXE:-/usr/local/MATLAB/R2026a/bin/matlab}"
parameter="${1:-weightDistF}"
shift || true

if [ "$#" -gt 0 ]; then
  values=("$@")
else
  values=(0.500 0.495 0.490 0.485 0.480 0.475 0.470 0.465 0.460)
fi

output_root="$repo_root/VehicleBalance_MF14_outputs/sweeps/$parameter"

case_name() {
  local value_text
  value_text="$(printf "%.4g" "$1" | sed 's/-/m/g; s/\./p/g')"
  printf "%s_%s" "$parameter" "$value_text"
}

cd "$repo_root"

for value in "${values[@]}"; do
  case_dir="$output_root/$(case_name "$value")"
  override_file="$case_dir/case_overrides.json"
  mkdir -p "$case_dir"

  python3 - "$override_file" "$case_dir" "$parameter" "$value" <<'PY'
import json
import sys

override_file, output_dir, parameter, value = sys.argv[1:5]
with open(override_file, "w", encoding="utf-8") as fh:
    json.dump({"outputDir": output_dir, parameter: float(value)}, fh, indent=2)
PY

  echo "Running $parameter=$value -> ${case_dir#$repo_root/}"
  "$matlab" -batch "setenv('VEHICLE_BALANCE_OVERRIDE_FILE','$override_file'); try, run('VehicleBalance_MF14.m'); catch ME, disp(getReport(ME,'extended')); exit(1); end"
done

python3 tools/run_vehicle_balance_sweep.py --skip-matlab --parameter "$parameter" --values "${values[@]}"
