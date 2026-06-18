#!/usr/bin/env python3
"""Animate VehicleBalance_MF14 scalar sweep plot outputs.

Example:
    python3 tools/run_vehicle_balance_sweep.py --skip-matlab

    python3 tools/run_vehicle_balance_sweep.py --parameter sprung_z --values 11.5 12 12.5 13
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


REPO_ROOT = Path(__file__).resolve().parents[1]
MATLAB_EXE = Path("/usr/local/MATLAB/R2026a/bin/matlab")
DEFAULT_VALUES = [0.500, 0.495, 0.490, 0.485, 0.480, 0.475, 0.470, 0.465, 0.460]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "VehicleBalance_MF14_outputs" / "sweeps"


@dataclass(frozen=True)
class SweepCase:
    parameter: str
    value: float
    output_dir: Path

    @property
    def label(self) -> str:
        if self.parameter == "weightDistF":
            return f"{self.parameter} = {self.value:.1%}"
        return f"{self.parameter} = {self.value:g}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parameter", default="weightDistF", help="Scalar VehicleBalance_MF14 variable to override.")
    parser.add_argument("--values", type=float, nargs="+", default=DEFAULT_VALUES, help="Sweep values.")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT, help="Root folder for case outputs.")
    parser.add_argument("--matlab", type=Path, default=MATLAB_EXE, help="MATLAB executable.")
    parser.add_argument("--run-matlab", action="store_true", help="Also run MATLAB cases before building GIFs.")
    parser.add_argument("--skip-matlab", action="store_true", help="Deprecated no-op; GIF-only is now the default.")
    parser.add_argument("--gif-duration-ms", type=int, default=650, help="Frame duration for generated GIFs.")
    return parser.parse_args()


def case_name(parameter: str, value: float) -> str:
    value_text = f"{value:.4g}".replace("-", "m").replace(".", "p")
    return f"{parameter}_{value_text}"


def build_cases(parameter: str, values: list[float], output_root: Path) -> list[SweepCase]:
    sweep_root = output_root / parameter
    return [
        SweepCase(parameter=parameter, value=value, output_dir=sweep_root / case_name(parameter, value))
        for value in values
    ]


def run_matlab_case(case: SweepCase, matlab: Path) -> None:
    case.output_dir.mkdir(parents=True, exist_ok=True)
    override_file = case.output_dir / "case_overrides.json"
    override_file.write_text(
        json.dumps({"outputDir": str(case.output_dir), case.parameter: case.value}, indent=2),
        encoding="utf-8",
    )
    matlab_override_file = str(override_file).replace("'", "''")
    matlab_command = (
        f"setenv('VEHICLE_BALANCE_OVERRIDE_FILE','{matlab_override_file}'); "
        "try, run('VehicleBalance_MF14.m'); "
        "catch ME, disp(getReport(ME,'extended')); exit(1); end"
    )

    subprocess.run(
        [str(matlab), "-batch", matlab_command],
        cwd=REPO_ROOT,
        check=True,
    )


def find_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    font_paths = [
        Path("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"),
        Path("/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf"),
    ]
    for font_path in font_paths:
        if font_path.exists():
            return ImageFont.truetype(str(font_path), size=size)
    return ImageFont.load_default()


def overlay_label(image_path: Path, label: str) -> Image.Image:
    image = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(image, "RGBA")
    font = find_font(max(86, image.width // 14))
    padding = max(30, image.width // 64)
    text_bbox = draw.textbbox((0, 0), label, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    box = (
        padding,
        padding,
        padding * 3 + text_width,
        padding * 3 + text_height,
    )
    draw.rounded_rectangle(box, radius=12, fill=(0, 0, 0, 235), outline=(255, 255, 255, 250), width=5)
    draw.text(
        (padding * 2, padding * 2),
        label,
        fill=(255, 255, 255, 255),
        font=font,
        stroke_width=4,
        stroke_fill=(0, 0, 0, 255),
    )
    return image


def common_plot_names(cases: list[SweepCase]) -> list[str]:
    plot_sets = []
    for case in cases:
        plot_sets.append({path.name for path in case.output_dir.glob("*.png")})
    if not plot_sets:
        return []
    return sorted(set.intersection(*plot_sets))


def write_gifs(cases: list[SweepCase], output_root: Path, duration_ms: int) -> list[Path]:
    gif_dir = output_root / cases[0].parameter / "gifs"
    gif_dir.mkdir(parents=True, exist_ok=True)
    gif_paths = []

    for plot_name in common_plot_names(cases):
        frames = [overlay_label(case.output_dir / plot_name, case.label) for case in cases]
        gif_path = gif_dir / f"{Path(plot_name).stem}.gif"
        frames[0].save(
            gif_path,
            save_all=True,
            append_images=frames[1:],
            duration=duration_ms,
            loop=0,
            optimize=False,
        )
        gif_paths.append(gif_path)

    return gif_paths


def main() -> None:
    args = parse_args()
    cases = build_cases(args.parameter, args.values, args.output_root)

    if args.run_matlab:
        for case in cases:
            print(f"Running {case.label} -> {case.output_dir.relative_to(REPO_ROOT)}", flush=True)
            run_matlab_case(case, args.matlab)

    gif_paths = write_gifs(cases, args.output_root, args.gif_duration_ms)
    for gif_path in gif_paths:
        print(f"Wrote {gif_path.relative_to(REPO_ROOT)}", flush=True)


if __name__ == "__main__":
    main()
