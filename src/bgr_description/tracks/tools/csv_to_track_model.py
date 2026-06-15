#!/usr/bin/env python3
"""
CSV -> Gazebo Model (track) generator

Reads a cone ground-truth CSV with columns:
  color_id,color_name,x_m,y_m

and generates a Gazebo *model package* you can load via:
  <uri>model://track_<name></uri>

It creates:
  <out_models_root>/tracks/track_<name>/
    ├── model.config
    └── model.sdf
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
from typing import Dict, List, Tuple


DEFAULT_URI_MAP: Dict[int, str] = {
    1: "model://cone_yellow",     # Yellow
    2: "model://cone_blue",       # Blue
    3: "model://cone_orange",     # Orange (optional)
    4: "model://cone_orange_big", # Big orange (optional)
}


MODEL_CONFIG_TEMPLATE = """<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>auto-generated</name>
    <email>n/a</email>
  </author>
  <description>Auto-generated track model from CSV ground-truth cones.</description>
</model>
"""


def parse_csv(csv_path: Path) -> List[Tuple[int, str, float, float]]:
    """Returns list of (color_id, color_name, x_m, y_m)."""
    required = {"color_id", "color_name", "x_m", "y_m"}
    cones: List[Tuple[int, str, float, float]] = []

    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None or not required.issubset(set(reader.fieldnames)):
            raise ValueError(
                f"CSV missing required columns. Need {sorted(required)}, got {reader.fieldnames}"
            )

        for row_idx, row in enumerate(reader, start=2):
            try:
                color_id = int(row["color_id"])
                color_name = str(row["color_name"])
                x_m = float(row["x_m"])
                y_m = float(row["y_m"])
            except Exception as e:
                raise ValueError(f"Bad row at line {row_idx}: {row} ({e})") from e

            cones.append((color_id, color_name, x_m, y_m))

    return cones


def include_block(uri: str, name: str, x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> str:
    return f"""    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.3f}</pose>
    </include>
"""


def build_model_sdf(model_name: str, cones: List[Tuple[int, str, float, float]], uri_map: Dict[int, str]) -> str:
    lines: List[str] = []
    lines.append('<?xml version="1.0"?>\n')
    lines.append('<sdf version="1.7">\n')
    lines.append(f'  <model name="{model_name}">\n')
    lines.append('    <static>true</static>\n\n')

    counts: Dict[int, int] = {}
    for color_id, _color_name, x, y in cones:
        if color_id not in uri_map:
            # Skip unknown cone class (safe behavior)
            continue

        counts[color_id] = counts.get(color_id, 0) + 1
        uri = uri_map[color_id]

        # Keep names unique and readable
        # e.g. cone_blue_001, cone_yellow_012, etc.
        if color_id == 1:
            base = "cone_yellow"
        elif color_id == 2:
            base = "cone_blue"
        elif color_id == 3:
            base = "cone_orange"
        elif color_id == 4:
            base = "cone_orange_big"
        else:
            base = f"cone_{color_id}"

        inst_name = f"{base}_{counts[color_id]:03d}"
        lines.append(include_block(uri, inst_name, x, y))

    lines.append("\n  </model>\n")
    lines.append("</sdf>\n")
    return "".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate Gazebo track model from cone CSV.")
    ap.add_argument("--csv", required=True, help="Path to CSV file (color_id,color_name,x_m,y_m).")
    ap.add_argument(
        "--name",
        default=None,
        help="Track model name (default: track_<csv_stem>). Example: track_acceleration",
    )
    ap.add_argument(
        "--models-root",
        default=str(Path.home() / "fsa_ws/src/fsa_simulation/models"),
        help="Root models folder that Gazebo searches (default: ~/fsa_ws/src/fsa_simulation/models)",
    )

    # Allow overriding cone URIs if your folders are named differently
    ap.add_argument("--uri-yellow", default=DEFAULT_URI_MAP[1], help="URI for yellow cone model.")
    ap.add_argument("--uri-blue", default=DEFAULT_URI_MAP[2], help="URI for blue cone model.")
    ap.add_argument("--uri-orange", default=DEFAULT_URI_MAP[3], help="URI for orange cone model.")
    ap.add_argument("--uri-orange-big", default=DEFAULT_URI_MAP[4], help="URI for big orange cone model.")
    args = ap.parse_args()

    csv_path = Path(args.csv).expanduser().resolve()
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    models_root = Path(args.models_root).expanduser().resolve()
    if not models_root.exists():
        raise FileNotFoundError(
            f"models-root does not exist: {models_root}\n"
            f"Create it or pass --models-root to a valid path."
        )

    track_name = args.name
    if track_name is None:
        track_name = f"track_{csv_path.stem}"

    # Output folder: <models_root>/tracks/<track_name>/
    out_dir = models_root / "tracks" / track_name
    out_dir.mkdir(parents=True, exist_ok=True)

    uri_map = {
        1: args.uri_yellow,
        2: args.uri_blue,
        3: args.uri_orange,
        4: args.uri_orange_big,
    }

    cones = parse_csv(csv_path)
    model_sdf = build_model_sdf(track_name, cones, uri_map)
    model_config = MODEL_CONFIG_TEMPLATE.format(model_name=track_name)

    (out_dir / "model.sdf").write_text(model_sdf, encoding="utf-8")
    (out_dir / "model.config").write_text(model_config, encoding="utf-8")

    # Helpful summary
    n_total = len(cones)
    n_by_color: Dict[int, int] = {}
    for c_id, _c_name, *_ in cones:
        n_by_color[c_id] = n_by_color.get(c_id, 0) + 1

    print(f"OK: wrote Gazebo model package:\n  {out_dir}")
    print(f"  model://tracks/{track_name}  (folder name: {track_name})")
    print(f"CSV cones: {n_total}  breakdown: {n_by_color}")
    print("\nTo include in your world (inside <world>):")
    print(f"""  <include>
    <uri>model://tracks/{track_name}</uri>
    <pose>0 0 0 0 0 0</pose>
  </include>""")
    print("\nIMPORTANT: Gazebo must search your models root. Add to ~/.bashrc if needed:")
    print(f'  export GZ_SIM_RESOURCE_PATH="{models_root}:$GZ_SIM_RESOURCE_PATH"')
    print(f'  export IGN_GAZEBO_RESOURCE_PATH="{models_root}:$IGN_GAZEBO_RESOURCE_PATH"')

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
