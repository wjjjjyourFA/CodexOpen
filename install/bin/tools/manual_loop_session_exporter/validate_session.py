#!/usr/bin/env python3
import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate a Manual-Loop-Closure-Tools session.")
    parser.add_argument("session", type=Path)
    args = parser.parse_args()

    root = args.session.expanduser().resolve()
    tum_path = root / "optimized_poses_tum.txt"
    g2o_path = root / "pose_graph.g2o"
    keyframe_dir = root / "key_point_frame"
    errors = []

    for path in (tum_path, g2o_path, keyframe_dir):
        if not path.exists():
            errors.append(f"missing: {path}")
    if errors:
        print("INVALID")
        print("\n".join(errors))
        return 1

    tum_lines = [line for line in tum_path.read_text().splitlines()
                 if line.strip() and not line.lstrip().startswith("#")]
    g2o_lines = [line for line in g2o_path.read_text().splitlines()
                 if line.strip()]
    vertices = [line for line in g2o_lines
                if line.startswith("VERTEX_SE3:QUAT ")]
    edges = [line for line in g2o_lines
             if line.startswith("EDGE_SE3:QUAT ")]
    pcds = sorted(
        (path for path in keyframe_dir.glob("*.pcd") if path.stem.isdigit()),
        key=lambda path: int(path.stem))

    if [path.name for path in pcds] != [
            f"{index}.pcd" for index in range(len(pcds))]:
        errors.append("PCD files are not numbered continuously from 0 to N-1")
    if not (len(tum_lines) == len(vertices) == len(pcds)):
        errors.append(
            f"count mismatch: TUM={len(tum_lines)}, "
            f"vertices={len(vertices)}, PCD={len(pcds)}")
    if len(edges) != max(0, len(vertices) - 1):
        errors.append("sequential edge count mismatch")
    if any(len(line.split()) < 8 for line in tum_lines):
        errors.append("one or more TUM lines have fewer than 8 fields")
    if any(len(line.split()) != 31 for line in edges):
        errors.append("one or more g2o edges have an invalid information matrix")

    if errors:
        print("INVALID")
        print("\n".join(errors))
        return 1
    print(f"VALID: {len(pcds)} keyframes, {len(edges)} odometry edges, "
          f"root={root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
