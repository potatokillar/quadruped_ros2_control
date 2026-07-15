#!/usr/bin/env python3

import argparse
import csv
import shutil
from pathlib import Path

import numpy as np
import trimesh


def parse_args():
    package_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Generate reduced SD05 visual meshes for Gazebo."
    )
    parser.add_argument("--source", type=Path, default=package_dir / "meshes")
    parser.add_argument("--output", type=Path, default=package_dir / "meshes" / "gazebo")
    parser.add_argument("--ratio", type=float, default=0.12)
    parser.add_argument("--threshold", type=int, default=10_000)
    parser.add_argument("--minimum-faces", type=int, default=5_000)
    return parser.parse_args()


def load_mesh(path):
    mesh = trimesh.load_mesh(path, file_type="stl", process=True)
    if not isinstance(mesh, trimesh.Trimesh) or mesh.faces.size == 0:
        raise RuntimeError(f"No triangle mesh found in {path}")
    return mesh


def main():
    args = parse_args()
    if not 0.0 < args.ratio <= 1.0:
        raise ValueError("--ratio must be in the range (0, 1]")

    sources = sorted(args.source.glob("*.STL"))
    if not sources:
        raise RuntimeError(f"No STL files found in {args.source}")

    args.output.mkdir(parents=True, exist_ok=True)
    rows = []

    for source in sources:
        original = load_mesh(source)
        original_faces = len(original.faces)
        destination = args.output / source.name
        max_bounds_delta = max(float(np.linalg.norm(original.extents)) * 0.02, 1e-5)

        if original_faces <= args.threshold:
            shutil.copyfile(source, destination)
        else:
            candidate_ratios = sorted(
                set([args.ratio, 0.18, 0.25, 0.35, 0.50, 0.75])
            )
            for ratio in candidate_ratios:
                if ratio < args.ratio:
                    continue
                target_faces = max(args.minimum_faces, round(original_faces * ratio))
                reduced = original.simplify_quadric_decimation(face_count=target_faces)
                bounds_delta = float(np.abs(reduced.bounds - original.bounds).max())
                if bounds_delta <= max_bounds_delta:
                    reduced.export(destination, file_type="stl")
                    break
            else:
                shutil.copyfile(source, destination)

        generated = load_mesh(destination)
        if not np.isfinite(generated.vertices).all():
            raise RuntimeError(f"Non-finite vertex generated in {destination}")

        bounds_delta = float(np.abs(generated.bounds - original.bounds).max())
        if bounds_delta > max_bounds_delta:
            raise RuntimeError(
                f"Bounds changed by {bounds_delta:.6g} m in {destination}; "
                f"limit is {max_bounds_delta:.6g} m"
            )

        generated_faces = len(generated.faces)
        rows.append(
            {
                "mesh": source.name,
                "original_faces": original_faces,
                "gazebo_faces": generated_faces,
                "face_ratio": f"{generated_faces / original_faces:.6f}",
                "original_bytes": source.stat().st_size,
                "gazebo_bytes": destination.stat().st_size,
                "bounds_delta_m": f"{bounds_delta:.9g}",
            }
        )
        print(
            f"{source.name}: {original_faces:,} -> {generated_faces:,} faces "
            f"({destination.stat().st_size / 1024 / 1024:.2f} MiB)"
        )

    manifest = args.output / "manifest.csv"
    with manifest.open("w", encoding="ascii", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=rows[0].keys(), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(rows)

    print(
        f"Total: {sum(row['original_faces'] for row in rows):,} -> "
        f"{sum(row['gazebo_faces'] for row in rows):,} faces"
    )


if __name__ == "__main__":
    main()
