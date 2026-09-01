#!/usr/bin/env python3
"""
Benchmark Map Generator for Path Planning
Generates 3 test scenarios (PGM + YAML) for Nav2 evaluation:
1. Rock Field (Obstacle slalom & dynamic avoidance)
2. Narrow Corridor (Precision passage through a narrow gate)
3. Dead-End Pocket (Cul-de-sac requiring Reeds-Shepp reverse driving)
"""

import os
import sys
from pathlib import Path


def main():
    # Detect workspace root (assuming script is located in testing/PathPlanner/scripts/)
    script_dir = Path(__file__).resolve().parent
    ws_root = script_dir.parents[2]  # Go up 3 levels to reach workspace root
    output_dir = ws_root / "custom_maps"
    output_dir.mkdir(parents=True, exist_ok=True)

    W, H = 200, 200  # 10m x 10m grid (Resolution: 0.05m/pixel)

    def save_map(name, grid):
        pgm_path = output_dir / f"{name}.pgm"
        yaml_path = output_dir / f"{name}.yaml"

        # Save PGM image (P2 ASCII format)
        with open(pgm_path, "w") as f:
            f.write(f"P2\n{W} {H}\n255\n")
            for row in grid:
                f.write(" ".join(str(val) for val in row) + "\n")

        # Save YAML configuration
        with open(yaml_path, "w") as f:
            f.write(
                f"image: {name}.pgm\n"
                f"resolution: 0.05\n"
                f"origin: [-5.0, -5.0, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )
        print(f"  [✓] Created: {yaml_path.name} and {pgm_path.name}")

    print(f"\n🗺️  Generating Benchmark Test Maps in: {output_dir}\n")

    # 1. Rock Field (حقل صخور متباعدة بممرات واسعة 1.5 متر)
    grid1 = [[254] * W for _ in range(H)]
    for y in range(H):
        for x in range(W):
            if x < 4 or x >= W - 4 or y < 4 or y >= H - 4:
                grid1[y][x] = 0  # Perimeter walls
    rocks = [
        (65, 85, 8),    # Top Left
        (135, 85, 8),   # Top Right
        (100, 130, 9),  # Center
        (50, 165, 8),   # Bottom Left
        (150, 165, 8),  # Bottom Right
    ]
    for cx, cy, r in rocks:
        for y in range(H):
            for x in range(W):
                if (x - cx) ** 2 + (y - cy) ** 2 <= r ** 2:
                    grid1[y][x] = 0
    save_map("rock_field", grid1)

    # 2. Narrow Corridor (ممر ضيق بعرض 1.5 متر)
    grid2 = [[254] * W for _ in range(H)]
    for y in range(H):
        for x in range(W):
            if x < 4 or x >= W - 4 or y < 4 or y >= H - 4:
                grid2[y][x] = 0  # Perimeter walls
            if 95 <= y <= 105 and not (85 <= x <= 115):
                grid2[y][x] = 0  # Wall with 1.5m gate in the center
    save_map("narrow_corridor", grid2)

    # 3. Dead-End Pocket (جيب مسدود يتطلب الرجوع للخلف Reeds-Shepp Reverse)
    grid3 = [[254] * W for _ in range(H)]
    for y in range(H):
        for x in range(W):
            if x < 4 or x >= W - 4 or y < 4 or y >= H - 4:
                grid3[y][x] = 0  # Perimeter walls
            # U-shaped pocket box
            if (
                (70 <= x <= 130 and 130 <= y <= 135)
                or (70 <= x <= 75 and 80 <= y <= 135)
                or (125 <= x <= 130 and 80 <= y <= 135)
            ):
                grid3[y][x] = 0
    save_map("dead_end", grid3)

    print("\n🎉 All 3 Benchmark Maps generated successfully!\n")
    print("To launch any map with Nav2, run:")
    print(f"  ros2 launch erc_path_planner path_planning.launch.py map:={output_dir}/<map_name>.yaml\n")


if __name__ == "__main__":
    main()
