"""Synthetic asset generators with explicit output_dir support (falls back to CWD)."""
import os
from pathlib import Path
import yaml
from PIL import Image, ImageDraw


def generate_synthetic_maps(output_dir=None):
    base = Path(output_dir) if output_dir else Path(".")
    maps_dir = base / "maps"
    reports_dir = base / "reports"
    maps_dir.mkdir(parents=True, exist_ok=True)
    reports_dir.mkdir(parents=True, exist_ok=True)

    # 1. Empty World
    empty_path = str(maps_dir / "empty_world.png")
    if not os.path.exists(empty_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        img.save(empty_path)
        print(f"[INFO] Generated synthetic map: {empty_path}")

    # 2. Scattered Rocks
    rocks_path = str(maps_dir / "scattered_rocks.png")
    if not os.path.exists(rocks_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        # Add rocks
        draw.ellipse([80, 80, 120, 120], fill=0)
        draw.ellipse([40, 50, 60, 70], fill=0)
        draw.ellipse([140, 130, 160, 150], fill=0)
        draw.rectangle([120, 40, 135, 60], fill=0)
        img.save(rocks_path)
        print(f"[INFO] Generated synthetic map: {rocks_path}")

    # 3. Canyon Gate
    canyon_path = str(maps_dir / "canyon_gate.png")
    if not os.path.exists(canyon_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        # Vertical canyon walls with central gate gap (row 100, col 80 to 120)
        draw.rectangle([0, 95, 79, 105], fill=0)
        draw.rectangle([120, 95, 199, 105], fill=0)
        img.save(canyon_path)
        print(f"[INFO] Generated synthetic map: {canyon_path}")

    # 4. Slopes
    slopes_path = str(maps_dir / "marsyard_slopes.png")
    if not os.path.exists(slopes_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        draw.rectangle([60, 60, 140, 140], fill=120)
        draw.ellipse([90, 90, 110, 110], fill=0)
        draw.rectangle([20, 80, 40, 120], fill=0)
        draw.rectangle([160, 80, 180, 120], fill=0)
        img.save(slopes_path)
        print(f"[INFO] Generated synthetic map: {slopes_path}")

    # 5. Labyrinth
    lab_path = str(maps_dir / "marsyard_labyrinth.png")
    if not os.path.exists(lab_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        draw.rectangle([0, 50, 140, 60], fill=0)
        draw.rectangle([60, 100, 199, 110], fill=0)
        draw.rectangle([0, 150, 140, 160], fill=0)
        img.save(lab_path)
        print(f"[INFO] Generated synthetic map: {lab_path}")

    # 6. Snake Passage
    snake_path = str(maps_dir / "snake_passage.png")
    if not os.path.exists(snake_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        draw.rectangle([0, 60, 140, 70], fill=0)
        draw.rectangle([60, 130, 199, 140], fill=0)
        img.save(snake_path)
        print(f"[INFO] Generated synthetic map: {snake_path}")

    # 7. Dead End Trap
    trap_path = str(maps_dir / "dead_end_trap.png")
    if not os.path.exists(trap_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        draw.rectangle([60, 115, 140, 125], fill=0)
        draw.rectangle([60, 60, 70, 115], fill=0)
        draw.rectangle([130, 60, 140, 115], fill=0)
        img.save(trap_path)
        print(f"[INFO] Generated synthetic map: {trap_path}")

    # 8. Crater Field
    field_path = str(maps_dir / "crater_field.png")
    if not os.path.exists(field_path):
        img = Image.new("L", (200, 200), 255)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, 199, 199], outline=0, width=5)
        draw.ellipse([80, 80, 120, 120], fill=180)
        draw.ellipse([36, 116, 84, 148], fill=140)
        draw.ellipse([116, 36, 148, 84], fill=150)
        img.save(field_path)
        print(f"[INFO] Generated synthetic map: {field_path}")

def generate_default_scenarios_yaml(output_dir=None):
    base = Path(output_dir) if output_dir else Path(".")
    cfg_dir = base / "config"
    cfg_dir.mkdir(parents=True, exist_ok=True)
    config_path = str(cfg_dir / "scenarios.yaml")
    if not os.path.exists(config_path):
        scenarios = {
            "scenarios": [
                {
                    "id": "empty_straight",
                    "map_image": "maps/empty_world.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-3.0, -3.0],
                    "goal": [3.0, 3.0],
                    "reference_path": [
                        [-3.0, -3.0],
                        [-1.5, -1.5],
                        [0.0, 0.0],
                        [1.5, 1.5],
                        [3.0, 3.0]
                    ]
                },
                {
                    "id": "scattered_rocks_detour",
                    "map_image": "maps/scattered_rocks.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [-2.5, -1.5],
                        [-1.0, 1.0],
                        [1.0, 2.5],
                        [4.0, 4.0]
                    ],
                    "dynamic_obstacles": [
                        {
                            "trigger_time": 0.5,
                            "x": 1.0,
                            "y": 2.5,
                            "radius": 0.4
                        }
                    ]
                },
                {
                    "id": "canyon_gate_passage",
                    "map_image": "maps/canyon_gate.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [0.0, -4.0],
                    "goal": [0.0, 4.0],
                    "reference_path": [
                        [0.0, -4.0],
                        [0.0, -1.0],
                        [0.0, 0.0],
                        [0.0, 2.0],
                        [0.0, 4.0]
                    ]
                },
                {
                    "id": "marsyard_rough_slopes",
                    "map_image": "maps/marsyard_slopes.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [-3.5, 0.0],
                        [-3.0, 3.0],
                        [0.0, 4.0],
                        [4.0, 4.0]
                    ]
                },
                {
                    "id": "marsyard_labyrinth",
                    "map_image": "maps/marsyard_labyrinth.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [3.5, -4.0],
                        [3.5, -1.0],
                        [-3.5, -1.0],
                        [-3.5, 1.5],
                        [3.5, 1.5],
                        [3.5, 4.0],
                        [4.0, 4.0]
                    ]
                },
                {
                    "id": "canyon_gate_blocked",
                    "map_image": "maps/canyon_gate.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [0.0, -4.0],
                    "goal": [0.0, 4.0],
                    "reference_path": [
                        [0.0, -4.0],
                        [0.0, 4.0]
                    ],
                    "dynamic_obstacles": [
                        {
                            "trigger_time": 0.5,
                            "x": 0.0,
                            "y": 0.0,
                            "radius": 1.5
                        }
                    ]
                },
                {
                    "id": "marsyard_snake_passage",
                    "map_image": "maps/snake_passage.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [3.5, -4.0],
                        [3.5, 0.0],
                        [-3.5, 0.0],
                        [-3.5, 4.0],
                        [4.0, 4.0]
                    ]
                },
                {
                    "id": "dead_end_trap",
                    "map_image": "maps/dead_end_trap.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [0.0, -1.0],
                    "goal": [0.0, 4.0],
                    "reference_path": [
                        [0.0, -1.0],
                        [0.0, -3.5],
                        [3.5, -3.5],
                        [3.5, 2.5],
                        [0.0, 4.0]
                    ]
                },
                {
                    "id": "crater_field",
                    "map_image": "maps/crater_field.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [-2.5, -1.5],
                        [0.0, -2.5],
                        [2.5, 0.0],
                        [1.5, 2.5],
                        [4.0, 4.0]
                    ]
                }
            ]
        }
        with open(config_path, "w") as f:
            yaml.dump(scenarios, f, default_flow_style=False)
        print(f"[INFO] Generated default scenarios: {config_path}")
