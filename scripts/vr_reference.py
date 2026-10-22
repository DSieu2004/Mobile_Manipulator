"""Generate a smooth pen-tip reference for the letters VR on a Y-Z wall."""

import argparse
import csv
import math
from pathlib import Path


def smoothstep(u: float) -> tuple[float, float]:
    """Return quintic position fraction and its derivative with respect to u."""
    return 10 * u**3 - 15 * u**4 + 6 * u**5, 30 * u**2 * (1 - u) ** 2


def generate(step: float, wall_x: float, hover: float) -> list[tuple]:
    if step <= 0 or hover <= 0:
        raise ValueError("step and hover must be positive")
    ink_x = wall_x - hover
    lift_x = wall_x - max(hover + 0.08, 0.12)
    rows: list[tuple] = []
    clock = 0.0

    def add_line(name: str, start: tuple[float, float, float],
                 end: tuple[float, float, float], speed: float, down: bool) -> None:
        nonlocal clock
        distance = math.dist(start, end)
        duration = max(distance / speed, 0.25)
        samples = math.ceil(duration / step)
        duration = samples * step
        for index in range(1, samples + 1):
            u = index / samples
            blend, derivative = smoothstep(u)
            point = [a + (b - a) * blend for a, b in zip(start, end)]
            velocity = [(b - a) * derivative / duration for a, b in zip(start, end)]
            clock += step
            rows.append((clock, *point, *velocity, int(down), name))

    def move_pen(name: str, start_yz: tuple[float, float],
                 end_yz: tuple[float, float]) -> None:
        add_line(name, (ink_x, *start_yz), (lift_x, *start_yz), 0.12, False)
        add_line(name, (lift_x, *start_yz), (lift_x, *end_yz), 0.18, False)
        add_line(name, (lift_x, *end_yz), (ink_x, *end_yz), 0.12, False)

    rows.append((clock, ink_x, -0.55, 1.0, 0.0, 0.0, 0.0, 0, "start"))
    add_line("V_left", (ink_x, -0.55, 1.0), (ink_x, -0.35, 0.58), 0.12, True)
    add_line("V_right", (ink_x, -0.35, 0.58), (ink_x, -0.15, 1.0), 0.12, True)
    move_pen("lift_V_to_R", (-0.15, 1.0), (0.05, 0.58))
    add_line("R_stem", (ink_x, 0.05, 0.58), (ink_x, 0.05, 1.0), 0.12, True)

    arc_duration = 4.0
    arc_samples = math.ceil(arc_duration / step)
    arc_duration = arc_samples * step
    for index in range(1, arc_samples + 1):
        u = index / arc_samples
        blend, derivative = smoothstep(u)
        angle = math.pi / 2 - math.pi * blend
        angular_rate = -math.pi * derivative / arc_duration
        y = 0.05 + 0.22 * math.cos(angle)
        z = 0.91 + 0.09 * math.sin(angle)
        vy = -0.22 * math.sin(angle) * angular_rate
        vz = 0.09 * math.cos(angle) * angular_rate
        clock += step
        rows.append((clock, ink_x, y, z, 0.0, vy, vz, 1, "R_bowl"))

    add_line("R_leg", (ink_x, 0.05, 0.82), (ink_x, 0.34, 0.58), 0.12, True)
    add_line("retract", (ink_x, 0.34, 0.58), (lift_x, 0.34, 0.58), 0.12, False)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--step", type=float, default=0.02, help="Reference interval in seconds")
    parser.add_argument("--wall-x", type=float, default=1.125, help="Near face of the wall in meters")
    parser.add_argument("--hover", type=float, default=0.05, help="Dry-run distance from wall in meters")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1]
                        / "src/Clearpath_mobile_manipulator/mujoco/vr_reference.csv")
    args = parser.parse_args()
    rows = generate(args.step, args.wall_x, args.hover)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(("time_s", "x_m", "y_m", "z_m", "vx_m_s", "vy_m_s", "vz_m_s",
                         "pen_down", "stroke"))
        writer.writerows(rows)
    print(f"Saved {len(rows)} samples ({rows[-1][0]:.2f} s) to {args.output}")


if __name__ == "__main__":
    main()
