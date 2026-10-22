"""Turn canvas strokes into a safe, time-parameterized MuJoCo pen reference."""

import math

import numpy as np


BOARD_Y = (-0.55, 0.34)
BOARD_Z = (0.58, 1.0)
INK_X = 1.075
LIFT_X = 0.98
STEP = 0.1
DRAW_SPEED = 0.045
LIFT_SPEED = 0.12


def _smooth(u: float) -> float:
    return u * u * u * (10 - 15 * u + 6 * u * u)


def _world(point: tuple[float, float]) -> np.ndarray:
    x, y = point
    return np.array((INK_X,
                     BOARD_Y[1] - (BOARD_Y[1] - BOARD_Y[0]) * x,
                     BOARD_Z[1] - (BOARD_Z[1] - BOARD_Z[0]) * y))


def generate_strokes(strokes: list[list[list[float]]]) -> list[dict]:
    """Map normalized [0,1] canvas coordinates to one continuous pen program."""
    if not isinstance(strokes, list) or not 1 <= len(strokes) <= 32:
        raise ValueError("Submit between 1 and 32 strokes")
    paths = []
    for stroke in strokes:
        if not isinstance(stroke, list) or not 1 <= len(stroke) <= 4000:
            raise ValueError("Each stroke needs 1 to 4000 points")
        points = []
        for point in stroke:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError("Canvas points must be [x, y]")
            x, y = (float(value) for value in point)
            if not math.isfinite(x) or not math.isfinite(y) or not (0 <= x <= 1 and 0 <= y <= 1):
                raise ValueError("Canvas points must be finite and inside the panel")
            mapped = _world((x, y))
            if not points or np.linalg.norm(mapped - points[-1]) >= 0.004:
                points.append(mapped)
        paths.append(np.asarray(points))
    total_ink_length = sum(float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
                           for path in paths)
    if total_ink_length > 4.0:
        raise ValueError("Drawing is longer than 4 m; submit it as multiple drawings")

    rows = []
    clock = 0.0

    def append(point: np.ndarray, down: bool, name: str) -> None:
        nonlocal clock
        rows.append({"time_s": round(clock, 4), "x_m": float(point[0]),
                     "y_m": float(point[1]), "z_m": float(point[2]),
                     "pen_down": int(down), "stroke": name})

    def travel(start: np.ndarray, end: np.ndarray, name: str) -> None:
        nonlocal clock
        duration = max(float(np.linalg.norm(end - start)) / LIFT_SPEED, 0.3)
        count = max(1, math.ceil(duration / STEP))
        for i in range(1, count + 1):
            clock += STEP
            append(start + (end - start) * _smooth(i / count), False, name)

    append(paths[0][0], False, "start")
    last = paths[0][0]
    for number, path in enumerate(paths, start=1):
        if number > 1:
            previous_lift = np.array((LIFT_X, last[1], last[2]))
            next_lift = np.array((LIFT_X, path[0, 1], path[0, 2]))
            travel(last, previous_lift, f"lift_{number - 1}_to_{number}")
            travel(previous_lift, next_lift, f"lift_{number - 1}_to_{number}")
            travel(next_lift, path[0], f"lift_{number - 1}_to_{number}")

        lengths = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        total = float(lengths[-1])
        duration = max(total / DRAW_SPEED, 0.5)
        count = max(1, math.ceil(duration / STEP))
        for i in range(1, count + 1):
            fraction = _smooth(i / count)
            distance = total * fraction
            point = np.array([np.interp(distance, lengths, path[:, axis]) for axis in range(3)])
            clock += STEP
            append(point, True, f"user_{number}")
        last = path[-1]
    travel(last, np.array((LIFT_X, last[1], last[2])), "retract")
    return rows
