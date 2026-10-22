"""Connect the Husky MuJoCo scene to the OCS2 mobile manipulator MPC."""

import argparse
from bisect import bisect_right
import csv
import json
import math
import os
from pathlib import Path
import queue
import sys
import threading
import time
import traceback

import mujoco
import mujoco.viewer as mjviewer
import numpy as np
import rclpy
from ocs2_msgs.msg import MpcFlattenedController, MpcInput, MpcObservation, MpcState
from ocs2_msgs.msg import MpcTargetTrajectories
from ocs2_msgs.srv import Reset
from rclpy.node import Node

from interactive_reference import generate_strokes


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/Clearpath_mobile_manipulator/mujoco/scene.xml"
REFERENCE = ROOT / "src/Clearpath_mobile_manipulator/mujoco/vr_reference.csv"
ARM_JOINTS = tuple(f"ur5e_{name}_joint" for name in (
    "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"))
WHEEL_RADIUS = 0.1651
MECANUM_HALF_SPAN = 0.256 + 0.2854
TIME_SCALE = 3.0
APPROACH_SECONDS = 10.0
REFERENCE_STEP = 0.1
BASE_HEIGHT = 0.13228
BASE_START_X = -0.35
BASE_START_Y = -0.2
DRAW_BASE_X = 0.30
PEN_TO_BASE_Y = 0.25
PARK_Y = -2.0  # Right of the wall: its edge is y=-1.5 m.
HOME_ARM = np.array((-0.4640732, -0.7754991, 0.2277219,
                     -1.9449161, -1.9782423, -1.46))


def build_reference(draw_rows: list[dict], start_tip: np.ndarray,
                    time_scale: float = TIME_SCALE) -> list[dict]:
    """Move the raised pen toward the board before tracing the letters."""
    first = draw_rows[0]
    drawing_start = np.array([float(first[key]) for key in ("x_m", "y_m", "z_m")])
    # Keep the arm near its comfortable lateral reach as the pen traverses the board.
    # A time-domain moving average keeps small hand tremors out of the base path.
    draw_times = np.asarray([float(row["time_s"]) * time_scale for row in draw_rows])
    draw_y = np.asarray([float(row["y_m"]) for row in draw_rows])
    base_y = np.empty(len(draw_rows))
    for i, instant in enumerate(draw_times):
        left = max(0, int(np.searchsorted(draw_times, instant - 1.0)))
        right = min(len(draw_rows), int(np.searchsorted(draw_times, instant + 1.0, side="right")))
        base_y[i] = float(np.mean(draw_y[left:right])) - PEN_TO_BASE_Y
    base_y = np.clip(base_y, -0.80, 0.15)
    count = round(APPROACH_SECONDS / REFERENCE_STEP)
    rows = []
    for index in range(count + 1):
        u = index / count
        blend = u**3 * (10 - 15 * u + 6 * u**2)
        point = start_tip + (drawing_start - start_tip) * blend
        rows.append({"time_s": index * REFERENCE_STEP, "x_m": point[0],
                     "y_m": point[1], "z_m": point[2],
                     "base_x_m": BASE_START_X + (DRAW_BASE_X - BASE_START_X) * blend,
                     "base_y_m": BASE_START_Y + (base_y[0] - BASE_START_Y) * blend,
                     "pen_down": 0, "stroke": "approach"})
    for i, row in enumerate(draw_rows[1:], start=1):
        shifted = row.copy()
        shifted["time_s"] = APPROACH_SECONDS + time_scale * float(row["time_s"])
        shifted["base_x_m"] = DRAW_BASE_X
        shifted["base_y_m"] = float(base_y[i])
        rows.append(shifted)
    return rows


class Bridge(Node):
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, reference: list[dict]):
        super().__init__("husky_ur5e_mujoco_bridge")
        self.model = model
        self.data = data
        self.reference = reference
        self.reference_times = [float(row["time_s"]) for row in reference]
        initial_quat = data.body("pen").xquat
        self.target_quat_xyzw = [float(initial_quat[i]) for i in (1, 2, 3, 0)]
        self.arm_qpos = [model.joint(name).qposadr[0] for name in ARM_JOINTS]
        self.arm_dof = [model.joint(name).dofadr[0] for name in ARM_JOINTS]
        self.base_mass = float(model.body_subtreemass[model.body("base_link").id])
        self.policy_input = np.zeros(9)
        self.policy_time = -math.inf
        self.policy_times = np.empty(0)
        self.policy_inputs = np.empty((0, 9))
        self.applied_input = np.zeros(9)
        self.observation_pub = self.create_publisher(
            MpcObservation, "/mobile_manipulator_mpc_observation", 1)
        self.target_pub = self.create_publisher(
            MpcTargetTrajectories, "/mobile_manipulator_mpc_target", 1)
        self.create_subscription(MpcFlattenedController,
                                 "/mobile_manipulator_mpc_policy", self.on_policy, 1)
        self.reset_client = self.create_client(Reset, "/mobile_manipulator_mpc_reset")
        self.pen_site = model.site("pen_tip").id

    def set_reference(self, reference: list[dict]) -> None:
        self.reference = reference
        self.reference_times = [float(row["time_s"]) for row in reference]
        self.policy_time = -math.inf
        self.policy_times = np.empty(0)
        self.policy_inputs = np.empty((0, 9))
        self.applied_input[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.qfrc_applied[:6] = 0.0

    def on_policy(self, message: MpcFlattenedController) -> None:
        if not message.input_trajectory or not message.time_trajectory:
            return
        commands = np.asarray([item.value for item in message.input_trajectory], dtype=float)
        times = np.asarray(message.time_trajectory, dtype=float)
        if (commands.ndim == 2 and commands.shape[1] == 9
                and len(times) == len(commands) and np.isfinite(commands).all()
                and np.isfinite(times).all() and np.all(np.diff(times) >= 0)
                and times[0] <= self.data.time + 2.0):
            self.policy_times = times
            self.policy_inputs = commands
            self.policy_time = self.data.time

    def state(self) -> np.ndarray:
        base = self.data.body("base_link")
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, base.xquat)
        yaw = math.atan2(rotation[3], rotation[0])
        return np.asarray([base.xpos[0], base.xpos[1], yaw,
                           *(self.data.qpos[index] for index in self.arm_qpos)])

    def observation(self) -> MpcObservation:
        message = MpcObservation()
        message.time = float(self.data.time)
        message.state = MpcState(value=self.state().astype(np.float32).tolist())
        message.input = MpcInput(value=self.applied_input.astype(np.float32).tolist())
        message.mode = 0
        return message

    def target(self, now: float, horizon: float = 1.5) -> MpcTargetTrajectories:
        result = MpcTargetTrajectories()
        first = max(0, bisect_right(self.reference_times, now) - 1)
        last = min(len(self.reference), max(first + 2,
                   bisect_right(self.reference_times, now + horizon) + 1))
        # Use time to select the horizon: panel paths and the VR demo have
        # different sampling intervals. Include both ends without flooding ROS.
        indices = np.unique(np.linspace(first, last - 1, min(7, last - first), dtype=int))
        for index in indices:
            row = self.reference[int(index)]
            result.time_trajectory.append(max(now, float(row["time_s"])))
            result.state_trajectory.append(MpcState(value=[
                float(row["x_m"]), float(row["y_m"]), float(row["z_m"]) - BASE_HEIGHT,
                float(row["base_x_m"]), float(row["base_y_m"]), 0.0,
                *self.target_quat_xyzw]))
            result.input_trajectory.append(MpcInput(value=[0.0] * 9))
        if len(result.time_trajectory) == 1:
            result.time_trajectory.append(now + 0.1)
            result.state_trajectory.append(result.state_trajectory[0])
            result.input_trajectory.append(MpcInput(value=[0.0] * 9))
        return result

    def reset_mpc(self, stop: threading.Event | None = None) -> bool:
        service_deadline = time.monotonic() + 180.0
        while not self.reset_client.wait_for_service(timeout_sec=0.5):
            if stop is not None and stop.is_set():
                return False
            if time.monotonic() >= service_deadline:
                raise RuntimeError("OCS2 reset service is unavailable")
        for attempt in range(3):
            request = Reset.Request()
            request.reset = True
            request.target_trajectories = self.target(0.0)
            future = self.reset_client.call_async(request)
            reply_deadline = time.monotonic() + 20.0
            while not future.done() and time.monotonic() < reply_deadline:
                if stop is not None and stop.is_set():
                    return False
                rclpy.spin_once(self, timeout_sec=0.2)
            if future.done() and future.result() is not None and future.result().done:
                return True
            self.get_logger().warning(f"OCS2 reset attempt {attempt + 1}/3 did not complete")
        raise RuntimeError("OCS2 reset failed after three attempts")

    def apply_policy(self, dt: float) -> None:
        if self.data.time - self.policy_time > 0.25:
            self.data.ctrl[:] = 0.0
            self.data.qfrc_applied[:6] = 0.0
            self.applied_input[:] = 0.0
            return
        right = int(np.searchsorted(self.policy_times, self.data.time, side="right"))
        if right == 0:
            self.policy_input = self.policy_inputs[0]
        elif right >= len(self.policy_times):
            self.policy_input = self.policy_inputs[-1]
        else:
            left = right - 1
            interval = self.policy_times[right] - self.policy_times[left]
            blend = (self.data.time - self.policy_times[left]) / interval if interval > 0 else 0.0
            self.policy_input = self.policy_inputs[left] * (1.0 - blend) + self.policy_inputs[right] * blend
        lower = [-0.2, -0.2, -0.5, *([-0.5] * 6)]
        upper = [0.2, 0.2, 0.5, *([0.5] * 6)]
        command = np.clip(self.policy_input, lower, upper)
        base_x = float(self.data.body("base_link").xpos[0])
        base_yaw = self.state()[2]
        cosine, sine = math.cos(base_yaw), math.sin(base_yaw)
        world_forward = cosine * command[0] - sine * command[1]
        if base_x > 0.52 and world_forward > -2.0 * (base_x - 0.52):
            correction = world_forward + 2.0 * (base_x - 0.52)
            command[0] -= cosine * correction
            command[1] += sine * correction
        tip_x = self.data.site("pen_tip").xpos[0]
        if tip_x > 1.07:
            jacobian = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, self.data, jacobian, None, self.pen_site)
            gradient = np.asarray([
                jacobian[0, 0] * cosine + jacobian[0, 1] * sine,
                -jacobian[0, 0] * sine + jacobian[0, 1] * cosine,
                jacobian[0, 5],
                *(jacobian[0, index] for index in self.arm_dof)])
            normal_speed = float(gradient @ command)
            maximum_normal_speed = -5.0 * (tip_x - 1.075)
            if normal_speed > maximum_normal_speed:
                command -= gradient * (normal_speed - maximum_normal_speed) / max(float(gradient @ gradient), 1e-8)
            command = np.clip(command, lower, upper)
        self.applied_input = command
        forward, lateral, turn = command[:3]
        # Idealized planar velocity servo; the original Husky wheels have no rollers.
        desired_world = np.array((cosine * forward - sine * lateral,
                                  sine * forward + cosine * lateral))
        velocity_error = desired_world - self.data.qvel[:2]
        self.data.qfrc_applied[:2] = np.clip(45.0 * self.base_mass * velocity_error, -700, 700)
        self.data.qfrc_applied[5] = np.clip(250.0 * (turn - self.data.qvel[5]), -120, 120)
        # Mecanum X roller pattern; wheel rotation is synchronized with the virtual base.
        span = MECANUM_HALF_SPAN * turn
        self.data.ctrl[:4] = [(forward - lateral - span) / WHEEL_RADIUS,
                              (forward + lateral - span) / WHEEL_RADIUS,
                              (forward + lateral + span) / WHEEL_RADIUS,
                              (forward - lateral + span) / WHEEL_RADIUS]
        self.data.ctrl[4:] = command[3:]


def reset_home(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    mujoco.mj_resetData(model, data)
    data.qpos[:3] = (BASE_START_X, BASE_START_Y, BASE_HEIGHT)
    data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    for joint, position in zip(ARM_JOINTS, HOME_ARM):
        data.joint(joint).qpos[0] = position
    mujoco.mj_forward(model, data)


def drive_to_pose(model: mujoco.MjModel, data: mujoco.MjData, bridge: Bridge,
                  viewer, fps: float, stop: threading.Event,
                  target: np.ndarray, stow_arm: bool) -> bool:
    """Drive the ideal omni base without resetting the physical MuJoCo state."""
    last_marker = np.array((*data.body("base_link").xpos[:2], 0.012))
    phase_start = data.time
    wall_start = time.monotonic()
    next_frame = data.time
    last_marker_time = data.time
    while data.time - phase_start < 20.0:
        if stop.is_set():
            return False
        base = data.body("base_link")
        position_error = target - base.xpos[:2]
        arm_error = HOME_ARM - data.qpos[bridge.arm_qpos]
        yaw = bridge.state()[2]
        if (np.linalg.norm(position_error) < 0.008 and abs(yaw) < 0.025
                and (not stow_arm or np.max(np.abs(arm_error)) < 0.03)):
            break
        desired_world = 1.6 * position_error
        speed = float(np.linalg.norm(desired_world))
        if speed > 0.18:
            desired_world *= 0.18 / speed
        turn = float(np.clip(-1.5 * yaw, -0.25, 0.25))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        forward = cosine * desired_world[0] + sine * desired_world[1]
        lateral = -sine * desired_world[0] + cosine * desired_world[1]
        span = MECANUM_HALF_SPAN * turn
        data.ctrl[:4] = ((forward - lateral - span) / WHEEL_RADIUS,
                         (forward + lateral - span) / WHEEL_RADIUS,
                         (forward + lateral + span) / WHEEL_RADIUS,
                         (forward - lateral + span) / WHEEL_RADIUS)
        data.ctrl[4:] = np.clip(2.0 * arm_error, -0.4, 0.4) if stow_arm else 0.0
        data.qfrc_applied[:2] = np.clip(
            45.0 * bridge.base_mass * (desired_world - data.qvel[:2]), -700, 700)
        data.qfrc_applied[5] = np.clip(250.0 * (turn - data.qvel[5]), -120, 120)
        data.qfrc_applied[bridge.arm_dof] = data.qfrc_bias[bridge.arm_dof]
        mujoco.mj_step(model, data)
        if data.site("pen_tip").xpos[0] > 1.115:
            raise RuntimeError("Pen approached the wall during travel")
        if viewer is not None and data.time - last_marker_time >= 0.35:
            point = np.array((*data.body("base_link").xpos[:2], 0.012))
            with viewer.lock():
                if viewer.user_scn.ngeom < viewer.user_scn.maxgeom:
                    geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
                    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                        np.array((0.005, 0, 0)), point,
                                        np.eye(3).ravel(),
                                        np.array((0.08, 0.5, 0.75, 0.8), dtype=np.float32))
                    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                        0.005, last_marker, point)
                    viewer.user_scn.ngeom += 1
            last_marker = point
            last_marker_time = data.time
        if viewer is not None and data.time >= next_frame:
            viewer.sync()
            next_frame = data.time + 1.0 / fps
        remaining = wall_start + (data.time - phase_start) - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
    else:
        raise RuntimeError(f"Travel timed out at base {data.body('base_link').xpos[:2]}")
    data.ctrl[:] = 0.0
    data.qfrc_applied[:6] = 0.0
    return True


def retreat_to_standby(model: mujoco.MjModel, data: mujoco.MjData, bridge: Bridge,
                       viewer, fps: float, stop: threading.Event) -> bool:
    """Leave the front of the board, then park fully beyond its right edge."""
    current_y = float(data.body("base_link").xpos[1])
    for target, stow in ((np.array((BASE_START_X, current_y)), False),
                         (np.array((BASE_START_X, PARK_Y)), True)):
        if not drive_to_pose(model, data, bridge, viewer, fps, stop, target, stow):
            return False
    if data.body("base_link").xpos[1] > -1.85:
        raise RuntimeError("Base did not clear the right edge of the board")
    return True


def panel_status(message: str) -> None:
    print("PANEL_STATUS " + json.dumps({"status": message}, ensure_ascii=False), flush=True)


def read_panel_commands(commands: queue.Queue, stop: threading.Event) -> None:
    for line in sys.stdin:
        try:
            message = json.loads(line)
        except ValueError:
            continue
        if message.get("type") == "quit":
            stop.set()
            return
        commands.put(message)
    stop.set()


def run_cycle(model: mujoco.MjModel, data: mujoco.MjData, bridge: Bridge,
              viewer, reference: list[dict], args: argparse.Namespace,
              stop: threading.Event | None = None) -> str:
    if viewer is not None:
        with viewer.lock():
            viewer.user_scn.ngeom = 0
        viewer.sync()
    rows = []
    trace = []
    last_ink_point = None
    last_ink_stroke = None
    last_ink_time = -math.inf
    last_base_point = np.array((BASE_START_X, BASE_START_Y, 0.012))
    last_base_time = 0.0
    next_viewer_time = 0.0
    if not bridge.reset_mpc(stop):
        return "Đã dừng trước khi vẽ."
    bridge.target_pub.publish(bridge.target(data.time))
    deadline = time.monotonic() + 300
    while (not np.isfinite(bridge.policy_time) and time.monotonic() < deadline
           and (stop is None or not stop.is_set())):
        bridge.observation_pub.publish(bridge.observation())
        rclpy.spin_once(bridge, timeout_sec=0.1)
    if stop is not None and stop.is_set():
        return "Đã dừng trước khi vẽ."
    if not np.isfinite(bridge.policy_time):
        raise RuntimeError("No OCS2 policy arrived within 300 s")
    stop_reason = None
    duration = float(reference[-1]["time_s"])
    if args.seconds is not None:
        duration = min(duration, args.seconds)
    simulation_steps = math.ceil(duration / model.opt.timestep)
    start_wall_time = time.monotonic()
    timings = {"ros": 0.0, "policy": 0.0, "physics": 0.0, "viewer": 0.0}
    late_steps = 0
    worst_lag = 0.0
    rendered_frames = 0
    for step in range(simulation_steps):
        if stop is not None and stop.is_set():
            stop_reason = "user_exit"
            break
        mark = time.monotonic() if args.profile else 0.0
        if step % 10 == 0:
            rclpy.spin_once(bridge, timeout_sec=0.0)
            bridge.observation_pub.publish(bridge.observation())
        if step % 50 == 0:
            bridge.target_pub.publish(bridge.target(data.time))
        if args.profile:
            timings["ros"] += time.monotonic() - mark
            mark = time.monotonic()
        bridge.apply_policy(model.opt.timestep)
        if args.profile:
            timings["policy"] += time.monotonic() - mark
            mark = time.monotonic()
        data.qfrc_applied[bridge.arm_dof] = data.qfrc_bias[bridge.arm_dof]
        mujoco.mj_step(model, data)
        if args.profile:
            timings["physics"] += time.monotonic() - mark
            mark = time.monotonic()
        if data.sensor("pen_normal_force").data[0] > 2.0 or data.site("pen_tip").xpos[0] > 1.115:
            stop_reason = "pen reached wall or exceeded 2 N during dry-run"
            data.ctrl[:4] = 0.0
            break
        if step % 10 == 0:
            pos = data.site("pen_tip").xpos
            index = max(0, bisect_right(bridge.reference_times, data.time) - 1)
            desired = reference[index]
            error = math.hypot(pos[1] - float(desired["y_m"]),
                               pos[2] - float(desired["z_m"]))
            pen_down = int(desired["pen_down"])
            rows.append((data.time, *data.body("base_link").xpos[:2], *pos,
                         float(data.sensor("pen_normal_force").data[0]),
                         *bridge.applied_input[:2], error, pen_down,
                         desired["stroke"], bridge.state()[2],
                         bridge.applied_input[2]))
            if pen_down and error < 0.04 and abs(pos[0] - 1.075) < 0.04:
                trace.append((1.124, pos[1], pos[2]))
                if viewer is not None and data.time - last_ink_time >= 0.14:
                    point = np.array((1.124, pos[1], pos[2]))
                    with viewer.lock():
                        if viewer.user_scn.ngeom < viewer.user_scn.maxgeom:
                            geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
                            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                                                np.array((0.004, 0, 0)), point,
                                                np.eye(3).ravel(),
                                                np.array((0.85, 0.03, 0.04, 1), dtype=np.float32))
                            if last_ink_stroke == desired["stroke"] and last_ink_point is not None:
                                mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                                    0.004, last_ink_point, point)
                            viewer.user_scn.ngeom += 1
                    last_ink_point = point
                    last_ink_stroke = desired["stroke"]
                    last_ink_time = data.time
        if viewer is not None and data.time - last_base_time >= 0.35:
            base_point = np.array((*data.body("base_link").xpos[:2], 0.012))
            with viewer.lock():
                if viewer.user_scn.ngeom < viewer.user_scn.maxgeom:
                    geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
                    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                        np.array((0.005, 0, 0)), base_point,
                                        np.eye(3).ravel(),
                                        np.array((0.05, 0.42, 0.72, 0.85), dtype=np.float32))
                    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE,
                                        0.005, last_base_point, base_point)
                    viewer.user_scn.ngeom += 1
            last_base_point = base_point
            last_base_time = data.time
        if viewer is not None and data.time >= next_viewer_time:
            viewer.sync()
            rendered_frames += 1
            next_viewer_time = data.time + 1.0 / args.viewer_fps
        if args.profile:
            timings["viewer"] += time.monotonic() - mark
        remaining = start_wall_time + data.time - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        elif args.profile:
            late_steps += 1
            worst_lag = max(worst_lag, -remaining)
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with args.log.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(("time_s", "base_x_m", "base_y_m", "pen_x_m", "pen_y_m", "pen_z_m",
                         "pen_force_n", "vx_m_s", "vy_m_s", "yz_error_m", "pen_down",
                         "stroke", "base_yaw_rad", "omega_rad_s"))
        writer.writerows(rows)
    errors = [row[9] for row in rows if row[10]]
    rms = math.sqrt(sum(e * e for e in errors) / len(errors)) if errors else math.nan
    rms_label = f"{rms:.3f} m" if errors else "n/a (approach only)"
    distance = math.hypot(data.body("base_link").xpos[0] - BASE_START_X,
                          data.body("base_link").xpos[1] - BASE_START_Y)
    summary = (f"Saved {len(rows)} observations to {args.log}; "
               f"base distance={distance:.3f} m; draw RMS={rms_label}; "
               f"ink points={len(trace)}")
    print(summary, flush=True)
    if args.profile:
        wall_elapsed = time.monotonic() - start_wall_time
        print(f"Timing: sim={data.time:.2f}s wall={wall_elapsed:.2f}s "
              f"frames={rendered_frames} late_steps={late_steps} "
              f"worst_lag={worst_lag:.3f}s "
              + " ".join(f"{key}={value:.2f}s" for key, value in timings.items()))
    if stop_reason and stop_reason != "user_exit":
        raise RuntimeError(f"Stopped safely at t={data.time:.2f}s: {stop_reason}")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=None)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--interactive", action="store_true", help="Keep MuJoCo open for repeated panel drawings")
    parser.add_argument("--demo-vr", action="store_true", help="Run the original VR reference")
    parser.add_argument("--strokes-json", type=Path, help="Replay strokes submitted from the drawing panel")
    parser.add_argument("--viewer-fps", type=float, default=30.0)
    parser.add_argument("--profile", action="store_true", help="Print runtime timing for the simulation loop")
    parser.add_argument("--log", type=Path, default=ROOT / "vr_ocs2_run.csv")
    args = parser.parse_args()
    if not 1.0 <= args.viewer_fps <= 60.0:
        parser.error("--viewer-fps must be between 1 and 60")
    if args.interactive and args.strokes_json:
        parser.error("--interactive reads strokes from stdin; omit --strokes-json")

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    reset_home(model, data)
    rclpy.init()
    bridge = Bridge(model, data, [])
    viewer = None if args.headless else mjviewer.launch_passive(model, data)
    if viewer is not None:
        viewer.sync()
    try:
        if args.interactive:
            commands: queue.Queue = queue.Queue()
            stop = threading.Event()
            threading.Thread(target=read_panel_commands, args=(commands, stop), daemon=True).start()
            panel_status("MuJoCo đã mở. Vẽ trên panel và bấm Gửi; robot đang chờ lệnh.")
            while not stop.is_set():
                try:
                    command = commands.get(timeout=0.1)
                except queue.Empty:
                    if viewer is not None:
                        viewer.sync()
                    continue
                if command.get("type") != "draw":
                    continue
                number = command.get("id", "?")
                try:
                    draw_reference = generate_strokes(command["strokes"])
                    if np.linalg.norm(data.body("base_link").xpos[:2] -
                                      np.array((BASE_START_X, BASE_START_Y))) > 0.04:
                        panel_status(f"Robot đang từ bên phải bảng quay lại vị trí bắt đầu cho bản #{number}.")
                        if not drive_to_pose(model, data, bridge, viewer, args.viewer_fps,
                                             stop, np.array((BASE_START_X, BASE_START_Y)), True):
                            break
                    # Reset the MPC clock only; keep the physical pose reached by retreat.
                    data.time = 0.0
                    mujoco.mj_forward(model, data)
                    reference = build_reference(draw_reference, data.site("pen_tip").xpos.copy(), 1.0)
                    bridge.set_reference(reference)
                    args.log = Path(command["log"])
                    panel_status(f"Robot đang vẽ bản #{number}; panel vẫn nhận bản tiếp theo.")
                    summary = run_cycle(model, data, bridge, viewer, reference, args, stop)
                    if not stop.is_set():
                        panel_status(f"Bản #{number} đã vẽ xong. Robot đang lùi rồi đi sang phải ra khỏi bảng.")
                        if retreat_to_standby(model, data, bridge, viewer, args.viewer_fps, stop):
                            position = data.body("base_link").xpos
                            panel_status(f"Bản #{number} hoàn thành. {summary} "
                                         f"Robot đỗ ngoài mép phải bảng tại "
                                         f"({position[0]:.2f}, {position[1]:.2f}) m "
                                         "và đang chờ nét tiếp theo trong MuJoCo.")
                except Exception as error:
                    panel_status(f"Bản #{number} lỗi: {error}. MuJoCo vẫn mở để nhận bản khác.")
            panel_status("Đã nhận lệnh Kết thúc; đang đóng MuJoCo.")
        else:
            if args.strokes_json:
                payload = json.loads(args.strokes_json.read_text(encoding="utf-8"))
                draw_reference = generate_strokes(payload["strokes"])
                scale = 1.0
            else:
                with REFERENCE.open(newline="") as file:
                    draw_reference = list(csv.DictReader(file))
                scale = TIME_SCALE
            reference = build_reference(draw_reference, data.site("pen_tip").xpos.copy(), scale)
            bridge.set_reference(reference)
            run_cycle(model, data, bridge, viewer, reference, args)
    finally:
        if viewer is not None:
            viewer.close()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except KeyboardInterrupt:
        exit_code = 130
    except Exception:
        traceback.print_exc()
        exit_code = 1
    if "--headless" not in sys.argv:
        # GLFW and rclpy can crash during Python's native atexit teardown on
        # this ROS/GLX stack, after both viewer.close() and rclpy.shutdown().
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)
    if exit_code:
        sys.exit(exit_code)
