#!/usr/bin/env python3
"""Record and summarize iSweep+TEB versus GlobalPlanner+TEB experiments.

Live mode subscribes to the navigation topics and appends one CSV row for every
goal. Offline mode reads one or more rosbag files and writes both per-run and
per-mode summary CSV files using the exact same metric implementation.
"""

import argparse
import csv
import math
import os
import statistics
import sys
import threading
from collections import defaultdict
from datetime import datetime

import rospy
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


ISWEEP_MODE = "isweep_teb"
CONTROL_MODE = "global_teb"
AUTO_MODE = "auto"

GOAL_TOPIC = "/move_base_simple/goal"
LOCALIZATION_TOPIC = "/localization"
CMD_TOPIC = "/cmd_vel"
SCAN_TOPIC = "/scan"
ISWEEP_PATH_TOPIC = "/isweep_planner/trajectory"
CONTROL_PATH_TOPIC = "/move_base/GlobalPlanner/plan"
ISWEEP_TIME_TOPIC = "/isweep_planner/planning_time"
ISWEEP_STATS_TOPIC = "/isweep_planner/planning_stats"
ISWEEP_STATUS_TOPIC = "/isweep_teb_local_planner/status"
CONTROL_STATUS_TOPIC = "/move_base/status"

PLANNING_STAT_FIELDS = [
    "isweep_total_solve_time_s",
    "isweep_search_time_s",
    "isweep_optimization_time_s",
    "isweep_min_clearance_m",
    "isweep_coarse_path_points",
    "isweep_support_points",
    "isweep_local_obstacle_points",
    "isweep_optimizer_iterations",
    "isweep_preprocess_time_s",
    "isweep_segment_solve_time_s",
    "isweep_full_feasibility_time_s",
    "isweep_tail_refine_time_s",
    "isweep_recovery_time_s",
]

CSV_FIELDS = [
    "run_id",
    "source",
    "mode",
    "goal_time_s",
    "goal_x",
    "goal_y",
    "goal_yaw_rad",
    "success",
    "success_source",
    "failure_reason",
    "planning_latency_s",
    "planner_reported_time_s",
    "global_path_length_m",
    "global_path_poses",
    "global_path_messages",
    "replan_count",
    "navigation_time_s",
    "executed_distance_m",
    "terminal_position_error_m",
    "terminal_yaw_error_rad",
    "minimum_lidar_range_m",
    "cmd_samples",
    "mean_abs_linear_speed_mps",
    "max_abs_linear_speed_mps",
    "mean_abs_angular_speed_radps",
    "max_abs_angular_speed_radps",
    "rms_linear_acceleration_mps2",
    "max_abs_linear_acceleration_mps2",
    "rms_angular_acceleration_radps2",
    "max_abs_angular_acceleration_radps2",
    "rms_linear_jerk_mps3",
    "max_abs_linear_jerk_mps3",
    "rms_angular_jerk_radps3",
    "max_abs_angular_jerk_radps3",
    "stop_count",
    "direction_switch_count",
    "blocked_status_count",
] + PLANNING_STAT_FIELDS

SUMMARY_FIELDS = [
    "mode",
    "runs",
    "successes",
    "success_rate",
    "planning_latency_mean_s",
    "planning_latency_median_s",
    "planning_latency_std_s",
    "global_path_length_mean_m",
    "global_path_length_std_m",
    "navigation_time_mean_s",
    "navigation_time_std_s",
    "executed_distance_mean_m",
    "executed_distance_std_m",
    "minimum_lidar_range_mean_m",
    "rms_linear_acceleration_mean_mps2",
    "rms_angular_acceleration_mean_radps2",
    "rms_linear_jerk_mean_mps3",
    "rms_angular_jerk_mean_radps3",
]


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def extract_pose(message):
    """Return (x, y, yaw) from PoseStamped, Odometry, or similar messages."""
    pose = message.pose
    while hasattr(pose, "pose"):
        pose = pose.pose
    return (
        float(pose.position.x),
        float(pose.position.y),
        quaternion_yaw(pose.orientation),
    )


def path_length(path_message):
    total = 0.0
    previous = None
    for stamped_pose in path_message.poses:
        current = stamped_pose.pose.position
        if previous is not None:
            total += math.hypot(current.x - previous.x, current.y - previous.y)
        previous = current
    return total


def finite_float(value):
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def rms(sum_of_squares, count):
    if count <= 0:
        return None
    return math.sqrt(sum_of_squares / float(count))


def csv_value(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return "{:.9g}".format(value)
    return value


class RunState:
    def __init__(self, run_id, source, requested_mode, goal_message, goal_time,
                 position_tolerance, yaw_tolerance, dwell_time, max_cmd_gap):
        self.run_id = run_id
        self.source = source
        self.requested_mode = requested_mode
        self.detected_mode = "" if requested_mode == AUTO_MODE else requested_mode
        self.goal_time = float(goal_time)
        self.goal_x, self.goal_y, self.goal_yaw = extract_pose(goal_message)
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.dwell_time = dwell_time
        self.max_cmd_gap = max_cmd_gap

        self.finished = False
        self.success = False
        self.success_source = ""
        self.failure_reason = ""
        self.finish_time = None

        self.planning_latency = None
        self.planner_reported_time = None
        self.global_path_length = None
        self.global_path_poses = 0
        self.global_path_messages = 0
        self.planning_stats = [None] * len(PLANNING_STAT_FIELDS)

        self.executed_distance = 0.0
        self.previous_pose = None
        self.latest_position_error = None
        self.latest_yaw_error = None
        self.in_tolerance_since = None
        self.minimum_lidar_range = None

        self.cmd_samples = 0
        self.sum_abs_linear = 0.0
        self.sum_abs_angular = 0.0
        self.max_abs_linear = 0.0
        self.max_abs_angular = 0.0
        self.previous_cmd = None
        self.previous_acceleration = None
        self.linear_accel_sq_sum = 0.0
        self.angular_accel_sq_sum = 0.0
        self.accel_samples = 0
        self.max_abs_linear_accel = 0.0
        self.max_abs_angular_accel = 0.0
        self.linear_jerk_sq_sum = 0.0
        self.angular_jerk_sq_sum = 0.0
        self.jerk_samples = 0
        self.max_abs_linear_jerk = 0.0
        self.max_abs_angular_jerk = 0.0
        self.was_moving = False
        self.last_direction = 0
        self.stop_count = 0
        self.direction_switch_count = 0
        self.blocked_status_count = 0

    @property
    def mode(self):
        return self.detected_mode or AUTO_MODE

    def accepts_mode(self, mode):
        if self.requested_mode != AUTO_MODE and mode != self.requested_mode:
            return False
        if self.detected_mode and self.detected_mode != mode:
            return False
        if not self.detected_mode:
            self.detected_mode = mode
        return True

    def observe_path(self, mode, message, timestamp):
        if self.finished or timestamp < self.goal_time or not self.accepts_mode(mode):
            return
        self.global_path_messages += 1
        if self.planning_latency is None and len(message.poses) > 0:
            self.planning_latency = max(0.0, timestamp - self.goal_time)
            self.global_path_length = path_length(message)
            self.global_path_poses = len(message.poses)

    def observe_planner_time(self, value, timestamp):
        if self.finished or timestamp < self.goal_time:
            return
        if not self.accepts_mode(ISWEEP_MODE):
            return
        if self.planner_reported_time is None:
            self.planner_reported_time = finite_float(value)

    def observe_planning_stats(self, values, timestamp):
        if self.finished or timestamp < self.goal_time:
            return
        if not self.accepts_mode(ISWEEP_MODE):
            return
        for index in range(min(len(values), len(self.planning_stats))):
            self.planning_stats[index] = finite_float(values[index])

    def observe_pose(self, message, timestamp):
        if self.finished or timestamp < self.goal_time:
            return
        x, y, yaw = extract_pose(message)
        if self.previous_pose is not None:
            self.executed_distance += math.hypot(
                x - self.previous_pose[0], y - self.previous_pose[1]
            )
        self.previous_pose = (x, y, yaw)
        self.latest_position_error = math.hypot(x - self.goal_x, y - self.goal_y)
        self.latest_yaw_error = abs(normalize_angle(yaw - self.goal_yaw))

        inside = (
            self.latest_position_error <= self.position_tolerance
            and self.latest_yaw_error <= self.yaw_tolerance
        )
        if not inside:
            self.in_tolerance_since = None
            return
        if self.in_tolerance_since is None:
            self.in_tolerance_since = timestamp
        if timestamp - self.in_tolerance_since >= self.dwell_time:
            self.finish(True, timestamp, "pose_tolerance", "")

    def observe_scan(self, message, timestamp):
        if self.finished or timestamp < self.goal_time:
            return
        valid = [
            value for value in message.ranges
            if math.isfinite(value)
            and value >= message.range_min
            and value <= message.range_max
        ]
        if not valid:
            return
        current_minimum = min(valid)
        if self.minimum_lidar_range is None:
            self.minimum_lidar_range = current_minimum
        else:
            self.minimum_lidar_range = min(self.minimum_lidar_range, current_minimum)

    def observe_cmd(self, message, timestamp):
        if self.finished or timestamp < self.goal_time:
            return
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        self.cmd_samples += 1
        self.sum_abs_linear += abs(linear)
        self.sum_abs_angular += abs(angular)
        self.max_abs_linear = max(self.max_abs_linear, abs(linear))
        self.max_abs_angular = max(self.max_abs_angular, abs(angular))

        moving = abs(linear) >= 0.02 or abs(angular) >= 0.02
        if self.was_moving and not moving:
            self.stop_count += 1
        self.was_moving = moving

        direction = 1 if linear >= 0.02 else (-1 if linear <= -0.02 else 0)
        if direction and self.last_direction and direction != self.last_direction:
            self.direction_switch_count += 1
        if direction:
            self.last_direction = direction

        if self.previous_cmd is None:
            self.previous_cmd = (timestamp, linear, angular)
            return
        dt = timestamp - self.previous_cmd[0]
        if dt <= 1e-6 or dt > self.max_cmd_gap:
            self.previous_cmd = (timestamp, linear, angular)
            self.previous_acceleration = None
            return

        linear_accel = (linear - self.previous_cmd[1]) / dt
        angular_accel = (angular - self.previous_cmd[2]) / dt
        self.linear_accel_sq_sum += linear_accel * linear_accel
        self.angular_accel_sq_sum += angular_accel * angular_accel
        self.accel_samples += 1
        self.max_abs_linear_accel = max(self.max_abs_linear_accel, abs(linear_accel))
        self.max_abs_angular_accel = max(self.max_abs_angular_accel, abs(angular_accel))

        if self.previous_acceleration is not None:
            accel_dt = timestamp - self.previous_acceleration[0]
            if 1e-6 < accel_dt <= self.max_cmd_gap:
                linear_jerk = (linear_accel - self.previous_acceleration[1]) / accel_dt
                angular_jerk = (angular_accel - self.previous_acceleration[2]) / accel_dt
                self.linear_jerk_sq_sum += linear_jerk * linear_jerk
                self.angular_jerk_sq_sum += angular_jerk * angular_jerk
                self.jerk_samples += 1
                self.max_abs_linear_jerk = max(
                    self.max_abs_linear_jerk, abs(linear_jerk)
                )
                self.max_abs_angular_jerk = max(
                    self.max_abs_angular_jerk, abs(angular_jerk)
                )
        self.previous_acceleration = (timestamp, linear_accel, angular_accel)
        self.previous_cmd = (timestamp, linear, angular)

    def observe_isweep_status(self, message, timestamp):
        if self.finished or timestamp < self.goal_time:
            return
        if not self.accepts_mode(ISWEEP_MODE):
            return
        state = message.data.split("|", 1)[0].strip().upper()
        if state == "GOAL_REACHED":
            self.finish(True, timestamp, "isweep_status", "")
        elif state == "BLOCKED":
            self.blocked_status_count += 1
        elif state == "ERROR":
            self.finish(False, timestamp, "", "isweep_error")

    def observe_control_status(self, message, timestamp):
        if self.finished or timestamp < self.goal_time:
            return
        relevant = []
        for status in message.status_list:
            stamp = status.goal_id.stamp.to_sec()
            if stamp > 0.0 and stamp + 0.5 < self.goal_time:
                continue
            relevant.append((stamp, status))
        if not relevant:
            return
        if not self.accepts_mode(CONTROL_MODE):
            return
        _, latest = max(relevant, key=lambda item: item[0])
        if latest.status == GoalStatus.SUCCEEDED:
            self.finish(True, timestamp, "move_base_status", "")
        elif latest.status in (
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST):
            self.finish(False, timestamp, "", "move_base_status_{}".format(latest.status))

    def finish(self, success, timestamp, success_source, failure_reason):
        if self.finished:
            return
        self.finished = True
        self.success = bool(success)
        self.finish_time = max(float(timestamp), self.goal_time)
        self.success_source = success_source if self.success else ""
        self.failure_reason = "" if self.success else failure_reason

    def to_row(self):
        navigation_time = None
        if self.finish_time is not None:
            navigation_time = self.finish_time - self.goal_time
        mean_linear = None
        mean_angular = None
        if self.cmd_samples:
            mean_linear = self.sum_abs_linear / self.cmd_samples
            mean_angular = self.sum_abs_angular / self.cmd_samples

        row = {
            "run_id": self.run_id,
            "source": self.source,
            "mode": self.mode,
            "goal_time_s": self.goal_time,
            "goal_x": self.goal_x,
            "goal_y": self.goal_y,
            "goal_yaw_rad": self.goal_yaw,
            "success": self.success,
            "success_source": self.success_source,
            "failure_reason": self.failure_reason,
            "planning_latency_s": self.planning_latency,
            "planner_reported_time_s": self.planner_reported_time,
            "global_path_length_m": self.global_path_length,
            "global_path_poses": self.global_path_poses,
            "global_path_messages": self.global_path_messages,
            "replan_count": max(0, self.global_path_messages - 1),
            "navigation_time_s": navigation_time,
            "executed_distance_m": self.executed_distance,
            "terminal_position_error_m": self.latest_position_error,
            "terminal_yaw_error_rad": self.latest_yaw_error,
            "minimum_lidar_range_m": self.minimum_lidar_range,
            "cmd_samples": self.cmd_samples,
            "mean_abs_linear_speed_mps": mean_linear,
            "max_abs_linear_speed_mps": self.max_abs_linear if self.cmd_samples else None,
            "mean_abs_angular_speed_radps": mean_angular,
            "max_abs_angular_speed_radps": self.max_abs_angular if self.cmd_samples else None,
            "rms_linear_acceleration_mps2": rms(
                self.linear_accel_sq_sum, self.accel_samples
            ),
            "max_abs_linear_acceleration_mps2": (
                self.max_abs_linear_accel if self.accel_samples else None
            ),
            "rms_angular_acceleration_radps2": rms(
                self.angular_accel_sq_sum, self.accel_samples
            ),
            "max_abs_angular_acceleration_radps2": (
                self.max_abs_angular_accel if self.accel_samples else None
            ),
            "rms_linear_jerk_mps3": rms(
                self.linear_jerk_sq_sum, self.jerk_samples
            ),
            "max_abs_linear_jerk_mps3": (
                self.max_abs_linear_jerk if self.jerk_samples else None
            ),
            "rms_angular_jerk_radps3": rms(
                self.angular_jerk_sq_sum, self.jerk_samples
            ),
            "max_abs_angular_jerk_radps3": (
                self.max_abs_angular_jerk if self.jerk_samples else None
            ),
            "stop_count": self.stop_count,
            "direction_switch_count": self.direction_switch_count,
            "blocked_status_count": self.blocked_status_count,
        }
        for name, value in zip(PLANNING_STAT_FIELDS, self.planning_stats):
            row[name] = value
        return row


class ExperimentProcessor:
    def __init__(self, requested_mode, source, run_prefix,
                 position_tolerance=0.10, yaw_tolerance=0.20,
                 dwell_time=1.0, max_cmd_gap=1.0, row_callback=None):
        if requested_mode not in (AUTO_MODE, ISWEEP_MODE, CONTROL_MODE):
            raise ValueError("unsupported mode: {}".format(requested_mode))
        self.requested_mode = requested_mode
        self.source = source
        self.run_prefix = run_prefix
        self.position_tolerance = position_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.dwell_time = dwell_time
        self.max_cmd_gap = max_cmd_gap
        self.row_callback = row_callback
        self.run_index = 0
        self.current = None
        self.rows = []

    def _run_id(self):
        self.run_index += 1
        return "{}_{:03d}".format(self.run_prefix, self.run_index)

    def _emit_current(self):
        if self.current is None or not self.current.finished:
            return
        row = self.current.to_row()
        self.rows.append(row)
        if self.row_callback is not None:
            self.row_callback(row)
        self.current = None

    def goal(self, message, timestamp):
        if self.current is not None:
            self.current.finish(False, timestamp, "", "superseded_by_new_goal")
            self._emit_current()
        self.current = RunState(
            self._run_id(), self.source, self.requested_mode, message, timestamp,
            self.position_tolerance, self.yaw_tolerance, self.dwell_time,
            self.max_cmd_gap,
        )

    def path(self, mode, message, timestamp):
        if self.current is not None:
            self.current.observe_path(mode, message, timestamp)

    def pose(self, message, timestamp):
        if self.current is not None:
            self.current.observe_pose(message, timestamp)
            self._emit_current()

    def scan(self, message, timestamp):
        if self.current is not None:
            self.current.observe_scan(message, timestamp)

    def cmd(self, message, timestamp):
        if self.current is not None:
            self.current.observe_cmd(message, timestamp)

    def planner_time(self, message, timestamp):
        if self.current is not None:
            self.current.observe_planner_time(message.data, timestamp)

    def planning_stats(self, message, timestamp):
        if self.current is not None:
            self.current.observe_planning_stats(message.data, timestamp)

    def isweep_status(self, message, timestamp):
        if self.current is not None:
            self.current.observe_isweep_status(message, timestamp)
            self._emit_current()

    def control_status(self, message, timestamp):
        if self.current is not None:
            self.current.observe_control_status(message, timestamp)
            self._emit_current()

    def timeout(self, timestamp, timeout_seconds):
        if (
            self.current is not None
            and timestamp - self.current.goal_time >= timeout_seconds
        ):
            self.current.finish(False, timestamp, "", "timeout")
            self._emit_current()

    def close(self, timestamp, reason="source_ended"):
        if self.current is not None:
            self.current.finish(False, timestamp, "", reason)
            self._emit_current()


class CsvSink:
    def __init__(self, path, append):
        self.path = os.path.abspath(os.path.expanduser(path))
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self.append = append
        self.lock = threading.Lock()
        if not append:
            with open(self.path, "w", newline="", encoding="utf-8") as stream:
                csv.DictWriter(stream, fieldnames=CSV_FIELDS).writeheader()

    def write(self, row):
        with self.lock:
            needs_header = (
                not os.path.exists(self.path) or os.path.getsize(self.path) == 0
            )
            with open(self.path, "a", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
                if needs_header:
                    writer.writeheader()
                writer.writerow({name: csv_value(row.get(name)) for name in CSV_FIELDS})


def numeric_values(rows, field):
    values = []
    for row in rows:
        value = row.get(field)
        if value is None or value == "":
            continue
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(parsed):
            values.append(parsed)
    return values


def mean_or_none(values):
    return statistics.mean(values) if values else None


def std_or_none(values):
    return statistics.stdev(values) if len(values) >= 2 else (0.0 if values else None)


def write_summary(rows, output_path):
    grouped = defaultdict(list)
    for row in rows:
        grouped[row.get("mode", AUTO_MODE)].append(row)

    summary_rows = []
    for mode in sorted(grouped):
        mode_rows = grouped[mode]
        successful_rows = [row for row in mode_rows if bool(row.get("success"))]
        planning = numeric_values(mode_rows, "planning_latency_s")
        path_lengths = numeric_values(mode_rows, "global_path_length_m")
        navigation = numeric_values(successful_rows, "navigation_time_s")
        executed = numeric_values(successful_rows, "executed_distance_m")
        lidar = numeric_values(mode_rows, "minimum_lidar_range_m")
        linear_accel = numeric_values(mode_rows, "rms_linear_acceleration_mps2")
        angular_accel = numeric_values(mode_rows, "rms_angular_acceleration_radps2")
        linear_jerk = numeric_values(mode_rows, "rms_linear_jerk_mps3")
        angular_jerk = numeric_values(mode_rows, "rms_angular_jerk_radps3")
        row = {
            "mode": mode,
            "runs": len(mode_rows),
            "successes": len(successful_rows),
            "success_rate": (
                float(len(successful_rows)) / len(mode_rows) if mode_rows else None
            ),
            "planning_latency_mean_s": mean_or_none(planning),
            "planning_latency_median_s": statistics.median(planning) if planning else None,
            "planning_latency_std_s": std_or_none(planning),
            "global_path_length_mean_m": mean_or_none(path_lengths),
            "global_path_length_std_m": std_or_none(path_lengths),
            "navigation_time_mean_s": mean_or_none(navigation),
            "navigation_time_std_s": std_or_none(navigation),
            "executed_distance_mean_m": mean_or_none(executed),
            "executed_distance_std_m": std_or_none(executed),
            "minimum_lidar_range_mean_m": mean_or_none(lidar),
            "rms_linear_acceleration_mean_mps2": mean_or_none(linear_accel),
            "rms_angular_acceleration_mean_radps2": mean_or_none(angular_accel),
            "rms_linear_jerk_mean_mps3": mean_or_none(linear_jerk),
            "rms_angular_jerk_mean_radps3": mean_or_none(angular_jerk),
        }
        summary_rows.append(row)

    with open(output_path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for row in summary_rows:
            writer.writerow({name: csv_value(row.get(name)) for name in SUMMARY_FIELDS})
    return summary_rows


def summary_path_for(output_path):
    stem, extension = os.path.splitext(output_path)
    return stem + "_summary" + (extension or ".csv")


def process_bags(arguments):
    try:
        import rosbag
    except ImportError as error:
        raise RuntimeError(
            "Offline rosbag mode requires the system ROS Python environment. "
            "Run this script with /usr/bin/python3 after sourcing "
            "/opt/ros/noetic/setup.bash. Original error: {}".format(error)
        )

    output = os.path.abspath(os.path.expanduser(arguments.output))
    sink = CsvSink(output, append=False)
    all_rows = []
    topics = [
        GOAL_TOPIC,
        LOCALIZATION_TOPIC,
        CMD_TOPIC,
        SCAN_TOPIC,
        ISWEEP_PATH_TOPIC,
        CONTROL_PATH_TOPIC,
        ISWEEP_TIME_TOPIC,
        ISWEEP_STATS_TOPIC,
        ISWEEP_STATUS_TOPIC,
        CONTROL_STATUS_TOPIC,
    ]

    for bag_name in arguments.bag:
        bag_path = os.path.abspath(os.path.expanduser(bag_name))
        prefix = os.path.splitext(os.path.basename(bag_path))[0]
        processor = ExperimentProcessor(
            arguments.mode,
            bag_path,
            prefix,
            arguments.position_tolerance,
            arguments.yaw_tolerance,
            arguments.dwell_time,
            arguments.max_cmd_gap,
            sink.write,
        )
        last_time = 0.0
        with rosbag.Bag(bag_path, "r") as bag:
            for topic, message, bag_time in bag.read_messages(topics=topics):
                timestamp = bag_time.to_sec()
                last_time = max(last_time, timestamp)
                if topic == GOAL_TOPIC:
                    processor.goal(message, timestamp)
                elif topic == LOCALIZATION_TOPIC:
                    processor.pose(message, timestamp)
                elif topic == CMD_TOPIC:
                    processor.cmd(message, timestamp)
                elif topic == SCAN_TOPIC:
                    processor.scan(message, timestamp)
                elif topic == ISWEEP_PATH_TOPIC:
                    processor.path(ISWEEP_MODE, message, timestamp)
                elif topic == CONTROL_PATH_TOPIC:
                    processor.path(CONTROL_MODE, message, timestamp)
                elif topic == ISWEEP_TIME_TOPIC:
                    processor.planner_time(message, timestamp)
                elif topic == ISWEEP_STATS_TOPIC:
                    processor.planning_stats(message, timestamp)
                elif topic == ISWEEP_STATUS_TOPIC:
                    processor.isweep_status(message, timestamp)
                elif topic == CONTROL_STATUS_TOPIC:
                    processor.control_status(message, timestamp)
        processor.close(last_time, "bag_ended_before_success")
        all_rows.extend(processor.rows)
        print("{}: {} run(s)".format(bag_path, len(processor.rows)))

    summary_output = summary_path_for(output)
    summary_rows = write_summary(all_rows, summary_output)
    print("Per-run CSV: {}".format(output))
    print("Summary CSV: {}".format(summary_output))
    for row in summary_rows:
        print(
            "{mode}: runs={runs} success={successes} rate={rate:.1%}".format(
                mode=row["mode"],
                runs=row["runs"],
                successes=row["successes"],
                rate=float(row["success_rate"] or 0.0),
            )
        )
    return 0


class LiveRecorder:
    def __init__(self, arguments):
        rospy.init_node("experiment_recorder")
        mode = arguments.mode or rospy.get_param("~mode", AUTO_MODE)
        output = arguments.output or rospy.get_param(
            "~output_csv", "/tmp/navigation_experiment_results.csv"
        )
        run_prefix = rospy.get_param(
            "~run_prefix", "{}_{}".format(mode, datetime.now().strftime("%Y%m%d_%H%M%S"))
        )
        self.timeout_seconds = rospy.get_param("~timeout", arguments.timeout)
        position_tolerance = rospy.get_param(
            "~position_tolerance", arguments.position_tolerance
        )
        yaw_tolerance = rospy.get_param("~yaw_tolerance", arguments.yaw_tolerance)
        dwell_time = rospy.get_param("~dwell_time", arguments.dwell_time)
        max_cmd_gap = rospy.get_param("~max_cmd_gap", arguments.max_cmd_gap)
        self.sink = CsvSink(output, append=True)
        self.lock = threading.RLock()
        self.processor = ExperimentProcessor(
            mode,
            "live",
            run_prefix,
            position_tolerance,
            yaw_tolerance,
            dwell_time,
            max_cmd_gap,
            self._write_row,
        )

        rospy.Subscriber(GOAL_TOPIC, PoseStamped, self._goal, queue_size=10)
        rospy.Subscriber(LOCALIZATION_TOPIC, Odometry, self._pose, queue_size=50)
        rospy.Subscriber(CMD_TOPIC, Twist, self._cmd, queue_size=100)
        rospy.Subscriber(SCAN_TOPIC, LaserScan, self._scan, queue_size=20)
        rospy.Subscriber(ISWEEP_PATH_TOPIC, Path, self._isweep_path, queue_size=10)
        rospy.Subscriber(CONTROL_PATH_TOPIC, Path, self._control_path, queue_size=10)
        rospy.Subscriber(ISWEEP_TIME_TOPIC, Float64, self._planner_time, queue_size=10)
        rospy.Subscriber(
            ISWEEP_STATS_TOPIC, Float64MultiArray, self._planning_stats, queue_size=10
        )
        rospy.Subscriber(ISWEEP_STATUS_TOPIC, String, self._isweep_status, queue_size=20)
        rospy.Subscriber(
            CONTROL_STATUS_TOPIC, GoalStatusArray, self._control_status, queue_size=20
        )
        self.timer = rospy.Timer(rospy.Duration(0.2), self._timer)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "experiment_recorder ready: mode=%s output=%s timeout=%.1fs",
            mode,
            self.sink.path,
            self.timeout_seconds,
        )

    @staticmethod
    def _now():
        return rospy.get_time()

    def _write_row(self, row):
        self.sink.write(row)
        rospy.loginfo(
            "experiment result saved: run=%s mode=%s success=%s planning=%s navigation=%s",
            row["run_id"],
            row["mode"],
            row["success"],
            row["planning_latency_s"],
            row["navigation_time_s"],
        )

    def _goal(self, message):
        with self.lock:
            self.processor.goal(message, self._now())
            rospy.loginfo("experiment started: %s", self.processor.current.run_id)

    def _pose(self, message):
        with self.lock:
            self.processor.pose(message, self._now())

    def _cmd(self, message):
        with self.lock:
            self.processor.cmd(message, self._now())

    def _scan(self, message):
        with self.lock:
            self.processor.scan(message, self._now())

    def _isweep_path(self, message):
        with self.lock:
            self.processor.path(ISWEEP_MODE, message, self._now())

    def _control_path(self, message):
        with self.lock:
            self.processor.path(CONTROL_MODE, message, self._now())

    def _planner_time(self, message):
        with self.lock:
            self.processor.planner_time(message, self._now())

    def _planning_stats(self, message):
        with self.lock:
            self.processor.planning_stats(message, self._now())

    def _isweep_status(self, message):
        with self.lock:
            self.processor.isweep_status(message, self._now())

    def _control_status(self, message):
        with self.lock:
            self.processor.control_status(message, self._now())

    def _timer(self, _event):
        with self.lock:
            self.processor.timeout(self._now(), self.timeout_seconds)

    def _shutdown(self):
        with self.lock:
            self.processor.close(self._now(), "recorder_shutdown")

    def spin(self):
        rospy.spin()


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description="Record live navigation experiments or summarize rosbag files."
    )
    parser.add_argument(
        "--bag", nargs="+", help="Offline mode: one or more rosbag files."
    )
    parser.add_argument(
        "--mode",
        choices=[AUTO_MODE, ISWEEP_MODE, CONTROL_MODE],
        default=None,
        help="Planner mode. Offline default: auto; live default: ROS param ~mode or auto.",
    )
    parser.add_argument(
        "--output",
        help="Output per-run CSV. Offline default: experiment_results.csv.",
    )
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--position-tolerance", type=float, default=0.10)
    parser.add_argument("--yaw-tolerance", type=float, default=0.20)
    parser.add_argument("--dwell-time", type=float, default=1.0)
    parser.add_argument("--max-cmd-gap", type=float, default=1.0)
    return parser.parse_args(argv)


def main():
    arguments = parse_arguments(rospy.myargv(argv=sys.argv)[1:])
    if arguments.bag:
        arguments.mode = arguments.mode or AUTO_MODE
        arguments.output = arguments.output or "experiment_results.csv"
        return process_bags(arguments)
    recorder = LiveRecorder(arguments)
    recorder.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
