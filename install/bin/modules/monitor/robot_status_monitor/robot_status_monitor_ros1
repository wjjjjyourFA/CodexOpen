#!/usr/bin/env python3

"""ROS1 conversion and presentation adapter for robot_status_monitor_core."""

import configparser
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


try:
    from robot_status_monitor_core import (RobotStatusMonitorCore,
                                           load_runtime_config)
except ImportError:
    # Support running the adapter directly from the source tree. Installed
    # layouts place the core next to this executable and use the branch above.
    _SOURCE_ROOT = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "../../../../.."))
    sys.path.insert(
        0, os.path.join(_SOURCE_ROOT, "modules", "monitor",
                        "robot_status_monitor"))
    from robot_status_monitor_core import (RobotStatusMonitorCore,
                                           load_runtime_config)


_TOPIC_KEYS = (
    "state_estimation_topic",
    "rog_map_occ_topic",
    "free_paths_topic",
    "cmd_vel_topic",
    "goal_valid_topic",
    "waypoint_topic",
)

_QUEUE_KEYS = (
    "state_estimation",
    "rog_map_occ",
    "free_paths",
    "cmd_vel",
    "goal_valid",
    "waypoint",
)


def load_interface_config(path):
    parser = configparser.ConfigParser()
    if not parser.read(path, encoding="utf-8"):
        raise RuntimeError("cannot read interface config: {}".format(path))
    if not parser.has_section("topics"):
        raise RuntimeError("interface config has no [topics] section: {}".format(path))
    if not parser.has_section("queues"):
        raise RuntimeError("interface config has no [queues] section: {}".format(path))

    topics = {key: parser.get("topics", key).strip() for key in _TOPIC_KEYS}
    queues = {key: parser.getint("queues", key) for key in _QUEUE_KEYS}
    for key, topic in topics.items():
        if not topic:
            raise ValueError("empty topic in interface config: {}".format(key))
    for key, queue_size in queues.items():
        if queue_size <= 0:
            raise ValueError("queue size must be positive: {}".format(key))
    return topics, queues


class Ros1StatusMonitor:
    def __init__(self, runtime_config, topics, queues):
        self.config = runtime_config
        self.core = RobotStatusMonitorCore(runtime_config)
        self.topics = {
            "odometry": topics["state_estimation_topic"],
            "rog_map": topics["rog_map_occ_topic"],
            "free_paths": topics["free_paths_topic"],
            "cmd_vel": topics["cmd_vel_topic"],
            "goal_valid": topics["goal_valid_topic"],
            "waypoint": topics["waypoint_topic"],
        }
        self.subscribers = [
            rospy.Subscriber(self.topics["odometry"], Odometry,
                             self._odometry_callback,
                             queue_size=queues["state_estimation"]),
            rospy.Subscriber(self.topics["rog_map"], PointCloud2,
                             self._rog_map_callback,
                             queue_size=queues["rog_map_occ"]),
            rospy.Subscriber(self.topics["free_paths"], PointCloud2,
                             self._free_paths_callback,
                             queue_size=queues["free_paths"]),
            rospy.Subscriber(self.topics["cmd_vel"], Twist,
                             self._cmd_vel_callback,
                             queue_size=queues["cmd_vel"]),
            rospy.Subscriber(self.topics["goal_valid"], Bool,
                             self._goal_valid_callback,
                             queue_size=queues["goal_valid"]),
            rospy.Subscriber(self.topics["waypoint"], PoseStamped,
                             self._waypoint_callback,
                             queue_size=queues["waypoint"]),
        ]

    @staticmethod
    def _now():
        return time.monotonic()

    def _odometry_callback(self, message):
        position = message.pose.pose.position
        self.core.update_odometry(
            self._now(), position.x, position.y, position.z)

    def _rog_map_callback(self, message):
        self.core.update_rog_map(
            self._now(), message.width * message.height)

    def _free_paths_callback(self, message):
        self.core.update_free_paths(
            self._now(), message.width * message.height)

    def _cmd_vel_callback(self, message):
        self.core.update_command(
            self._now(), message.linear.x, message.linear.y,
            message.angular.z)

    def _goal_valid_callback(self, message):
        self.core.update_goal_valid(self._now(), bool(message.data))

    def _waypoint_callback(self, message):
        position = message.pose.position
        self.core.update_waypoint(
            self._now(), position.x, position.y, position.z)

    @staticmethod
    def _age_text(age_seconds):
        if age_seconds is None:
            return "从未收到"
        return "{:.0f} ms".format(age_seconds * 1000.0)

    def _snapshot(self):
        return self.core.snapshot(self._now())

    def run_terminal(self):
        period = 1.0 / self.config.refresh_rate
        interactive = os.isatty(1)
        while not rospy.is_shutdown():
            rows = self._snapshot()
            if interactive:
                print("\033[2J\033[H", end="")
            print("机器人功能状态监看器  默认超时: {:.0f} ms  "
                  "way_point: {:.0f} ms".format(
                      self.config.timeout_sec * 1000.0,
                      self.config.waypoint_timeout_sec * 1000.0))
            print("-" * 92)
            for row in rows:
                status = "在线" if row.online else "离线"
                if row.fresh and not row.online:
                    status = "无有效目标"
                print("[{:<10}] {:<30} {:<24} {:<12} {}".format(
                    status, row.label, self.topics[row.key],
                    self._age_text(row.age_seconds), row.detail))
            print("-" * 92, flush=True)
            rospy.sleep(period)

    def run_gui(self):
        try:
            import tkinter as tk
        except ImportError:
            rospy.logerr("GUI mode needs tkinter (Ubuntu: sudo apt install python3-tk)")
            rospy.logwarn("Falling back to terminal mode")
            self.run_terminal()
            return

        try:
            root = tk.Tk()
        except Exception as error:
            rospy.logerr("Cannot open GUI (%s); falling back to terminal mode", error)
            self.run_terminal()
            return
        root.title("机器人功能状态监看器")
        root.resizable(False, False)
        root.protocol(
            "WM_DELETE_WINDOW",
            lambda: rospy.signal_shutdown("status window closed"))

        tk.Label(root, text="机器人功能状态", font=("Sans", 16, "bold")).grid(
            row=0, column=0, columnspan=4, padx=12, pady=(12, 4))
        tk.Label(root, text="红色：离线/无效    绿色：在线", fg="#555555").grid(
            row=1, column=0, columnspan=4, pady=(0, 10))

        widgets = {}
        for row_index, (key, label) in enumerate(
                self.core.stream_definitions(), start=2):
            canvas = tk.Canvas(root, width=34, height=34, highlightthickness=0)
            lamp = canvas.create_oval(
                6, 6, 28, 28, fill="#d32f2f", outline="#7f0000")
            canvas.grid(row=row_index, column=0, padx=(12, 4), pady=5)
            tk.Label(root, text=label, width=26, anchor="w").grid(
                row=row_index, column=1, padx=4)
            status_label = tk.Label(root, text="离线", width=18, anchor="w")
            status_label.grid(row=row_index, column=2, padx=4)
            detail_label = tk.Label(root, text="等待消息", width=42, anchor="w")
            detail_label.grid(row=row_index, column=3, padx=(4, 12))
            widgets[key] = (canvas, lamp, status_label, detail_label)

        def refresh():
            if rospy.is_shutdown():
                root.destroy()
                return
            for row in self._snapshot():
                canvas, lamp, status_label, detail_label = widgets[row.key]
                color = "#2e7d32" if row.online else "#d32f2f"
                outline = "#005005" if row.online else "#7f0000"
                canvas.itemconfigure(lamp, fill=color, outline=outline)
                status = "在线" if row.online else "离线"
                if row.fresh and not row.online:
                    status = "无有效目标"
                status_label.configure(text="{} ({})".format(
                    status, self._age_text(row.age_seconds)))
                detail_label.configure(text=row.detail)
            root.after(
                max(10, int(1000.0 / self.config.refresh_rate)), refresh)

        refresh()
        root.mainloop()

    def run(self):
        rospy.loginfo(
            "robot_status_monitor mode=%s timeout=%.0fms waypoint_timeout=%.0fms",
            self.config.display_mode, self.config.timeout_sec * 1000.0,
            self.config.waypoint_timeout_sec * 1000.0)
        if self.config.display_mode == "gui":
            self.run_gui()
        else:
            self.run_terminal()


def main():
    rospy.init_node("robot_status_monitor")
    arguments = rospy.myargv(argv=sys.argv)
    if len(arguments) < 3:
        rospy.logfatal(
            "usage: robot_status_monitor_ros1 <runtime.ini> <Interface.ini> "
            "[gui|terminal]")
        return 1
    try:
        runtime_config = load_runtime_config(
            arguments[1], arguments[3] if len(arguments) > 3 else None)
        topics, queues = load_interface_config(arguments[2])
    except (configparser.Error, KeyError, RuntimeError, ValueError) as error:
        rospy.logfatal("status monitor configuration failed: %s", error)
        return 1
    Ros1StatusMonitor(runtime_config, topics, queues).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
