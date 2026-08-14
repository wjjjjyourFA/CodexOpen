#!/usr/bin/env python3

import configparser
import os
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class StreamState:
    def __init__(self, label):
        self.label = label
        self.last_receive = None
        self.detail = "等待消息"
        self.value = None


class StatusMonitor:
    def __init__(self):
        self.timeout_sec = max(0.001, float(rospy.get_param("~timeout_sec", 0.12)))
        self.waypoint_timeout_sec = max(
            0.001, float(rospy.get_param("~waypoint_timeout_sec", 0.25)))
        self.refresh_rate = max(1.0, float(rospy.get_param("~refresh_rate", 20.0)))
        self.display_mode = str(rospy.get_param("~display_mode", "gui")).lower()
        if self.display_mode not in ("gui", "terminal"):
            rospy.logwarn("Unknown display_mode '%s'; using terminal", self.display_mode)
            self.display_mode = "terminal"

        self.lock = threading.Lock()
        self.states = {
            "odometry": StreamState("里程计 / state_estimation"),
            "rog_map": StreamState("占据地图 / rog_map/occ"),
            "free_paths": StreamState("候选路径 / free_paths"),
            "cmd_vel": StreamState("控制指令 / cmd_vel_corrected"),
            "goal_valid": StreamState("目标状态 / isgoal_vaild"),
            "waypoint": StreamState("当前目标点 / way_point"),
        }

        self.topics = {
            "odometry": rospy.get_param("~state_estimation_topic", "/state_estimation"),
            "rog_map": rospy.get_param("~rog_map_occ_topic", "/rm_node/rog_map/occ"),
            "free_paths": rospy.get_param("~free_paths_topic", "/free_paths"),
            "cmd_vel": rospy.get_param("~cmd_vel_topic", "/cmd_vel_corrected"),
            "goal_valid": rospy.get_param("~goal_valid_topic", "/isgoal_vaild"),
            "waypoint": rospy.get_param("~waypoint_topic", "/way_point"),
        }

        self.subscribers = [
            rospy.Subscriber(self.topics["odometry"], Odometry,
                             self._odometry_callback, queue_size=5),
            rospy.Subscriber(self.topics["rog_map"], PointCloud2,
                             self._rog_map_callback, queue_size=2),
            rospy.Subscriber(self.topics["free_paths"], PointCloud2,
                             self._free_paths_callback, queue_size=2),
            rospy.Subscriber(self.topics["cmd_vel"], Twist,
                             self._cmd_vel_callback, queue_size=5),
            rospy.Subscriber(self.topics["goal_valid"], Bool,
                             self._goal_valid_callback, queue_size=5),
            rospy.Subscriber(self.topics["waypoint"], PoseStamped,
                             self._waypoint_callback, queue_size=5),
        ]

    @staticmethod
    def _now():
        return time.monotonic()

    def _update(self, key, detail, value=None):
        with self.lock:
            state = self.states[key]
            state.last_receive = self._now()
            state.detail = detail
            state.value = value

    def _odometry_callback(self, message):
        p = message.pose.pose.position
        self._update("odometry", "位置 x={:.2f}, y={:.2f}, z={:.2f}".format(
            p.x, p.y, p.z))

    def _rog_map_callback(self, message):
        self._update("rog_map", "点数 {}".format(message.width * message.height))

    def _free_paths_callback(self, message):
        self._update("free_paths", "候选路径点 {}".format(
            message.width * message.height))

    def _cmd_vel_callback(self, message):
        self._update(
            "cmd_vel",
            "线速度 x={:.2f}, y={:.2f}；角速度 z={:.2f}".format(
                message.linear.x, message.linear.y, message.angular.z))

    def _goal_valid_callback(self, message):
        valid = bool(message.data)
        self._update("goal_valid", "目标有效" if valid else "目标无效", valid)

    def _waypoint_callback(self, message):
        p = message.pose.position
        self._update("waypoint", "x={:.2f}, y={:.2f}, z={:.2f}".format(
            p.x, p.y, p.z))

    def snapshot(self):
        now = self._now()
        result = []
        with self.lock:
            for key, state in self.states.items():
                age = None if state.last_receive is None else now - state.last_receive
                timeout = (self.waypoint_timeout_sec
                           if key == "waypoint" else self.timeout_sec)
                fresh = age is not None and age <= timeout
                online = fresh and (key != "goal_valid" or state.value is True)
                if age is None:
                    age_text = "从未收到"
                else:
                    age_text = "{:.0f} ms".format(age * 1000.0)
                detail = state.detail
                if key == "goal_valid" and fresh and state.value is False:
                    detail += "（话题在线）"
                result.append((key, state.label, self.topics[key], online,
                               fresh, age_text, detail))
        return result

    def run_terminal(self):
        period = 1.0 / self.refresh_rate
        interactive = os.isatty(1)
        while not rospy.is_shutdown():
            rows = self.snapshot()
            if interactive:
                print("\033[2J\033[H", end="")
            print("机器人功能状态监看器  默认超时: {:.0f} ms  "
                  "way_point: {:.0f} ms".format(
                      self.timeout_sec * 1000.0,
                      self.waypoint_timeout_sec * 1000.0))
            print("-" * 92)
            for _, label, topic, online, fresh, age, detail in rows:
                status = "在线" if online else "离线"
                if fresh and not online:
                    status = "无有效目标"
                print("[{:<10}] {:<30} {:<24} {:<12} {}".format(
                    status, label, topic, age, detail))
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
        root.protocol("WM_DELETE_WINDOW",
                      lambda: rospy.signal_shutdown("status window closed"))

        tk.Label(root, text="机器人功能状态", font=("Sans", 16, "bold")).grid(
            row=0, column=0, columnspan=4, padx=12, pady=(12, 4))
        tk.Label(root, text="红色：离线/无效    绿色：在线",
                 fg="#555555").grid(row=1, column=0, columnspan=4, pady=(0, 10))

        widgets = {}
        for row_index, (key, state) in enumerate(self.states.items(), start=2):
            canvas = tk.Canvas(root, width=34, height=34, highlightthickness=0)
            lamp = canvas.create_oval(6, 6, 28, 28, fill="#d32f2f", outline="#7f0000")
            canvas.grid(row=row_index, column=0, padx=(12, 4), pady=5)
            tk.Label(root, text=state.label, width=26, anchor="w").grid(
                row=row_index, column=1, padx=4)
            status_label = tk.Label(root, text="离线", width=12, anchor="w")
            status_label.grid(row=row_index, column=2, padx=4)
            detail_label = tk.Label(root, text="等待消息", width=42, anchor="w")
            detail_label.grid(row=row_index, column=3, padx=(4, 12))
            widgets[key] = (canvas, lamp, status_label, detail_label)

        def refresh():
            if rospy.is_shutdown():
                root.destroy()
                return
            for key, _, _, online, fresh, age, detail in self.snapshot():
                canvas, lamp, status_label, detail_label = widgets[key]
                color = "#2e7d32" if online else "#d32f2f"
                outline = "#005005" if online else "#7f0000"
                canvas.itemconfigure(lamp, fill=color, outline=outline)
                status = "在线" if online else "离线"
                if fresh and not online:
                    status = "无有效目标"
                status_label.configure(text="{} ({})".format(status, age))
                detail_label.configure(text=detail)
            root.after(max(10, int(1000.0 / self.refresh_rate)), refresh)

        refresh()
        root.mainloop()

    def run(self):
        rospy.loginfo("robot_status_monitor mode=%s timeout=%.0fms "
                      "waypoint_timeout=%.0fms",
                      self.display_mode, self.timeout_sec * 1000.0,
                      self.waypoint_timeout_sec * 1000.0)
        if self.display_mode == "gui":
            self.run_gui()
        else:
            self.run_terminal()


def load_standard_config(runtime_path, interface_path, mode_override=None):
    runtime = configparser.ConfigParser()
    interface = configparser.ConfigParser()
    if not runtime.read(runtime_path):
        raise RuntimeError("cannot read runtime config: {}".format(runtime_path))
    if not interface.read(interface_path):
        raise RuntimeError("cannot read interface config: {}".format(interface_path))

    rospy.set_param("~display_mode",
                    mode_override or runtime.get("runtime", "display_mode"))
    rospy.set_param("~timeout_sec",
                    runtime.getfloat("runtime", "timeout_sec"))
    rospy.set_param("~waypoint_timeout_sec",
                    runtime.getfloat("runtime", "waypoint_timeout_sec"))
    rospy.set_param("~refresh_rate",
                    runtime.getfloat("runtime", "refresh_rate"))
    for key, value in interface.items("topics"):
        rospy.set_param("~" + key, value)


if __name__ == "__main__":
    rospy.init_node("robot_status_monitor")
    arguments = rospy.myargv(argv=sys.argv)
    if len(arguments) > 1:
        try:
            load_standard_config(
                arguments[1],
                arguments[2] if len(arguments) > 2 else
                "./../../../config/RobotStatusMonitor/Interface.ini",
                arguments[3] if len(arguments) > 3 else None)
        except Exception as error:
            rospy.logfatal("status monitor configuration failed: %s", error)
            raise SystemExit(1)
    StatusMonitor().run()
