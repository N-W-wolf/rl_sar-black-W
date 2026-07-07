#!/usr/bin/env python3

import json
import threading
import time
import tkinter as tk
import tkinter.font as tkfont

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_msgs.msg import RobotState
from sensor_msgs.msg import Joy
from std_msgs.msg import String


MODE_NAMES = {
    "RLFSMStatePassive": "Passive",
    "RLFSMStateGetUp": "Get Up",
    "RLFSMStateGetDown": "Get Down",
    "RLFSMStateRL_Locomotion": "RL Locomotion",
    "RLFSMStatePolicyTransition": "Policy Switch",
    "RLFSMStatePolicyReload": "Policy Reload",
    "RLFSMStateBridgeDrive": "Bridge Drive",
    "RLFSMStateBridgeToRLTransition": "Bridge to RL",
    "RLFSMStateLowBarDrive": "Low Bar Drive",
    "RLFSMStateLowBarToRLTransition": "Low Bar to RL",
    "RLFSMStateCarDrive": "Car Drive",
    "RLFSMStateCarToRLTransition": "Car to RL",
}

PALETTES = {
    "Passive": ("#4b5563", "#eef0f2"),
    "Get Up": ("#a16207", "#fff4cc"),
    "Get Down": ("#a16207", "#fff4cc"),
    "RL Locomotion": ("#277a46", "#dcefe3"),
    "Policy Switch": ("#c2410c", "#ffeadb"),
    "Policy Reload": ("#c2410c", "#ffeadb"),
    "Bridge Drive": ("#0369a1", "#e0f2fe"),
    "Bridge to RL": ("#0369a1", "#e0f2fe"),
    "Low Bar Drive": ("#6d28d9", "#eee7ff"),
    "Low Bar to RL": ("#6d28d9", "#eee7ff"),
    "Car Drive": ("#0f766e", "#dbf4ef"),
    "Car to RL": ("#0f766e", "#dbf4ef"),
}

COLORS = {
    "bg": "#f5f7f2",
    "panel": "#ffffff",
    "ink": "#11140f",
    "muted": "#5f675b",
    "line": "#cbd2c2",
    "stale": "#9a3412",
}

POLICY_PALETTE = [
    ("#0f766e", "#def7f1"),
    ("#2563eb", "#e5efff"),
    ("#7c3aed", "#f0e9ff"),
    ("#c2410c", "#ffeadb"),
    ("#b45309", "#fff2d6"),
    ("#be185d", "#ffe4f0"),
    ("#047857", "#e1f6e9"),
]

STATUS_PILLS = ("Pad", "Runner")
HEARTBEAT_TIMEOUT_SEC = 0.75


class StatusStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = {}
        self.health_states = {}
        self.updated_at = 0.0

    def update(self, message):
        try:
            status = json.loads(message)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.status = status
            self.updated_at = time.time()

    def update_health(self, health_states):
        with self.lock:
            self.health_states = dict(health_states)

    def snapshot(self):
        with self.lock:
            age = time.time() - self.updated_at if self.updated_at else None
            return {
                "status": dict(self.status),
                "health_states": dict(self.health_states),
                "age": age,
                "fresh": age is not None and age < 1.5,
            }


class RuntimeStatusNode(Node):
    def __init__(self, store):
        super().__init__(
            "rl_sim_status_ui",
            automatically_declare_parameters_from_overrides=True,
        )
        self.store = store
        self.joy_seen_at = 0.0
        self.joy_connected = False
        self.runner_seen_at = 0.0
        self.create_subscription(String, "/rl_sim/runtime_status", self.status_callback, 10)
        self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.create_subscription(RobotState, "/_lowState/joint", self.runner_state_callback, 10)
        self.create_timer(0.2, self.update_health_states)
        self.update_health_states()

    def status_callback(self, msg):
        self.store.update(msg.data)

    def joy_callback(self, msg):
        self.joy_seen_at = time.time()
        frame_id = str(msg.header.frame_id or "")
        self.joy_connected = frame_id != "joy_disconnected"
        self.update_health_states()

    def runner_state_callback(self, _msg):
        self.runner_seen_at = time.time()
        self.update_health_states()

    def update_health_states(self):
        now = time.time()
        joy_fresh = now - self.joy_seen_at < HEARTBEAT_TIMEOUT_SEC if self.joy_seen_at else False
        runner_fresh = now - self.runner_seen_at < HEARTBEAT_TIMEOUT_SEC if self.runner_seen_at else False
        self.store.update_health({
            "Pad": joy_fresh and self.joy_connected,
            "Runner": runner_fresh,
        })


class StatusWindow:
    def __init__(self, store, on_close):
        self.store = store
        self.on_close = on_close
        self.root = tk.Tk()
        self.root.title("rl_sim Status")
        self.root.configure(bg=COLORS["bg"])
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        width = max(760, min(1180, int(screen_w * 0.82)))
        height = max(440, min(700, int(screen_h * 0.72)))
        x = max(0, int((screen_w - width) / 2))
        y = max(0, int((screen_h - height) / 2))
        self.root.geometry(f"{width}x{height}+{x}+{y}")
        self.root.minsize(680, 390)

        self.canvas = tk.Canvas(self.root, highlightthickness=0, bg=COLORS["bg"])
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.draw())

        self.snapshot = {"status": {}, "health_states": {}, "age": None, "fresh": False}
        self.root.after(120, self.refresh)

    def run(self):
        self.root.mainloop()

    def close(self):
        self.on_close()
        self.root.destroy()

    def refresh(self):
        self.snapshot = self.store.snapshot()
        self.draw()
        self.root.after(200, self.refresh)

    def font(self, size, weight="normal"):
        return tkfont.Font(family="DejaVu Sans", size=max(7, int(size)), weight=weight)

    def fit_text(self, text, font, max_width):
        text = str(text or "-")
        if font.measure(text) <= max_width:
            return text
        ellipsis = "..."
        available = max_width - font.measure(ellipsis)
        if available <= 0:
            return ellipsis
        trimmed = text
        while trimmed and font.measure(trimmed) > available:
            trimmed = trimmed[:-1]
        return trimmed.rstrip() + ellipsis

    def adaptive_font(self, text, max_width, base_size, min_size, weight="bold"):
        text = str(text or "-")
        for size in range(int(base_size), int(min_size) - 1, -1):
            font = self.font(size, weight)
            if font.measure(text) <= max_width:
                return font
        return self.font(min_size, weight)

    def palette_for_policy(self, policy):
        policy = str(policy or "")
        index = sum((i + 1) * ord(ch) for i, ch in enumerate(policy)) % len(POLICY_PALETTE)
        return POLICY_PALETTE[index]

    def format_speed(self, value):
        try:
            return f"{float(value):+.2f}"
        except (TypeError, ValueError):
            return "-"

    def draw_round_rect(self, x1, y1, x2, y2, radius, fill, outline=None, width=1):
        radius = min(radius, (x2 - x1) / 2, (y2 - y1) / 2)
        points = [
            x1 + radius, y1, x2 - radius, y1, x2, y1, x2, y1 + radius,
            x2, y2 - radius, x2, y2, x2 - radius, y2, x1 + radius, y2,
            x1, y2, x1, y2 - radius, x1, y1 + radius, x1, y1,
        ]
        self.canvas.create_polygon(points, smooth=True, fill=fill, outline=outline or fill, width=width)

    def draw(self):
        c = self.canvas
        c.delete("all")
        w = max(1, c.winfo_width())
        h = max(1, c.winfo_height())
        s = min(w / 1120.0, h / 660.0)

        margin = 28 * s
        top_h = 72 * s
        details_h = max(178 * s, h * 0.30)
        mode_y1 = margin + top_h + 18 * s
        mode_y2 = h - margin - details_h - 18 * s
        mode_y2 = max(mode_y1 + 170 * s, mode_y2)
        details_y1 = mode_y2 + 18 * s
        details_y2 = h - margin

        status = self.snapshot["status"]
        health_states = self.snapshot.get("health_states", {})
        fresh = self.snapshot["fresh"]
        fsm_state = status.get("fsm_state", "")
        mode = MODE_NAMES.get(fsm_state, fsm_state or "Waiting for rl_sim")
        accent, accent_bg = PALETTES.get(mode, ("#277a46", "#dcefe3"))

        c.create_rectangle(0, 0, w, h, fill=COLORS["bg"], outline="")

        title_font = self.font(18 * s, "bold")
        sub_font = self.font(12 * s)
        pill_font = self.font(13 * s, "bold")
        label_font = self.font(13 * s, "bold")

        title_y = margin + 2 * s
        sub_y = title_y + title_font.metrics("linespace") + 5 * s
        c.create_text(margin, title_y, anchor="nw", text="rl_sim Status", fill=COLORS["ink"], font=title_font)
        c.create_text(margin, sub_y, anchor="nw", text="/rl_sim/runtime_status", fill=COLORS["muted"], font=sub_font)

        pill_text = "Live" if fresh else ("Stale" if status else "Waiting")
        pill_w = max(92 * s, pill_font.measure(pill_text) + 48 * s)
        pill_h = 34 * s
        pill_x2 = w - margin
        pill_x1 = pill_x2 - pill_w
        pill_y1 = margin + 9 * s
        self.draw_round_rect(pill_x1, pill_y1, pill_x2, pill_y1 + pill_h, pill_h / 2, COLORS["panel"], COLORS["line"], 1)
        dot_color = accent if fresh else COLORS["stale"]
        c.create_oval(pill_x1 + 13 * s, pill_y1 + 12 * s, pill_x1 + 22 * s, pill_y1 + 21 * s, fill=dot_color, outline=dot_color)
        c.create_text(pill_x1 + 30 * s, pill_y1 + pill_h / 2, anchor="w", text=pill_text, fill=COLORS["muted"], font=pill_font)

        node_gap = 8 * s
        node_x2 = pill_x1 - node_gap
        for label in reversed(STATUS_PILLS):
            online = bool(health_states.get(label, False))
            node_text = label
            node_w = max(74 * s, pill_font.measure(node_text) + 34 * s)
            node_x1 = node_x2 - node_w
            node_color = accent if online else COLORS["stale"]
            self.draw_round_rect(node_x1, pill_y1, node_x2, pill_y1 + pill_h, pill_h / 2, COLORS["panel"], COLORS["line"], 1)
            c.create_oval(node_x1 + 11 * s, pill_y1 + 13 * s, node_x1 + 20 * s, pill_y1 + 22 * s, fill=node_color, outline=node_color)
            c.create_text(node_x1 + 27 * s, pill_y1 + pill_h / 2, anchor="w", text=node_text, fill=COLORS["muted"], font=pill_font)
            node_x2 = node_x1 - node_gap

        main_gap = 14 * s
        main_width = w - 2 * margin
        speed_w = max(190 * s, min(260 * s, main_width * 0.28))
        mode_x1 = margin
        mode_x2 = w - margin - speed_w - main_gap
        speed_x1 = mode_x2 + main_gap
        speed_x2 = w - margin

        c.create_rectangle(mode_x1, mode_y1, mode_x2, mode_y2, fill=accent_bg, outline=COLORS["line"], width=max(1, int(s)))
        c.create_rectangle(mode_x1, mode_y1, mode_x1 + 12 * s, mode_y2, fill=accent, outline=accent)
        c.create_text(mode_x1 + 34 * s, mode_y1 + 34 * s, anchor="nw", text="CURRENT MODE", fill=COLORS["muted"], font=label_font)
        mode_max_width = mode_x2 - mode_x1 - 70 * s
        mode_base_size = min(104 * s, max(30, (mode_y2 - mode_y1) * 0.30))
        mode_font = self.adaptive_font(mode, mode_max_width, mode_base_size, 30 * s, "bold")
        mode_text = self.fit_text(mode, mode_font, mode_max_width)
        c.create_text(mode_x1 + 34 * s, (mode_y1 + mode_y2) / 2 + 18 * s, anchor="w", text=mode_text, fill=COLORS["ink"], font=mode_font)

        command_x = self.format_speed(status.get("command_x"))
        command_yaw = self.format_speed(status.get("command_yaw"))
        speed_bg = "#eef7f4" if fresh else COLORS["panel"]
        self.draw_round_rect(speed_x1, mode_y1, speed_x2, mode_y2, 7 * s, speed_bg, COLORS["line"], 1)
        c.create_rectangle(speed_x1, mode_y1, speed_x1 + 8 * s, mode_y2, fill=accent, outline=accent)
        c.create_text(speed_x1 + 22 * s, mode_y1 + 28 * s, anchor="nw", text="COMMAND", fill=COLORS["muted"], font=label_font)

        speed_label_font = self.font(14 * s, "bold")
        speed_value_font = self.adaptive_font("+0.00", speed_x2 - speed_x1 - 92 * s, 31 * s, 15 * s, "bold")
        row1_y = mode_y1 + (mode_y2 - mode_y1) * 0.44
        row2_y = mode_y1 + (mode_y2 - mode_y1) * 0.70
        label_x = speed_x1 + 24 * s
        value_x = speed_x2 - 22 * s
        c.create_text(label_x, row1_y, anchor="w", text="X", fill=COLORS["muted"], font=speed_label_font)
        c.create_text(value_x, row1_y, anchor="e", text=command_x, fill=COLORS["ink"], font=speed_value_font)
        c.create_text(label_x, row2_y, anchor="w", text="YAW", fill=COLORS["muted"], font=speed_label_font)
        c.create_text(value_x, row2_y, anchor="e", text=command_yaw, fill=COLORS["ink"], font=speed_value_font)

        robot = status.get("robot_name", "-")
        policy = status.get("policy_config", "-")
        model = status.get("model_name", "-")
        navigation = "ON" if status.get("navigation_mode") is True else "OFF" if status.get("navigation_mode") is False else "-"

        policy_accent, policy_bg = self.palette_for_policy(policy)
        nav_accent = accent if navigation == "ON" else COLORS["muted"]
        nav_bg = accent_bg if navigation == "ON" else COLORS["panel"]

        info_gap = 14 * s
        left_w = max(190 * s, (w - 2 * margin - info_gap) * 0.28)
        right_w = w - 2 * margin - info_gap - left_w
        left_x1 = margin
        left_x2 = left_x1 + left_w
        right_x1 = left_x2 + info_gap
        right_x2 = w - margin
        row_gap = 12 * s
        row_h = (details_y2 - details_y1 - row_gap) / 2

        def draw_tile(x1, y1, x2, y2, label, value, bg, stripe, base_size, min_size):
            self.draw_round_rect(x1, y1, x2, y2, 7 * s, bg, COLORS["line"], 1)
            c.create_rectangle(x1, y1, x1 + 8 * s, y2, fill=stripe, outline=stripe)
            c.create_text(x1 + 22 * s, y1 + 15 * s, anchor="nw", text=label.upper(), fill=COLORS["muted"], font=label_font)
            max_value_width = x2 - x1 - 44 * s
            max_value_height = y2 - y1 - 54 * s
            tile_value_font = self.adaptive_font(value, max_value_width, base_size, min_size, "bold")
            current_size = int(tile_value_font.cget("size"))
            while tile_value_font.metrics("linespace") > max_value_height and current_size > int(min_size):
                current_size -= 1
                tile_value_font = self.font(current_size, "bold")
            fitted = self.fit_text(value, tile_value_font, max_value_width)
            c.create_text(x1 + 22 * s, y2 - 18 * s, anchor="sw", text=fitted, fill=COLORS["ink"], font=tile_value_font)

        draw_tile(left_x1, details_y1, left_x2, details_y1 + row_h, "Robot", robot, COLORS["panel"], accent, 28 * s, 13 * s)
        draw_tile(left_x1, details_y1 + row_h + row_gap, left_x2, details_y2, "Navigation", navigation, nav_bg, nav_accent, 30 * s, 14 * s)
        draw_tile(right_x1, details_y1, right_x2, details_y1 + row_h, "Policy", policy, policy_bg, policy_accent, 38 * s, 15 * s)
        draw_tile(right_x1, details_y1 + row_h + row_gap, right_x2, details_y2, "Model", model, COLORS["panel"], accent, 36 * s, 15 * s)


def spin_ros(node):
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


def main():
    rclpy.init()
    store = StatusStore()
    node = RuntimeStatusNode(store)
    ros_thread = threading.Thread(target=spin_ros, args=(node,), daemon=True)
    ros_thread.start()

    def shutdown():
        if rclpy.ok():
            rclpy.shutdown()

    try:
        window = StatusWindow(store, shutdown)
        print("[rl_sim_status_ui] window started", flush=True)
        window.run()
    finally:
        shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
