#!/usr/bin/env python3

import json
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>rl_sim Status</title>
  <style>
    :root {
      color-scheme: light;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      --bg: #f5f7f2;
      --ink: #11140f;
      --muted: #5f675b;
      --line: #cbd2c2;
      --accent: #277a46;
      --accent-bg: #dcefe3;
      --panel: #ffffff;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--ink);
      overflow: hidden;
    }
    button {
      border: 1px solid var(--line);
      background: var(--panel);
      color: var(--ink);
      min-width: 42px;
      height: 42px;
      border-radius: 6px;
      font-size: 22px;
      cursor: pointer;
    }
    .shell {
      min-height: 100vh;
      padding: 28px;
      display: flex;
      flex-direction: column;
      gap: 22px;
      transform-origin: top left;
    }
    .topbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
    }
    .brand {
      display: flex;
      flex-direction: column;
      gap: 4px;
      min-width: 0;
    }
    .title {
      font-size: 18px;
      font-weight: 700;
      letter-spacing: 0;
    }
    .subtitle {
      font-size: 13px;
      color: var(--muted);
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }
    .tools {
      display: flex;
      align-items: center;
      gap: 8px;
      flex-shrink: 0;
    }
    .scale-readout {
      min-width: 56px;
      text-align: center;
      color: var(--muted);
      font-variant-numeric: tabular-nums;
    }
    .mode-band {
      border: 1px solid var(--line);
      background: var(--panel);
      min-height: 42vh;
      display: flex;
      flex-direction: column;
      justify-content: center;
      padding: 32px;
      border-left: 12px solid var(--accent);
    }
    .mode-label {
      color: var(--muted);
      font-size: 16px;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      margin-bottom: 14px;
    }
    .mode {
      font-size: clamp(42px, 9vw, 118px);
      line-height: 0.95;
      font-weight: 800;
      letter-spacing: 0;
      overflow-wrap: anywhere;
    }
    .details {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 12px;
    }
    .tile {
      border: 1px solid var(--line);
      background: var(--panel);
      border-radius: 6px;
      min-height: 112px;
      padding: 18px;
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      gap: 16px;
    }
    .tile .label {
      color: var(--muted);
      font-size: 13px;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }
    .tile .value {
      font-size: clamp(20px, 3vw, 34px);
      font-weight: 750;
      line-height: 1.05;
      overflow-wrap: anywhere;
    }
    .status-dot {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 9px 12px;
      border: 1px solid var(--line);
      background: var(--panel);
      border-radius: 999px;
      color: var(--muted);
      font-weight: 650;
      white-space: nowrap;
    }
    .status-dot::before {
      content: "";
      width: 9px;
      height: 9px;
      border-radius: 999px;
      background: var(--accent);
    }
    .stale .status-dot::before { background: #9a3412; }
    @media (max-width: 820px) {
      body { overflow: auto; }
      .shell { padding: 16px; }
      .topbar { align-items: flex-start; }
      .details { grid-template-columns: 1fr; }
      .mode-band { min-height: 34vh; padding: 24px; }
    }
  </style>
</head>
<body>
  <main class="shell" id="shell">
    <header class="topbar">
      <div class="brand">
        <div class="title">rl_sim Status</div>
        <div class="subtitle" id="topic">/rl_sim/runtime_status</div>
      </div>
      <div class="tools">
        <span class="status-dot" id="freshness">Waiting</span>
        <button id="zoomOut" title="Zoom out">-</button>
        <span class="scale-readout" id="scaleReadout">100%</span>
        <button id="zoomIn" title="Zoom in">+</button>
      </div>
    </header>
    <section class="mode-band" id="modeBand">
      <div class="mode-label">Current Mode</div>
      <div class="mode" id="mode">Waiting for rl_sim</div>
    </section>
    <section class="details">
      <div class="tile">
        <div class="label">Robot</div>
        <div class="value" id="robot">-</div>
      </div>
      <div class="tile">
        <div class="label">Policy</div>
        <div class="value" id="policy">-</div>
      </div>
      <div class="tile">
        <div class="label">Model</div>
        <div class="value" id="model">-</div>
      </div>
      <div class="tile">
        <div class="label">Navigation</div>
        <div class="value" id="nav">-</div>
      </div>
    </section>
  </main>
  <script>
    const shell = document.getElementById("shell");
    const scaleReadout = document.getElementById("scaleReadout");
    let scale = Number(localStorage.getItem("rlStatusScale") || "1");

    function applyScale() {
      scale = Math.max(0.7, Math.min(1.8, scale));
      shell.style.transform = `scale(${scale})`;
      shell.style.width = `${100 / scale}vw`;
      shell.style.minHeight = `${100 / scale}vh`;
      scaleReadout.textContent = `${Math.round(scale * 100)}%`;
      localStorage.setItem("rlStatusScale", String(scale));
    }

    document.getElementById("zoomIn").addEventListener("click", () => { scale += 0.1; applyScale(); });
    document.getElementById("zoomOut").addEventListener("click", () => { scale -= 0.1; applyScale(); });
    applyScale();

    const modeNames = {
      RLFSMStatePassive: "Passive",
      RLFSMStateGetUp: "Get Up",
      RLFSMStateGetDown: "Get Down",
      RLFSMStateRL_Locomotion: "RL Locomotion",
      RLFSMStatePolicyTransition: "Policy Switch",
      RLFSMStatePolicyReload: "Policy Reload",
      RLFSMStateBridgeDrive: "Bridge Drive",
      RLFSMStateBridgeToRLTransition: "Bridge to RL",
      RLFSMStateLowBarDrive: "Low Bar Drive",
      RLFSMStateLowBarToRLTransition: "Low Bar to RL",
      RLFSMStateCarDrive: "Car Drive",
      RLFSMStateCarToRLTransition: "Car to RL"
    };

    const palettes = {
      Passive: ["#4b5563", "#eef0f2"],
      "Get Up": ["#a16207", "#fff4cc"],
      "Get Down": ["#a16207", "#fff4cc"],
      "RL Locomotion": ["#277a46", "#dcefe3"],
      "Policy Switch": ["#c2410c", "#ffeadb"],
      "Policy Reload": ["#c2410c", "#ffeadb"],
      "Bridge Drive": ["#0369a1", "#e0f2fe"],
      "Low Bar Drive": ["#6d28d9", "#eee7ff"],
      "Car Drive": ["#0f766e", "#dbf4ef"]
    };

    function setText(id, value) {
      document.getElementById(id).textContent = value || "-";
    }

    function setPalette(mode) {
      const palette = palettes[mode] || ["#277a46", "#dcefe3"];
      document.documentElement.style.setProperty("--accent", palette[0]);
      document.documentElement.style.setProperty("--accent-bg", palette[1]);
      document.getElementById("modeBand").style.background = palette[1];
    }

    async function refresh() {
      try {
        const response = await fetch("/status.json", { cache: "no-store" });
        const payload = await response.json();
        const status = payload.status || {};
        const mode = modeNames[status.fsm_state] || status.fsm_state || "Waiting for rl_sim";
        setText("mode", mode);
        setText("robot", status.robot_name);
        setText("policy", status.policy_config);
        setText("model", status.model_name);
        setText("nav", status.navigation_mode === true ? "ON" : status.navigation_mode === false ? "OFF" : "-");
        document.body.classList.toggle("stale", !payload.fresh);
        document.getElementById("freshness").textContent = payload.fresh ? "Live" : "Stale";
        setPalette(mode);
      } catch (error) {
        document.body.classList.add("stale");
        document.getElementById("freshness").textContent = "Offline";
      }
    }

    refresh();
    setInterval(refresh, 250);
  </script>
</body>
</html>
"""


class StatusStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.status = {}
        self.updated_at = 0.0

    def update(self, message):
        try:
            status = json.loads(message)
        except json.JSONDecodeError:
            return
        with self.lock:
            self.status = status
            self.updated_at = time.time()

    def snapshot(self):
        with self.lock:
            age = time.time() - self.updated_at if self.updated_at else None
            return {
                "status": self.status,
                "age": age,
                "fresh": age is not None and age < 1.5,
            }


class Handler(BaseHTTPRequestHandler):
    store = None

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, "text/html; charset=utf-8", HTML.encode("utf-8"))
            return
        if self.path == "/status.json":
            data = json.dumps(self.store.snapshot()).encode("utf-8")
            self._send(200, "application/json; charset=utf-8", data)
            return
        self._send(404, "text/plain; charset=utf-8", b"not found")

    def log_message(self, fmt, *args):
        return

    def _send(self, code, content_type, body):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


class RuntimeStatusUi(Node):
    def __init__(self, store):
        super().__init__("rl_sim_status_ui")
        self.store = store
        self.create_subscription(String, "/rl_sim/runtime_status", self.status_callback, 10)

    def status_callback(self, msg):
        self.store.update(msg.data)


def open_browser(url):
    try:
        subprocess.Popen(
            ["xdg-open", url],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        pass


def main():
    rclpy.init()
    temp_node = Node("rl_sim_status_ui_config")
    port = int(temp_node.declare_parameter("port", 8765).value)
    open_browser_param = bool(temp_node.declare_parameter("open_browser", True).value)
    temp_node.destroy_node()

    store = StatusStore()
    Handler.store = store
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    url = f"http://127.0.0.1:{port}"
    print(f"[rl_sim_status_ui] serving {url}", flush=True)
    if open_browser_param:
        open_browser(url)

    node = RuntimeStatusUi(store)
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        server.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
