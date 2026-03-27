#!/usr/bin/env python3
import argparse
import json
import os
import shlex
import shutil
import signal
import subprocess
import threading
import time
from functools import partial
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict

UI_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = UI_ROOT.parent
RUNTIME_DIR = UI_ROOT / "runtime"


def build_shell_prefix() -> str:
    parts = ["source /opt/ros/noetic/setup.bash"]
    catkin_setup = Path.home() / "catkin_ws" / "devel" / "setup.bash"
    if catkin_setup.exists():
        parts.append(f"source {shlex.quote(str(catkin_setup))}")
    parts.append(f"cd {shlex.quote(str(PROJECT_ROOT))}")
    return "; ".join(parts)


START_COMMANDS = {
    "motion": [
        "rosrun",
        "memory_game",
        "motion_moveit_node",
        "_planning_group:=arm",
    ],
    "vision": [
        "rosrun",
        "memory_game",
        "vision_node",
        "_disable_red:=true",
        "_max_depth_age_sec:=1.0",
        "_depth_buffer_size:=10",
        "_enable_player_detection:=false",
        "_workspace_enable:=false",
        "_roi_enable:=false",
        "_min_block_area:=1500",
        "_mask_open_iterations:=2",
        "_mask_close_iterations:=2",
    ],
    "game": [
        "rosrun",
        "memory_game",
        "game_node",
        "_disable_red:=true",
        "_require_detected_blocks:=true",
        "_min_detected_blocks_required:=3",
    ],
}


def iso_now() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


class BridgeState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = {
            "bridge_status": "starting",
            "bridge_message": "Preparing ROS bridge.",
            "score": None,
            "game_state": "UNKNOWN",
            "motion_status": "UNKNOWN",
            "detected_block_count": 0,
            "detected_blocks": [],
            "player_selection": None,
            "process_status": {},
            "last_update": None,
            "log": [
                {"time": iso_now(), "message": "UI bridge process started."},
            ],
        }

    def snapshot(self):
        with self._lock:
            return json.loads(json.dumps(self._state))

    def update(self, **kwargs):
        with self._lock:
            self._state.update(kwargs)
            self._state["last_update"] = iso_now()

    def append_log(self, message: str):
        with self._lock:
            self._state["log"].insert(0, {"time": iso_now(), "message": message})
            self._state["log"] = self._state["log"][:16]
            self._state["last_update"] = iso_now()


class ProcessManager:
    def __init__(self, state: BridgeState) -> None:
        self.state = state
        self._lock = threading.Lock()
        self._procs: Dict[str, subprocess.Popen] = {}
        self._logs: Dict[str, object] = {}
        self._terminal_cmd = self._detect_terminal()
        RUNTIME_DIR.mkdir(exist_ok=True)
        self._refresh_state()

    def _detect_terminal(self):
        for candidate in ("gnome-terminal", "x-terminal-emulator"):
            path = shutil.which(candidate)
            if path:
                return path
        return None

    def _launch_in_terminal(self, name: str, cmd):
        if not self._terminal_cmd:
            raise RuntimeError("No supported terminal emulator found (expected gnome-terminal or x-terminal-emulator)")

        log_path = RUNTIME_DIR / f"{name}.log"
        shell_prefix = build_shell_prefix()
        command_text = shlex.join(cmd)
        log_text = shlex.quote(str(log_path))
        shell_script = (
            f"{shell_prefix}; "
            f"echo '[{name}] {command_text}'; "
            f"set -o pipefail; {command_text} 2>&1 | tee -a {log_text}; "
            "status=${PIPESTATUS[0]}; "
            'echo; echo "Exit code: $status"; '
            "exec bash"
        )

        if os.path.basename(self._terminal_cmd) == "gnome-terminal":
            terminal_cmd = [
                self._terminal_cmd,
                "--title",
                f"memory_game_{name}",
                "--",
                "bash",
                "-lc",
                shell_script,
            ]
        else:
            terminal_cmd = [
                self._terminal_cmd,
                "-T",
                f"memory_game_{name}",
                "-e",
                "bash",
                "-lc",
                shell_script,
            ]

        proc = subprocess.Popen(terminal_cmd, cwd=str(PROJECT_ROOT), start_new_session=True)
        self.state.append_log(f"Opened terminal for {name} (pid={proc.pid}).")
        return proc

    def _status_for(self, name: str) -> dict:
        proc = self._procs.get(name)
        log_path = str(RUNTIME_DIR / f"{name}.log")
        if proc is None:
            return {"running": False, "pid": None, "log_path": log_path}
        running = proc.poll() is None
        return {"running": running, "pid": proc.pid, "log_path": log_path}

    def _refresh_state(self) -> None:
        self.state.update(process_status={name: self._status_for(name) for name in START_COMMANDS})

    def start_all(self) -> dict:
        with self._lock:
            already_running = [name for name, proc in self._procs.items() if proc.poll() is None]
            if already_running:
                self._refresh_state()
                return {
                    "ok": False,
                    "message": f"Processes already running: {', '.join(sorted(already_running))}",
                }

            started = []
            try:
                for name, cmd in START_COMMANDS.items():
                    log_path = RUNTIME_DIR / f"{name}.log"
                    with open(log_path, "a", encoding="utf-8") as log_file:
                        log_file.write(f"\n[{iso_now()}] Starting in terminal: {' '.join(cmd)}\n")
                    proc = self._launch_in_terminal(name, cmd)
                    self._procs[name] = proc
                    started.append(name)
            except FileNotFoundError as exc:
                self.state.append_log(f"Start failed: {exc}")
                self._stop_locked()
                self._refresh_state()
                return {
                    "ok": False,
                    "message": "Could not run ROS commands. Source the ROS/catkin environment before starting the UI server.",
                }
            except Exception as exc:  # pragma: no cover - runtime environment dependent
                self.state.append_log(f"Start failed: {exc}")
                self._stop_locked()
                self._refresh_state()
                return {
                    "ok": False,
                    "message": f"Failed to start session: {exc}",
                }

            self._refresh_state()
            return {"ok": True, "message": f"Opened terminals for: {', '.join(started)}"}

    def _stop_locked(self) -> None:
        for name, proc in list(self._procs.items()):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if all(proc.poll() is not None for proc in self._procs.values()):
                break
            time.sleep(0.1)
        for name, proc in list(self._procs.items()):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._procs.clear()
        self._logs.clear()

    def stop_all(self) -> dict:
        with self._lock:
            if not self._procs:
                self._refresh_state()
                return {"ok": True, "message": "No managed processes were running."}
            self._stop_locked()
            self.state.append_log("Stopped managed session processes.")
            self._refresh_state()
            return {"ok": True, "message": "Stopped motion, vision, and game processes."}


class RosBridge:
    def __init__(self, state: BridgeState) -> None:
        self.state = state
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        try:
            import rospy
            from std_msgs.msg import Int32, String
            from memory_game.msg import BlockArray, PlayerSelection
        except Exception as exc:  # pragma: no cover - runtime environment dependent
            self.state.update(
                bridge_status="error",
                bridge_message=(
                    "ROS Python packages are not available. Source the catkin workspace "
                    "before running ui/server.py."
                ),
            )
            self.state.append_log(f"Bridge import failed: {exc}")
            return

        self.state.update(
            bridge_status="connecting",
            bridge_message="Connecting to ROS master and subscribing to topics.",
        )

        try:
            rospy.init_node("memory_game_ui_bridge", anonymous=True, disable_signals=True)
        except Exception as exc:  # pragma: no cover - runtime environment dependent
            self.state.update(
                bridge_status="error",
                bridge_message="Could not initialize rospy. Check ROS master and environment.",
            )
            self.state.append_log(f"rospy.init_node failed: {exc}")
            return

        self.state.update(
            bridge_status="connected",
            bridge_message="Live ROS topic bridge is active.",
        )
        self.state.append_log("Subscribed to /score, /game_state, /motion_status, /detected_blocks, /player_selection.")

        rospy.Subscriber("/score", Int32, self._score_cb, queue_size=5)
        rospy.Subscriber("/game_state", String, self._game_state_cb, queue_size=5)
        rospy.Subscriber("/motion_status", String, self._motion_cb, queue_size=5)
        rospy.Subscriber("/detected_blocks", BlockArray, self._blocks_cb, queue_size=5)
        rospy.Subscriber("/player_selection", PlayerSelection, self._selection_cb, queue_size=5)

        rospy.spin()

    def _score_cb(self, msg):
        self.state.update(score=msg.data)

    def _game_state_cb(self, msg):
        self.state.update(game_state=msg.data)

    def _motion_cb(self, msg):
        self.state.update(motion_status=msg.data)

    def _blocks_cb(self, msg):
        blocks = []
        for block in msg.blocks:
            blocks.append({
                "id": int(block.id),
                "color": block.color,
                "position": {
                    "x": round(float(block.position.x), 3),
                    "y": round(float(block.position.y), 3),
                    "z": round(float(block.position.z), 3),
                },
                "confidence": round(float(block.confidence), 3),
            })
        self.state.update(detected_blocks=blocks, detected_block_count=len(blocks))

    def _selection_cb(self, msg):
        self.state.update(player_selection={
            "block_id": int(msg.block_id),
            "color": msg.color,
            "selection_type": msg.selection_type,
            "confidence": round(float(msg.confidence), 3),
        })


class UiHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, directory=None, bridge_state=None, process_manager=None, **kwargs):
        self.bridge_state = bridge_state
        self.process_manager = process_manager
        super().__init__(*args, directory=directory, **kwargs)

    def _write_json(self, payload: dict, status: int = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/api/status":
            self._write_json(self.bridge_state.snapshot())
            return
        return super().do_GET()

    def do_POST(self):
        if self.path == "/api/start":
            result = self.process_manager.start_all()
            self._write_json(result, HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST)
            return
        if self.path == "/api/stop":
            result = self.process_manager.stop_all()
            self._write_json(result)
            return
        self._write_json({"ok": False, "message": "Unknown endpoint"}, HTTPStatus.NOT_FOUND)

    def log_message(self, format, *args):
        return


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve the memory-game UI and bridge simple ROS topics.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    state = BridgeState()
    process_manager = ProcessManager(state)
    bridge = RosBridge(state)
    bridge.start()

    handler = partial(
        UiHandler,
        directory=str(UI_ROOT),
        bridge_state=state,
        process_manager=process_manager,
    )
    server = ThreadingHTTPServer((args.host, args.port), handler)
    print(f"UI server listening on http://{args.host}:{args.port}")
    print("Source your ROS workspace before launching if you want live topics and working Start/End buttons.")
    server.serve_forever()


if __name__ == "__main__":
    main()
