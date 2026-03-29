# AMR Motion Test GUI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert the CLI-only `amr_motion_test_ui` into a PyQt5 GUI with process launch buttons (Gazebo, SLAM, Motion Control), topic Hz monitoring, and all 5 motion action test tabs.

**Architecture:** PyQt5 `QMainWindow` with 7 tabs (Connection, Status, Spin, Turn, Translate, TranslateRev, YawControl). A `QThread`-based ROS bridge runs `rclpy.spin_once()` for subscriptions, TF, action clients, and topic Hz tracking. A `ProcessManager` handles subprocess lifecycle with process-group-based cleanup. Follows T-AMR reference architecture pattern: `RosBridge(QThread)` → `pyqtSignal` → UI widgets.

**Tech Stack:** Python 3.10, PyQt5, rclpy (ROS2 Humble), tf2_ros, amr_interfaces actions, subprocess

**Reference:** T-AMR `amr_motion_test_ui` (PyQt5 + .ui files + QThread bridge), `acs` (process management), `slam_manager_2d` (multi-process panel)

---

## File Structure

```
amr_motion_test_ui/
├── amr_motion_test_ui/
│   ├── __init__.py                  (unchanged)
│   ├── test_ui_node.py              (unchanged — CLI fallback preserved)
│   ├── process_manager.py           (CREATE — subprocess launch/stop, ~90 lines)
│   ├── ros_bridge.py                (CREATE — QThread ROS bridge + topic Hz, ~220 lines)
│   └── gui_node.py                  (CREATE — PyQt5 main window + all tabs, ~480 lines)
├── launch/
│   ├── test_ui.launch.py            (unchanged — CLI launcher)
│   └── test_gui.launch.py           (CREATE — GUI launcher)
├── package.xml                      (MODIFY — add PyQt5 + rosgraph_msgs deps)
├── setup.py                         (MODIFY — add gui_node entry point + new launch)
├── setup.cfg                        (unchanged)
└── resource/amr_motion_test_ui      (unchanged)
```

**Design decisions:**
- CLI version (`test_ui_node.py`) is preserved as-is for headless/SSH testing
- GUI version is a separate entry point (`gui_node`) — no breaking changes
- No `.ui` files — pure code layout (no Qt Designer dependency)
- Generic tab factory (`_create_action_tab`, `_create_pose_action_tab`) eliminates repetition across 5 action tabs
- `ProcessManager` uses `os.setsid` + `os.killpg(SIGINT)` for clean process group shutdown (T-AMR pattern)

---

## Task 1: Create `process_manager.py`

**Files:**
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/process_manager.py`

- [ ] **Step 1: Create process_manager.py**

```python
"""Subprocess manager for launching ROS2 nodes from the GUI."""

import os
import signal
import subprocess
from typing import Callable, Dict, Optional

# Workspace path for sourcing
_WS = os.path.expanduser('~/Study/ros2_3dslam_ws')

# Launch configurations: key -> {cmd, label}
LAUNCH_CONFIGS = {
    'gazebo': {
        'cmd': 'ros2 launch tm_gazebo gazebo_no_odom.launch.py odom_tf:=false',
        'label': 'Gazebo Simulation',
    },
    'slam': {
        'cmd': 'ros2 launch rtab_map_3d_config rtabmap_3dlidar_rgbd_slam_gazebo.launch.py',
        'label': 'RTAB-Map 3D SLAM',
    },
    'motion_control': {
        'cmd': 'ros2 launch amr_motion_control_2wd motion_control_gazebo.launch.py',
        'label': 'Motion Control',
    },
}


class ProcessManager:
    """Manages ROS2 launch processes with process-group lifecycle control."""

    def __init__(self, log_callback: Optional[Callable[[str], None]] = None):
        self._processes: Dict[str, subprocess.Popen] = {}
        self._log = log_callback or print

    def start(self, key: str) -> bool:
        """Start a process by key. Returns True on success."""
        if key not in LAUNCH_CONFIGS:
            self._log(f'[ERROR] Unknown process: {key}')
            return False

        if self.is_running(key):
            self._log(f'[WARN] {LAUNCH_CONFIGS[key]["label"]} already running')
            return False

        config = LAUNCH_CONFIGS[key]
        shell_cmd = f'source {_WS}/install/setup.bash && {config["cmd"]}'

        try:
            proc = subprocess.Popen(
                ['bash', '-c', shell_cmd],
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._processes[key] = proc
            self._log(f'{config["label"]} started (PID: {proc.pid})')
            return True
        except Exception as e:
            self._log(f'[ERROR] Failed to start {config["label"]}: {e}')
            return False

    def stop(self, key: str) -> bool:
        """Stop a process by key. Returns True if it was running."""
        proc = self._processes.pop(key, None)
        if proc is None or proc.poll() is not None:
            return False

        label = LAUNCH_CONFIGS.get(key, {}).get('label', key)
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
                proc.wait(timeout=3.0)
            self._log(f'{label} stopped')
        except ProcessLookupError:
            pass
        except Exception as e:
            self._log(f'[ERROR] Stop {label}: {e}')
        return True

    def stop_all(self):
        """Stop all managed processes."""
        for key in list(self._processes.keys()):
            self.stop(key)

    def is_running(self, key: str) -> bool:
        """Check if a process is still alive."""
        proc = self._processes.get(key)
        if proc is None:
            return False
        if proc.poll() is not None:
            self._processes.pop(key, None)
            return False
        return True
```

- [ ] **Step 2: Verify file created**

Run: `python3 -c "from amr_motion_test_ui.process_manager import ProcessManager, LAUNCH_CONFIGS; print('OK', len(LAUNCH_CONFIGS))"`
Expected: `OK 3`

Note: This import test requires being in the package directory or having it installed. Skip if not yet built.

- [ ] **Step 3: Commit**

```bash
git add src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/process_manager.py
git commit -m "feat(amr_motion_test_ui): add ProcessManager for subprocess launch/stop

Manages Gazebo, SLAM, and Motion Control processes with os.setsid
process-group lifecycle and SIGINT→SIGKILL escalation."
```

---

## Task 2: Create `ros_bridge.py`

**Files:**
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/ros_bridge.py`

- [ ] **Step 1: Create ros_bridge.py**

```python
"""QThread-based ROS2 bridge for PyQt5 GUI — topic Hz, TF pose, action clients."""

import math
import time
from collections import deque
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from PyQt5.QtCore import QThread, pyqtSignal

from tf2_ros import Buffer, TransformListener

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from amr_interfaces.action import (
    AMRMotionSpin,
    AMRMotionTurn,
    AMRMotionTranslate,
    AMRMotionYawControl,
)

# Topics to monitor: topic -> (msg_class, label, use_sensor_qos)
MONITORED_TOPICS = {
    '/clock':                  (Clock,       'Gazebo Clock',  True),
    '/scan/points':            (PointCloud2, '3D LiDAR',      True),
    '/imu/data':               (Imu,         'IMU',           True),
    '/rtabmap/odom':           (Odometry,    'SLAM Odom',     True),
    '/cmd_vel':                (Twist,       'Cmd Vel',       False),
    '/camera/color/image_raw': (Image,       'RGB Camera',    True),
}

# Action servers: key -> (server_name, action_type)
ACTION_SERVERS = {
    'spin':              ('spin',                   AMRMotionSpin),
    'turn':              ('turn',                   AMRMotionTurn),
    'translate':         ('amr_motion_translate',   AMRMotionTranslate),
    'translate_reverse': ('translate_reverse',       AMRMotionTranslate),
    'yaw_control':       ('amr_motion_yaw_control', AMRMotionYawControl),
}

PHASE_NAMES = {0: 'WAIT', 1: 'ACCEL', 2: 'CRUISE', 3: 'DECEL'}
STATUS_DESC = {
    0: 'success', -1: 'cancelled', -2: 'invalid_param',
    -3: 'timeout', -4: 'tf_fail',
}

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5,
)


class RosBridge(QThread):
    """Runs rclpy in a background thread; emits Qt signals for GUI updates."""

    topic_hz_updated = pyqtSignal(dict)              # {topic_name: hz}
    pose_updated = pyqtSignal(float, float, float)   # x, y, yaw_deg
    action_feedback = pyqtSignal(str, str)            # action_key, text
    action_result = pyqtSignal(str, str, int)         # action_key, text, status
    log_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._running = False
        self._node = None
        self._action_clients: Dict[str, ActionClient] = {}
        self._active_goal_handle = None
        self._cmd_vel_pub = None
        self._topic_stamps: Dict[str, deque] = {
            t: deque(maxlen=50) for t in MONITORED_TOPICS
        }

    # ── Thread lifecycle ──────────────────────────────────────────────

    def run(self):
        rclpy.init()
        self._node = Node('amr_motion_test_gui')
        self._node.declare_parameter('use_sim_time', True)
        self._running = True

        # TF2
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self._node)

        # cmd_vel publisher (for E-STOP)
        self._cmd_vel_pub = self._node.create_publisher(Twist, '/cmd_vel', 10)

        # Topic subscriptions for Hz monitoring
        for topic, (msg_cls, _label, sensor_qos) in MONITORED_TOPICS.items():
            qos = _SENSOR_QOS if sensor_qos else 10
            self._node.create_subscription(
                msg_cls, topic,
                lambda _msg, t=topic: self._topic_stamps[t].append(time.monotonic()),
                qos,
            )

        # Action clients
        for key, (srv_name, action_type) in ACTION_SERVERS.items():
            self._action_clients[key] = ActionClient(
                self._node, action_type, srv_name)

        # Timers
        self._node.create_timer(1.0, self._emit_hz)
        self._node.create_timer(0.2, self._emit_pose)

        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.02)

        self._node.destroy_node()
        rclpy.shutdown()

    def stop(self):
        self._running = False
        self.wait(5000)

    # ── Topic Hz ──────────────────────────────────────────────────────

    def _emit_hz(self):
        now = time.monotonic()
        hz = {}
        for topic, stamps in self._topic_stamps.items():
            recent = [t for t in stamps if now - t < 3.0]
            if len(recent) < 2:
                hz[topic] = 0.0
            else:
                dt = recent[-1] - recent[0]
                hz[topic] = (len(recent) - 1) / dt if dt > 0 else 0.0
        self.topic_hz_updated.emit(hz)

    # ── Pose ──────────────────────────────────────────────────────────

    def _emit_pose(self):
        try:
            tf = self._tf_buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            t = tf.transform.translation
            q = tf.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.pose_updated.emit(t.x, t.y, math.degrees(math.atan2(siny, cosy)))
        except Exception:
            pass

    # ── Action API ────────────────────────────────────────────────────

    def send_goal(self, action_key: str, goal):
        """Send an action goal asynchronously."""
        client = self._action_clients.get(action_key)
        if client is None:
            self.log_message.emit(f'[ERROR] Unknown action: {action_key}')
            return
        if not client.wait_for_server(timeout_sec=3.0):
            self.log_message.emit(f'[ERROR] Server "{action_key}" not available')
            return

        future = client.send_goal_async(
            goal,
            feedback_callback=lambda msg, k=action_key: self._on_feedback(k, msg),
        )
        future.add_done_callback(
            lambda f, k=action_key: self._on_goal_response(k, f))

    def cancel_goal(self):
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
            self.log_message.emit('Cancel requested')

    def publish_zero_cmd_vel(self):
        if self._cmd_vel_pub:
            self._cmd_vel_pub.publish(Twist())

    # ── Action callbacks ──────────────────────────────────────────────

    def _on_goal_response(self, key, future):
        gh = future.result()
        if not gh or not gh.accepted:
            self.log_message.emit(f'[{key}] Goal rejected')
            self.action_result.emit(key, 'Goal rejected', -2)
            return
        self._active_goal_handle = gh
        self.log_message.emit(f'[{key}] Goal accepted')
        gh.get_result_async().add_done_callback(
            lambda f, k=key: self._on_result(k, f))

    def _on_feedback(self, key, msg):
        fb = msg.feedback
        phase = PHASE_NAMES.get(getattr(fb, 'phase', 0), '?')
        if key == 'spin':
            txt = (f'angle={fb.current_angle:.1f}° '
                   f'speed={fb.current_speed:.1f}°/s phase={phase}')
        elif key == 'turn':
            txt = (f'angle={fb.current_angle:.1f}° '
                   f'remain={fb.remaining_angle:.1f}° '
                   f'v={fb.current_linear_speed:.3f}m/s '
                   f'ω={fb.current_angular_speed:.1f}°/s phase={phase}')
        elif key in ('translate', 'translate_reverse'):
            txt = (f'dist={fb.current_distance:.3f}m '
                   f'lat={fb.current_lateral_error:.4f}m '
                   f'hdg={fb.current_heading_error:.1f}° '
                   f'vx={fb.current_vx:.3f}m/s phase={phase}')
        elif key == 'yaw_control':
            txt = (f'dist={fb.current_distance:.3f}m '
                   f'hdg={fb.current_heading_error:.1f}° '
                   f'vx={fb.current_vx:.3f}m/s '
                   f'ω={fb.current_omega:.4f}rad/s phase={phase}')
        else:
            txt = str(fb)
        self.action_feedback.emit(key, txt)

    def _on_result(self, key, future):
        self._active_goal_handle = None
        r = future.result().result
        s = STATUS_DESC.get(getattr(r, 'status', 0), 'unknown')
        if key == 'spin':
            txt = (f'{s}  angle={r.actual_angle:.2f}° '
                   f'time={r.elapsed_time:.2f}s')
        elif key == 'turn':
            txt = (f'{s}  angle={r.actual_angle:.2f}° '
                   f'time={r.elapsed_time:.2f}s')
        else:
            txt = (f'{s}  dist={r.actual_distance:.3f}m '
                   f'lat={r.final_lateral_error:.4f}m '
                   f'hdg={r.final_heading_error:.2f}° '
                   f'time={r.elapsed_time:.2f}s')
        self.action_result.emit(key, txt, getattr(r, 'status', 0))
```

- [ ] **Step 2: Verify syntax**

Run: `python3 -c "import ast; ast.parse(open('src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/ros_bridge.py').read()); print('OK')"`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
git add src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/ros_bridge.py
git commit -m "feat(amr_motion_test_ui): add QThread ROS bridge with topic Hz and action clients

RosBridge runs rclpy in background thread, monitors 6 topics for Hz,
tracks TF pose, and provides async action client API via Qt signals."
```

---

## Task 3: Create `gui_node.py`

**Files:**
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/gui_node.py`

- [ ] **Step 1: Create gui_node.py**

```python
"""PyQt5 GUI for AMR motion control testing with launch management."""

import sys
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QLabel, QPushButton, QTextEdit, QFormLayout,
    QDoubleSpinBox, QCheckBox, QTableWidget, QTableWidgetItem,
    QHeaderView, QGroupBox, QGridLayout,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor

from amr_motion_test_ui.ros_bridge import RosBridge, MONITORED_TOPICS
from amr_motion_test_ui.process_manager import ProcessManager, LAUNCH_CONFIGS

from amr_interfaces.action import (
    AMRMotionSpin,
    AMRMotionTurn,
    AMRMotionTranslate,
    AMRMotionYawControl,
)


class MotionTestGUI(QMainWindow):
    """Main window with 7 tabs: Connection, Status, and 5 action tests."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('AMR Motion Test UI')
        self.resize(880, 720)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # Tab widget
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)

        # Bottom bar: E-STOP + log
        bottom = QHBoxLayout()

        self.estop_btn = QPushButton('E-STOP')
        self.estop_btn.setStyleSheet(
            'background-color:#cc0000; color:white; font-weight:bold; '
            'font-size:16px; padding:10px 20px;')
        self.estop_btn.setFixedWidth(130)
        self.estop_btn.clicked.connect(self._on_estop)
        bottom.addWidget(self.estop_btn)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(120)
        bottom.addWidget(self.log_text)
        layout.addLayout(bottom)

        # Internal state
        self._pose = (0.0, 0.0, 0.0)
        self._fb_labels = {}   # action_key -> QLabel
        self._res_labels = {}  # action_key -> QLabel

        # Build tabs
        self._build_connection_tab()
        self._build_status_tab()
        self._build_spin_tab()
        self._build_turn_tab()
        self._build_translate_tab()
        self._build_translate_reverse_tab()
        self._build_yaw_control_tab()

        # Process manager
        self.proc = ProcessManager(log_callback=self._log)

        # ROS bridge
        self.bridge = RosBridge()
        self.bridge.topic_hz_updated.connect(self._on_hz)
        self.bridge.pose_updated.connect(self._on_pose)
        self.bridge.action_feedback.connect(self._on_fb)
        self.bridge.action_result.connect(self._on_res)
        self.bridge.log_message.connect(self._log)
        self.bridge.start()

        # Poll process status every 2 s
        self._ptimer = QTimer(self)
        self._ptimer.timeout.connect(self._poll_procs)
        self._ptimer.start(2000)

    # ── Connection Tab ────────────────────────────────────────────────

    def _build_connection_tab(self):
        tab = QWidget()
        vbox = QVBoxLayout(tab)

        grp = QGroupBox('Launch / Stop')
        grid = QGridLayout()

        self._proc_labels = {}
        for row, (key, cfg) in enumerate(LAUNCH_CONFIGS.items()):
            grid.addWidget(QLabel(cfg['label']), row, 0)

            btn_start = QPushButton('Start')
            btn_start.clicked.connect(lambda _, k=key: self.proc.start(k))
            grid.addWidget(btn_start, row, 1)

            btn_stop = QPushButton('Stop')
            btn_stop.clicked.connect(lambda _, k=key: self.proc.stop(k))
            grid.addWidget(btn_stop, row, 2)

            status = QLabel('○ Stopped')
            grid.addWidget(status, row, 3)
            self._proc_labels[key] = status

        grp.setLayout(grid)
        vbox.addWidget(grp)

        row_btns = QHBoxLayout()
        btn_all = QPushButton('Start All (순서대로)')
        btn_all.clicked.connect(self._start_all)
        row_btns.addWidget(btn_all)

        btn_stop_all = QPushButton('Stop All')
        btn_stop_all.clicked.connect(self.proc.stop_all)
        row_btns.addWidget(btn_stop_all)
        vbox.addLayout(row_btns)

        vbox.addStretch()
        self.tabs.addTab(tab, 'Connection')

    def _start_all(self):
        self.proc.start('gazebo')
        QTimer.singleShot(8000, lambda: self.proc.start('slam'))
        QTimer.singleShot(15000, lambda: self.proc.start('motion_control'))
        self._log('Start All: Gazebo → SLAM(+8s) → MotionCtrl(+15s)')

    def _poll_procs(self):
        for key, lbl in self._proc_labels.items():
            if self.proc.is_running(key):
                lbl.setText('● Running')
                lbl.setStyleSheet('color:green; font-weight:bold;')
            else:
                lbl.setText('○ Stopped')
                lbl.setStyleSheet('color:gray;')

    # ── Status Tab ────────────────────────────────────────────────────

    def _build_status_tab(self):
        tab = QWidget()
        vbox = QVBoxLayout(tab)

        # Topic table
        topics = list(MONITORED_TOPICS.keys())
        self._topic_table = QTableWidget(len(topics), 3)
        self._topic_table.setHorizontalHeaderLabels(['Topic', 'Hz', 'Status'])
        hdr = self._topic_table.horizontalHeader()
        hdr.setSectionResizeMode(0, QHeaderView.Stretch)
        hdr.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        hdr.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self._topic_table.setEditTriggers(QTableWidget.NoEditTriggers)

        self._topic_rows = {}
        for r, topic in enumerate(topics):
            _label = MONITORED_TOPICS[topic][1]
            self._topic_table.setItem(r, 0, QTableWidgetItem(f'{topic}  ({_label})'))
            self._topic_table.setItem(r, 1, QTableWidgetItem('--'))
            self._topic_table.setItem(r, 2, QTableWidgetItem('○ Idle'))
            self._topic_rows[topic] = r

        vbox.addWidget(self._topic_table)

        # Pose
        grp = QGroupBox('Current Pose  (map → base_footprint)')
        h = QHBoxLayout()
        self._lbl_x = QLabel('X: --')
        self._lbl_y = QLabel('Y: --')
        self._lbl_yaw = QLabel('Yaw: --')
        for w in (self._lbl_x, self._lbl_y, self._lbl_yaw):
            w.setStyleSheet('font-size:14px; font-weight:bold;')
            h.addWidget(w)
        grp.setLayout(h)
        vbox.addWidget(grp)

        vbox.addStretch()
        self.tabs.addTab(tab, 'Status')

    def _on_hz(self, hz_map):
        for topic, hz in hz_map.items():
            r = self._topic_rows.get(topic)
            if r is None:
                continue
            self._topic_table.item(r, 1).setText(f'{hz:.1f}')
            if hz > 0.5:
                self._topic_table.item(r, 2).setText('● Active')
                self._topic_table.item(r, 2).setForeground(QColor('green'))
            else:
                self._topic_table.item(r, 2).setText('○ Idle')
                self._topic_table.item(r, 2).setForeground(QColor('gray'))

    def _on_pose(self, x, y, yaw):
        self._pose = (x, y, yaw)
        self._lbl_x.setText(f'X: {x:.3f} m')
        self._lbl_y.setText(f'Y: {y:.3f} m')
        self._lbl_yaw.setText(f'Yaw: {yaw:.1f}°')

    # ── Generic tab builders ──────────────────────────────────────────

    def _make_action_tab(self, title, key, fields, goal_fn):
        """Simple action tab (Spin, Turn) with numeric inputs only.

        fields: [(label, param, default, lo, hi, suffix), ...]
        goal_fn(widgets) -> goal msg
        """
        tab = QWidget()
        vbox = QVBoxLayout(tab)
        form = QFormLayout()
        widgets = {}

        for label, param, default, lo, hi, suffix in fields:
            sb = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setDecimals(2)
            sb.setValue(default)
            sb.setSuffix(f' {suffix}')
            widgets[param] = sb
            form.addRow(f'{label}:', sb)

        vbox.addLayout(form)
        self._add_action_buttons(vbox, key, widgets, goal_fn)
        vbox.addStretch()
        self.tabs.addTab(tab, title)

    def _make_pose_action_tab(self, title, key, extra_fields, goal_fn,
                              has_next_checkbox=False):
        """Action tab with start/end XY + 'Use Current Pose' button.

        extra_fields: [(label, param, default, lo, hi, suffix), ...]
        """
        tab = QWidget()
        vbox = QVBoxLayout(tab)
        widgets = {}

        # Position group
        grp = QGroupBox('Position (map frame, meters)')
        g = QGridLayout()
        for i, (lbl, param, default) in enumerate([
            ('Start X', 'start_x', 0.0), ('Start Y', 'start_y', 0.0),
            ('End X', 'end_x', 1.0),     ('End Y', 'end_y', 0.0),
        ]):
            row, col = divmod(i, 2)
            g.addWidget(QLabel(lbl + ':'), row, col * 2)
            sb = QDoubleSpinBox()
            sb.setRange(-100.0, 100.0)
            sb.setDecimals(3)
            sb.setValue(default)
            sb.setSuffix(' m')
            widgets[param] = sb
            g.addWidget(sb, row, col * 2 + 1)

        btn_pose = QPushButton('← Current Pose')
        btn_pose.clicked.connect(lambda: self._fill_pose(widgets))
        g.addWidget(btn_pose, 0, 4, 2, 1)
        grp.setLayout(g)
        vbox.addWidget(grp)

        # Extra numeric fields
        form = QFormLayout()
        for label, param, default, lo, hi, suffix in extra_fields:
            sb = QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setDecimals(3)
            sb.setValue(default)
            sb.setSuffix(f' {suffix}')
            widgets[param] = sb
            form.addRow(f'{label}:', sb)
        vbox.addLayout(form)

        # Optional has_next checkbox
        if has_next_checkbox:
            cb = QCheckBox('has_next (다음 세그먼트 존재 → 감속 생략)')
            cb.setChecked(False)
            widgets['has_next'] = cb
            vbox.addWidget(cb)

        self._add_action_buttons(vbox, key, widgets, goal_fn)
        vbox.addStretch()
        self.tabs.addTab(tab, title)

    def _add_action_buttons(self, layout, key, widgets, goal_fn):
        """Add Send/Cancel buttons + feedback/result labels."""
        h = QHBoxLayout()
        btn_send = QPushButton('Send Goal')
        btn_send.setStyleSheet(
            'background-color:#2196F3; color:white; padding:8px 16px;')
        btn_send.clicked.connect(
            lambda: self._send(key, widgets, goal_fn))
        h.addWidget(btn_send)

        btn_cancel = QPushButton('Cancel')
        btn_cancel.clicked.connect(self.bridge.cancel_goal)
        h.addWidget(btn_cancel)
        layout.addLayout(h)

        fb = QLabel('Feedback: --')
        fb.setWordWrap(True)
        layout.addWidget(fb)
        self._fb_labels[key] = fb

        res = QLabel('Result: --')
        res.setWordWrap(True)
        layout.addWidget(res)
        self._res_labels[key] = res

    def _fill_pose(self, widgets):
        x, y, _ = self._pose
        widgets['start_x'].setValue(x)
        widgets['start_y'].setValue(y)

    def _send(self, key, widgets, goal_fn):
        self._fb_labels[key].setText('Feedback: sending...')
        self._fb_labels[key].setStyleSheet('')
        self._res_labels[key].setText('Result: --')
        self._res_labels[key].setStyleSheet('')
        goal = goal_fn(widgets)
        self.bridge.send_goal(key, goal)

    def _on_fb(self, key, text):
        lbl = self._fb_labels.get(key)
        if lbl:
            lbl.setText(f'Feedback: {text}')

    def _on_res(self, key, text, status):
        lbl = self._res_labels.get(key)
        if lbl:
            color = 'green' if status == 0 else 'red'
            lbl.setText(f'Result: {text}')
            lbl.setStyleSheet(f'color:{color}; font-weight:bold;')

    # ── 5 Action Tabs ─────────────────────────────────────────────────

    def _build_spin_tab(self):
        def goal(w):
            g = AMRMotionSpin.Goal()
            g.target_angle = w['target_angle'].value()
            g.max_angular_speed = w['max_angular_speed'].value()
            g.angular_acceleration = w['angular_acceleration'].value()
            return g

        self._make_action_tab('Spin', 'spin', [
            ('Target Angle',  'target_angle',        90.0, -360, 360, 'deg'),
            ('Max Speed',     'max_angular_speed',   30.0,  1,  57.3, 'deg/s'),
            ('Acceleration',  'angular_acceleration', 30.0, 1,  180,  'deg/s²'),
        ], goal)

    def _build_turn_tab(self):
        def goal(w):
            g = AMRMotionTurn.Goal()
            g.target_angle = w['target_angle'].value()
            g.turn_radius = w['turn_radius'].value()
            g.max_linear_speed = w['max_linear_speed'].value()
            g.accel_angle = w['accel_angle'].value()
            return g

        self._make_action_tab('Turn', 'turn', [
            ('Target Angle', 'target_angle',    90.0, -360, 360, 'deg'),
            ('Turn Radius',  'turn_radius',      0.5,  0.1,  5,  'm'),
            ('Max Speed',    'max_linear_speed',  0.2, 0.05, 0.4, 'm/s'),
            ('Accel Angle',  'accel_angle',      20.0,  1,   90,  'deg'),
        ], goal)

    def _build_translate_tab(self):
        def goal(w):
            g = AMRMotionTranslate.Goal()
            g.start_x = w['start_x'].value()
            g.start_y = w['start_y'].value()
            g.end_x = w['end_x'].value()
            g.end_y = w['end_y'].value()
            g.max_linear_speed = w['max_linear_speed'].value()
            g.acceleration = w['acceleration'].value()
            g.exit_speed = w['exit_speed'].value()
            g.has_next = w['has_next'].isChecked()
            return g

        self._make_pose_action_tab('Translate', 'translate', [
            ('Max Speed',    'max_linear_speed', 0.2, 0.05, 0.4, 'm/s'),
            ('Acceleration', 'acceleration',     0.2, 0.05, 1.5, 'm/s²'),
            ('Exit Speed',   'exit_speed',       0.0, 0.0,  0.4, 'm/s'),
        ], goal, has_next_checkbox=True)

    def _build_translate_reverse_tab(self):
        def goal(w):
            g = AMRMotionTranslate.Goal()
            g.start_x = w['start_x'].value()
            g.start_y = w['start_y'].value()
            g.end_x = w['end_x'].value()
            g.end_y = w['end_y'].value()
            g.max_linear_speed = -abs(w['max_linear_speed'].value())
            g.acceleration = w['acceleration'].value()
            g.exit_speed = w['exit_speed'].value()
            g.has_next = w['has_next'].isChecked()
            return g

        self._make_pose_action_tab('TranslateRev', 'translate_reverse', [
            ('Max Speed',    'max_linear_speed', 0.2, 0.05, 0.4, 'm/s'),
            ('Acceleration', 'acceleration',     0.2, 0.05, 1.5, 'm/s²'),
            ('Exit Speed',   'exit_speed',       0.0, 0.0,  0.4, 'm/s'),
        ], goal, has_next_checkbox=True)

    def _build_yaw_control_tab(self):
        def goal(w):
            g = AMRMotionYawControl.Goal()
            g.start_x = w['start_x'].value()
            g.start_y = w['start_y'].value()
            g.end_x = w['end_x'].value()
            g.end_y = w['end_y'].value()
            g.max_linear_speed = w['max_linear_speed'].value()
            g.acceleration = w['acceleration'].value()
            return g

        self._make_pose_action_tab('YawControl', 'yaw_control', [
            ('Max Speed',    'max_linear_speed', 0.2, 0.05, 0.4, 'm/s'),
            ('Acceleration', 'acceleration',     0.2, 0.05, 1.5, 'm/s²'),
        ], goal)

    # ── E-STOP & Logging ──────────────────────────────────────────────

    def _on_estop(self):
        self.bridge.cancel_goal()
        self.bridge.publish_zero_cmd_vel()
        self._log('[E-STOP] cancelled + zero cmd_vel')

    def _log(self, msg):
        ts = datetime.now().strftime('%H:%M:%S')
        self.log_text.append(f'[{ts}] {msg}')

    def closeEvent(self, event):
        self.bridge.stop()
        self.proc.stop_all()
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = MotionTestGUI()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Verify syntax**

Run: `python3 -c "import ast; ast.parse(open('src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/gui_node.py').read()); print('OK')"`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
git add src/Control/AMR-Motion-Control/amr_motion_test_ui/amr_motion_test_ui/gui_node.py
git commit -m "feat(amr_motion_test_ui): add PyQt5 GUI with launch buttons and topic monitor

7-tab window: Connection (launch Gazebo/SLAM/MotionCtrl), Status (topic Hz
table + pose), and 5 action test tabs (Spin/Turn/Translate/TranslateRev/
YawControl). E-STOP button cancels active goals and publishes zero cmd_vel."
```

---

## Task 4: Update package configuration

**Files:**
- Modify: `src/Control/AMR-Motion-Control/amr_motion_test_ui/package.xml`
- Modify: `src/Control/AMR-Motion-Control/amr_motion_test_ui/setup.py`

- [ ] **Step 1: Update package.xml — add PyQt5 and rosgraph_msgs dependencies**

Add after the existing `<exec_depend>tf2_ros_py</exec_depend>` line:

```xml
  <exec_depend>rosgraph_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>python3-pyqt5</exec_depend>
```

Also update the description:

```xml
  <description>CLI and GUI test UI for AMR motion control action servers</description>
```

Full resulting `package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>amr_motion_test_ui</name>
  <version>0.1.0</version>
  <description>CLI and GUI test UI for AMR motion control action servers</description>
  <maintainer email="user@example.com">amap</maintainer>
  <license>BSD-3-Clause</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>amr_interfaces</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>tf2_ros_py</exec_depend>
  <exec_depend>rosgraph_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>python3-pyqt5</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 2: Update setup.py — add gui_node entry point and new launch file**

```python
from setuptools import find_packages, setup

package_name = 'amr_motion_test_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/test_ui.launch.py',
            'launch/test_gui.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'test_ui_node = amr_motion_test_ui.test_ui_node:main',
            'gui_node = amr_motion_test_ui.gui_node:main',
        ],
    },
)
```

- [ ] **Step 3: Commit**

```bash
git add src/Control/AMR-Motion-Control/amr_motion_test_ui/package.xml \
        src/Control/AMR-Motion-Control/amr_motion_test_ui/setup.py
git commit -m "feat(amr_motion_test_ui): add PyQt5 deps and gui_node entry point

Adds rosgraph_msgs, sensor_msgs, nav_msgs, python3-pyqt5 dependencies
and registers gui_node console_scripts entry point."
```

---

## Task 5: Create GUI launch file

**Files:**
- Create: `src/Control/AMR-Motion-Control/amr_motion_test_ui/launch/test_gui.launch.py`

- [ ] **Step 1: Create test_gui.launch.py**

```python
"""Launch the PyQt5 GUI version of the motion test UI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='amr_motion_test_ui',
            executable='gui_node',
            name='amr_motion_test_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
```

- [ ] **Step 2: Commit**

```bash
git add src/Control/AMR-Motion-Control/amr_motion_test_ui/launch/test_gui.launch.py
git commit -m "feat(amr_motion_test_ui): add launch file for GUI version"
```

---

## Task 6: Build and verify

- [ ] **Step 1: Install PyQt5 if not present**

Run: `pip3 install PyQt5 2>/dev/null; python3 -c "from PyQt5.QtWidgets import QApplication; print('PyQt5 OK')"`
Expected: `PyQt5 OK`

- [ ] **Step 2: Build the package**

Run:
```bash
cd ~/Study/ros2_3dslam_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select amr_motion_test_ui
```
Expected: `1 package finished` with no errors

- [ ] **Step 3: Verify entry points registered**

Run:
```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 run amr_motion_test_ui gui_node --help 2>&1 | head -5
```

This should not error with "No executable found". If it shows any output or usage, the entry point is registered.

- [ ] **Step 4: Verify launch file discoverable**

Run:
```bash
ros2 launch amr_motion_test_ui test_gui.launch.py --show-args
```
Expected: Shows `use_sim_time` argument

- [ ] **Step 5: Commit (if any fixes needed)**

Only if build required fixes. Otherwise skip.

---

## Task 7: Manual integration test

- [ ] **Step 1: Launch GUI standalone (no Gazebo)**

Run:
```bash
source ~/Study/ros2_3dslam_ws/install/setup.bash
ros2 run amr_motion_test_ui gui_node
```
Expected: PyQt5 window opens with 7 tabs, all topics show "○ Idle"

- [ ] **Step 2: Test Connection tab**

1. Click "Start" next to Gazebo → status should change to "● Running"
2. Wait ~8s, click "Start" next to SLAM → status shows "● Running"
3. Wait ~7s, click "Start" next to Motion Control → "● Running"
4. Switch to Status tab → `/clock`, `/scan/points`, `/imu/data` should show Hz > 0 and "● Active"

- [ ] **Step 3: Test action tab**

1. Switch to Spin tab
2. Set target_angle=90, leave defaults
3. Click "Send Goal"
4. Observe feedback updates and final result

- [ ] **Step 4: Test E-STOP**

1. While an action is running, click E-STOP
2. Feedback should stop, result should show "cancelled"
3. Robot should stop moving

- [ ] **Step 5: Test Stop All**

1. Switch to Connection tab
2. Click "Stop All"
3. All processes should show "○ Stopped"
4. Status tab topics should transition to "○ Idle"

---

## Execution Summary

| Task | File | Lines | Description |
|------|------|-------|-------------|
| 1 | `process_manager.py` | ~90 | Subprocess launch/stop with process group lifecycle |
| 2 | `ros_bridge.py` | ~200 | QThread ROS bridge: topic Hz, TF pose, action clients |
| 3 | `gui_node.py` | ~370 | PyQt5 main window: 7 tabs, E-STOP, log area |
| 4 | `package.xml` + `setup.py` | ~40 | Dependencies, entry points |
| 5 | `test_gui.launch.py` | ~20 | ROS2 launch configuration |
| 6 | Build verification | -- | colcon build + entry point check |
| 7 | Integration test | -- | Manual GUI walkthrough |

**Total new code:** ~720 lines across 4 new files
**Modifications:** 2 existing files (package.xml, setup.py)
**Existing CLI preserved:** `test_ui_node.py` unchanged — no breaking changes

## Run Commands (After Build)

```bash
# Option A: GUI with manual launch
ros2 run amr_motion_test_ui gui_node

# Option B: GUI via launch file
ros2 launch amr_motion_test_ui test_gui.launch.py

# Option C: Original CLI (unchanged)
ros2 run amr_motion_test_ui test_ui_node
```
