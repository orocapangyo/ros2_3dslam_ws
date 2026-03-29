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
