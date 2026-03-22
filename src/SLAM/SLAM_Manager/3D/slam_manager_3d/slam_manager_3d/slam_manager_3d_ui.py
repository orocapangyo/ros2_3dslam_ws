#!/usr/bin/env python3
"""
SLAM Manager 3D UI - PyQt5 UI class for 3D SLAM Manager

This module handles:
- Tab-based UI similar to 2D SLAM Manager
- 6-DOF position display (x, y, z, roll, pitch, yaw)
- Launch file control and button state management
- ROS Bag playback control
"""

import os
import signal
import time
import subprocess
import math
from pathlib import Path
from datetime import datetime

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer, QDateTime
from PyQt5.QtWidgets import QFileDialog, QMessageBox
from ament_index_python.packages import get_package_share_directory


def get_workspace_path():
    """Detect workspace path from slam_manager_3d package installation"""
    try:
        pkg_path = get_package_share_directory('slam_manager_3d')
        workspace = Path(pkg_path).parents[3]
        return workspace
    except Exception:
        return Path.home()


class SlamManager3DUI(QtWidgets.QMainWindow):
    """PyQt5 Main Window for 3D SLAM Manager"""

    def __init__(self):
        super().__init__()

        # Load UI file
        ui_file = Path(__file__).parent / 'ui' / 'slam_manager_3d.ui'
        uic.loadUi(ui_file, self)

        # ROS2 node (will be set later)
        self.node = None

        # Workspace path (auto-detected from package installation)
        self.workspace_path = get_workspace_path()

        # ROS Bag state
        self.bag_process = None
        self.bag_paused = False
        self.bag_duration = 0.0
        self.bag_start_time = None

        # Connect buttons - Data Source tab
        self.btnStartOrbbec.clicked.connect(self.on_start_orbbec)
        self.btnStopOrbbec.clicked.connect(self.on_stop_orbbec)
        self.btnBrowseBag.clicked.connect(self.on_browse_bag)
        self.btnPlayBag.clicked.connect(self.on_play_bag)
        self.btnPauseBag.clicked.connect(self.on_pause_bag)
        self.btnStopBag.clicked.connect(self.on_stop_bag)

        # Connect buttons - RTAB-Map RGB-D tab
        self.btnStartRgbdMapping.clicked.connect(self.on_start_rgbd_mapping)
        self.btnStopRgbdMapping.clicked.connect(self.on_stop_rgbd_mapping)
        self.btnSaveRgbdMap.clicked.connect(self.on_save_rgbd_map)
        self.btnBrowseRgbdMap.clicked.connect(self.on_browse_rgbd_map)
        self.btnStartRgbdLoc.clicked.connect(self.on_start_rgbd_loc)
        self.btnStopRgbdLoc.clicked.connect(self.on_stop_rgbd_loc)

        # Connect buttons - RTAB-Map RGB-D + LiDAR tab
        self.btnStartRgbdLidarMapping.clicked.connect(self.on_start_rgbd_lidar_mapping)
        self.btnStopRgbdLidarMapping.clicked.connect(self.on_stop_rgbd_lidar_mapping)
        self.btnSaveRgbdLidarMap.clicked.connect(self.on_save_rgbd_lidar_map)
        self.btnBrowseRgbdLidarMap.clicked.connect(self.on_browse_rgbd_lidar_map)
        self.btnStartRgbdLidarLoc.clicked.connect(self.on_start_rgbd_lidar_loc)
        self.btnStopRgbdLidarLoc.clicked.connect(self.on_stop_rgbd_lidar_loc)
        self.btnScanTopics.clicked.connect(self.on_scan_lidar_topics)

        # Connect buttons - LIO-SAM tab
        self.btnStartLioSamMapping.clicked.connect(self.on_start_liosam_mapping)
        self.btnStopLioSamMapping.clicked.connect(self.on_stop_liosam_mapping)
        self.btnBrowseLioSamMap.clicked.connect(self.on_browse_liosam_map)
        self.btnStartLioSamLoc.clicked.connect(self.on_start_liosam_loc)
        self.btnStopLioSamLoc.clicked.connect(self.on_stop_liosam_loc)

        # Connect buttons - RTAB-Map 3D LiDAR tab
        self.btnStart3dLidarMapping.clicked.connect(self.on_start_3dlidar_mapping)
        self.btnStop3dLidarMapping.clicked.connect(self.on_stop_3dlidar_mapping)
        self.btnSave3dLidarMap.clicked.connect(self.on_save_3dlidar_map)
        self.btnBrowse3dLidarMap.clicked.connect(self.on_browse_3dlidar_map)
        self.btnStart3dLidarLoc.clicked.connect(self.on_start_3dlidar_loc)
        self.btnStop3dLidarLoc.clicked.connect(self.on_stop_3dlidar_loc)

        # Connect common buttons
        self.btnStopAll.clicked.connect(self.on_stop_all)

        # Status timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_button_states)
        self.status_timer.start(500)

        # Set default map paths
        self.txtRgbdMapPath.setText(
            str(self.workspace_path / "maps" / "rtabmap_3d") + "/")
        self.txtRgbdLidarMapPath.setText(
            str(self.workspace_path / "maps" / "rtabmap_3d") + "/")
        self.txtLioSamMapPath.setText(
            str(self.workspace_path / "maps" / "lio_sam" / "GlobalMap.pcd"))
        self.txt3dLidarMapPath.setText(
            str(self.workspace_path / "maps" / "rtabmap_3d") + "/")

        self.log("SLAM Manager 3D UI Ready")

    def set_node(self, node):
        """Set the ROS2 node"""
        self.node = node
        self.auto_detect_launch_files()
        self.update_button_states()

    def _is_package_available(self, package_name):
        """Check if ROS2 package is installed"""
        try:
            result = subprocess.run(
                ['ros2', 'pkg', 'prefix', package_name],
                capture_output=True, text=True, timeout=5
            )
            return result.returncode == 0
        except Exception:
            return False

    def _is_gazebo_running(self):
        """Check if Gazebo simulation is running.

        Detection methods:
        1. Check for 'ign gazebo' or 'gz sim' process
        2. Check for /clock topic (Gazebo publishes simulation time)

        Returns:
            bool: True if Gazebo is detected, False otherwise
        """
        try:
            # Method 1: Check for Ignition Gazebo process
            result = subprocess.run(
                ['pgrep', '-f', 'ign gazebo'],
                capture_output=True,
                timeout=2
            )
            if result.returncode == 0:
                return True

            # Method 2: Check for Gazebo Sim (newer versions)
            result = subprocess.run(
                ['pgrep', '-f', 'gz sim'],
                capture_output=True,
                timeout=2
            )
            if result.returncode == 0:
                return True

            # Method 3: Check /clock topic (Gazebo publishes this)
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0 and '/clock' in result.stdout:
                return True

        except Exception:
            pass

        return False

    def auto_detect_launch_files(self):
        """Auto-detect launch files by checking installed ROS2 packages.

        Automatically selects Gazebo or real robot launch files based on
        whether Gazebo simulation is running.
        """
        # Check if Gazebo is running
        is_gazebo = self._is_gazebo_running()

        if is_gazebo:
            self.log("Gazebo detected - using simulation launch files")
            # Gazebo simulation launch files (*_gazebo.launch.py)
            launch_config = {
                'orbbec_camera': ('orbbec_camera', 'astra_pro.launch.py'),  # Not used in Gazebo
                'rgbd_mapping': ('rtab_map_3d_config', 'rtabmap_rgbd_gazebo.launch.py'),
                'rgbd_loc': ('rtab_map_3d_config', 'rtabmap_rgbd_loc_gazebo.launch.py'),
                'rgbd_lidar_mapping': ('rtab_map_3d_config', 'rtabmap_2dlidar_rgbd_gazebo.launch.py'),
                'rgbd_lidar_loc': ('rtab_map_3d_config', 'rtabmap_2dlidar_rgbd_loc_gazebo.launch.py'),
                'lio_sam_mapping': ('livox_lio_sam', 'run_slam_gazebo.launch.py'),
                'lio_sam_loc': ('livox_lio_sam', 'run_localization_gazebo.launch.py'),
                'rtabmap_3dlidar_mapping': ('rtab_map_3d_config', 'rtabmap_3dlidar_only_slam_gazebo.launch.py'),
                'rtabmap_3dlidar_loc': ('rtab_map_3d_config', 'rtabmap_3dlidar_only_localization_gazebo.launch.py'),
            }
        else:
            self.log("Real robot mode - using real robot launch files")
            # Real robot launch files
            launch_config = {
                'orbbec_camera': ('orbbec_camera', 'astra_pro.launch.py'),
                'rgbd_mapping': ('rtab_map_3d_config', 'rtabmap_rgbd.launch.py'),
                'rgbd_loc': ('rtab_map_3d_config', 'rtabmap_rgbd_loc.launch.py'),
                'rgbd_lidar_mapping': ('rtab_map_3d_config', 'rtabmap_2dlidar_rgbd.launch.py'),
                'rgbd_lidar_loc': ('rtab_map_3d_config', 'rtabmap_2dlidar_rgbd_loc.launch.py'),
                'lio_sam_mapping': ('livox_lio_sam', 'run_slam.launch.py'),
                'lio_sam_loc': ('livox_lio_sam', 'run_localization.launch.py'),
            }

        # Check each package and register launch files
        checked_packages = set()
        for key, (package_name, launch_file) in launch_config.items():
            if package_name not in checked_packages:
                if self._is_package_available(package_name):
                    self.log(f"Found package: {package_name}")
                    checked_packages.add(package_name)
                else:
                    self.log(f"Package not found: {package_name}")

            if package_name in checked_packages:
                self.node.launch_files[key] = (package_name, launch_file)
                self.log(f"  - Registered: {key} -> {launch_file}")

    def log(self, message):
        """Add message to log"""
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        self.txtLog.append(f"[{timestamp}] {message}")

    def update_rgbd_position(self, x, y, z, roll, pitch, yaw):
        """Update RTAB-Map RGB-D position labels (6-DOF)"""
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        self.lblRgbdPosX.setText(f"X: {x:.3f}")
        self.lblRgbdPosY.setText(f"Y: {y:.3f}")
        self.lblRgbdPosZ.setText(f"Z: {z:.3f}")
        self.lblRgbdRoll.setText(f"Roll: {roll_deg:.1f}")
        self.lblRgbdPitch.setText(f"Pitch: {pitch_deg:.1f}")
        self.lblRgbdYaw.setText(f"Yaw: {yaw_deg:.1f}")

    def update_rgbd_lidar_position(self, x, y, z, roll, pitch, yaw):
        """Update RTAB-Map RGB-D + LiDAR position labels (6-DOF)"""
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        self.lblRgbdLidarPosX.setText(f"X: {x:.3f}")
        self.lblRgbdLidarPosY.setText(f"Y: {y:.3f}")
        self.lblRgbdLidarPosZ.setText(f"Z: {z:.3f}")
        self.lblRgbdLidarRoll.setText(f"Roll: {roll_deg:.1f}")
        self.lblRgbdLidarPitch.setText(f"Pitch: {pitch_deg:.1f}")
        self.lblRgbdLidarYaw.setText(f"Yaw: {yaw_deg:.1f}")

    def update_3dlidar_position(self, x, y, z, roll, pitch, yaw):
        """Update RTAB-Map 3D LiDAR position labels (6-DOF)"""
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        self.lbl3dLidarPosX.setText(f"X: {x:.3f}")
        self.lbl3dLidarPosY.setText(f"Y: {y:.3f}")
        self.lbl3dLidarPosZ.setText(f"Z: {z:.3f}")
        self.lbl3dLidarRoll.setText(f"Roll: {roll_deg:.1f}")
        self.lbl3dLidarPitch.setText(f"Pitch: {pitch_deg:.1f}")
        self.lbl3dLidarYaw.setText(f"Yaw: {yaw_deg:.1f}")

    def update_liosam_position(self, x, y, z, roll, pitch, yaw):
        """Update LIO-SAM position labels (6-DOF)"""
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        self.lblLioSamPosX.setText(f"X: {x:.3f}")
        self.lblLioSamPosY.setText(f"Y: {y:.3f}")
        self.lblLioSamPosZ.setText(f"Z: {z:.3f}")
        self.lblLioSamRoll.setText(f"Roll: {roll_deg:.1f}")
        self.lblLioSamPitch.setText(f"Pitch: {pitch_deg:.1f}")
        self.lblLioSamYaw.setText(f"Yaw: {yaw_deg:.1f}")

    def update_rtabmap3d_position(self, x, y, z, roll, pitch, yaw):
        """Update position for all RTAB-Map tabs (same /rtabmap/odom topic)"""
        self.update_rgbd_position(x, y, z, roll, pitch, yaw)
        self.update_rgbd_lidar_position(x, y, z, roll, pitch, yaw)
        self.update_3dlidar_position(x, y, z, roll, pitch, yaw)

    # ==================== Data Source Tab ====================

    def on_start_orbbec(self):
        """Start Orbbec camera"""
        launch_info = self.node.launch_files.get('orbbec_camera')
        if launch_info:
            pkg, launch = launch_info
            if self.node.start_launch_file('orbbec_camera', pkg, launch):
                self.log("Orbbec camera started")
                self.update_button_states()
        else:
            self.log("Orbbec camera driver not installed!")
            QMessageBox.warning(
                self, "Error",
                "Orbbec camera driver not found!\n\n"
                "Note: In Gazebo simulation, camera runs externally.\n"
                "For real robot, install OrbbecSDK_ROS2."
            )

    def on_stop_orbbec(self):
        """Stop Orbbec camera"""
        if self.node.stop_launch_file('orbbec_camera'):
            self.log("Orbbec camera stopped")
            self.update_button_states()

    def on_browse_bag(self):
        """Browse for ROS bag file"""
        default_path = self.workspace_path / 'bag'
        if not default_path.exists():
            default_path = Path.home()

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select ROS Bag",
            str(default_path),
            "ROS Bag (*.db3 *.mcap);;All Files (*)"
        )
        if file_path:
            if file_path.endswith('.db3') or file_path.endswith('.mcap'):
                bag_dir = str(Path(file_path).parent)
                self.txtBagFile.setText(bag_dir)
            else:
                self.txtBagFile.setText(file_path)

    def _get_bag_info(self, bag_path):
        """Get bag duration using ros2 bag info"""
        try:
            result = subprocess.run(
                ['ros2', 'bag', 'info', bag_path],
                capture_output=True,
                text=True,
                timeout=10
            )
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'Duration:' in line:
                        duration_str = line.split('Duration:')[1].strip()
                        total_seconds = 0.0
                        if 'm' in duration_str:
                            parts = duration_str.replace('s', '').split('m')
                            total_seconds = float(parts[0]) * 60 + float(parts[1].strip())
                        else:
                            total_seconds = float(duration_str.replace('s', ''))
                        return total_seconds
        except Exception as e:
            self.log(f"Failed to get bag info: {e}")
        return 0.0

    def on_play_bag(self):
        """Play ROS bag"""
        bag_path = self.txtBagFile.text()
        if not bag_path:
            self.log("Please select a ROS bag file!")
            QMessageBox.warning(self, "Error", "Please select a ROS bag file!")
            return

        if not os.path.exists(bag_path):
            self.log(f"Bag path not found: {bag_path}")
            QMessageBox.warning(self, "Error", f"Bag path not found:\n{bag_path}")
            return

        try:
            self.bag_duration = self._get_bag_info(bag_path)
            self.log(f"Bag duration: {self.bag_duration:.1f}s")

            cmd = ['ros2', 'bag', 'play', bag_path]

            if self.chkBagLoop.isChecked():
                cmd.append('--loop')

            rate = self.spinBagRate.value()
            if rate != 1.0:
                cmd.extend(['--rate', str(rate)])

            if self.chkBagClock.isChecked():
                cmd.append('--clock')

            env = os.environ.copy()

            self.bag_process = subprocess.Popen(
                cmd,
                env=env,
                stdin=subprocess.PIPE,
                preexec_fn=os.setpgrp
            )

            self.bag_paused = False
            self.bag_start_time = QDateTime.currentDateTime()
            self.progressBag.setValue(0)
            self.chkUseSimTime.setChecked(True)
            self.log(f"Playing bag: {bag_path}")
            self.update_button_states()

        except Exception as e:
            self.log(f"Failed to play bag: {str(e)}")
            QMessageBox.critical(self, "Error", f"Failed to play bag:\n{str(e)}")

    def on_pause_bag(self):
        """Pause/Resume ROS bag playback"""
        if self.bag_process is None:
            return

        try:
            if self.bag_paused:
                os.kill(self.bag_process.pid, signal.SIGCONT)
                self.bag_paused = False
                self.log("Bag playback resumed")
            else:
                os.kill(self.bag_process.pid, signal.SIGSTOP)
                self.bag_paused = True
                self.log("Bag playback paused")

            self._update_bag_button_states()

        except Exception as e:
            self.log(f"Failed to pause/resume bag: {str(e)}")

    def on_stop_bag(self):
        """Stop ROS bag playback"""
        if self.bag_process is None:
            return

        try:
            if self.bag_paused:
                os.kill(self.bag_process.pid, signal.SIGCONT)

            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
            time.sleep(1)

            if self.bag_process.poll() is None:
                os.killpg(os.getpgid(self.bag_process.pid), signal.SIGKILL)

            self.bag_process = None
            self.bag_paused = False
            self.bag_start_time = None
            self.log("Bag playback stopped")
            self.update_button_states()

        except Exception as e:
            self.log(f"Failed to stop bag: {str(e)}")
            self.bag_process = None
            self.bag_paused = False
            self.bag_start_time = None
            self._update_bag_button_states()

    # ==================== RTAB-Map RGB-D Tab ====================

    def on_start_rgbd_mapping(self):
        """Start RTAB-Map RGB-D mapping"""
        launch_info = self.node.launch_files.get('rgbd_mapping')
        if launch_info:
            pkg, launch = launch_info
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')
            else:
                extra_args.append('rviz:=false')

            if self.node.start_launch_file('rgbd_mapping', pkg, launch,
                                           extra_args=' '.join(extra_args) if extra_args else None):
                self.log("RTAB-Map RGB-D Mapping started")
                self.update_button_states()
        else:
            self.log("RTAB-Map RGB-D mapping launch file not found!")
            QMessageBox.warning(self, "Error", "RTAB-Map RGB-D mapping launch file not found!")

    def on_stop_rgbd_mapping(self):
        """Stop RTAB-Map RGB-D mapping"""
        if self.node.stop_launch_file('rgbd_mapping'):
            self.log("RTAB-Map RGB-D Mapping stopped")
            self.update_button_states()

    def on_save_rgbd_map(self):
        """Save RTAB-Map RGB-D map"""
        self._save_rtabmap_db("rgbd")

    def on_browse_rgbd_map(self):
        """Browse for RTAB-Map RGB-D map file"""
        default_path = str(self.workspace_path / "maps" / "rtabmap_3d")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select RTAB-Map Database (.db)",
            default_path,
            "Database Files (*.db);;All Files (*)"
        )
        if file_path:
            self.txtRgbdMapPath.setText(file_path)
            self.log(f"RGB-D map selected: {file_path}")

    def on_start_rgbd_loc(self):
        """Start RTAB-Map RGB-D localization"""
        launch_info = self.node.launch_files.get('rgbd_loc')
        if launch_info:
            pkg, launch = launch_info
            map_path = self.txtRgbdMapPath.text()

            if not self._validate_map_path(map_path):
                return

            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')
            else:
                extra_args.append('rviz:=false')

            # Copy map to default location for RTAB-Map
            self._prepare_map_for_localization(map_path)

            if self.node.start_launch_file('rgbd_loc', pkg, launch,
                                           extra_args=' '.join(extra_args) if extra_args else None):
                self.log(f"RTAB-Map RGB-D Localization started with map: {map_path}")
                self.update_button_states()
        else:
            self.log("RTAB-Map RGB-D localization launch file not found!")
            QMessageBox.warning(self, "Error", "RTAB-Map RGB-D localization launch file not found!")

    def on_stop_rgbd_loc(self):
        """Stop RTAB-Map RGB-D localization"""
        if self.node.stop_launch_file('rgbd_loc'):
            self.log("RTAB-Map RGB-D Localization stopped")
            self.update_button_states()

    # ==================== RTAB-Map RGB-D + LiDAR Tab ====================

    def on_start_rgbd_lidar_mapping(self):
        """Start RTAB-Map RGB-D + LiDAR mapping"""
        launch_info = self.node.launch_files.get('rgbd_lidar_mapping')
        if launch_info:
            pkg, launch = launch_info
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')
            else:
                extra_args.append('rviz:=false')

            # Add LiDAR topic parameter
            lidar_topic = self.cmbLidarTopic.currentText()
            if lidar_topic:
                extra_args.append(f'scan_topic:={lidar_topic}')
                self.log(f"Using LiDAR topic: {lidar_topic}")

            if self.node.start_launch_file('rgbd_lidar_mapping', pkg, launch,
                                           extra_args=' '.join(extra_args) if extra_args else None):
                self.log("RTAB-Map RGB-D + LiDAR Mapping started")
                self.update_button_states()
        else:
            self.log("RTAB-Map RGB-D + LiDAR mapping launch file not found!")
            QMessageBox.warning(self, "Error", "RTAB-Map RGB-D + LiDAR mapping launch file not found!")

    def on_stop_rgbd_lidar_mapping(self):
        """Stop RTAB-Map RGB-D + LiDAR mapping"""
        if self.node.stop_launch_file('rgbd_lidar_mapping'):
            self.log("RTAB-Map RGB-D + LiDAR Mapping stopped")
            self.update_button_states()

    def on_save_rgbd_lidar_map(self):
        """Save RTAB-Map RGB-D + LiDAR map"""
        self._save_rtabmap_db("rgbd_lidar")

    def on_browse_rgbd_lidar_map(self):
        """Browse for RTAB-Map RGB-D + LiDAR map file"""
        default_path = str(self.workspace_path / "maps" / "rtabmap_3d")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select RTAB-Map Database (.db)",
            default_path,
            "Database Files (*.db);;All Files (*)"
        )
        if file_path:
            self.txtRgbdLidarMapPath.setText(file_path)
            self.log(f"RGB-D + LiDAR map selected: {file_path}")

    def on_start_rgbd_lidar_loc(self):
        """Start RTAB-Map RGB-D + LiDAR localization"""
        launch_info = self.node.launch_files.get('rgbd_lidar_loc')
        if launch_info:
            pkg, launch = launch_info
            map_path = self.txtRgbdLidarMapPath.text()

            if not self._validate_map_path(map_path):
                return

            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')
            else:
                extra_args.append('rviz:=false')

            # Add LiDAR topic parameter
            lidar_topic = self.cmbLidarTopic.currentText()
            if lidar_topic:
                extra_args.append(f'scan_topic:={lidar_topic}')
                self.log(f"Using LiDAR topic: {lidar_topic}")

            # Copy map to default location for RTAB-Map
            self._prepare_map_for_localization(map_path)

            if self.node.start_launch_file('rgbd_lidar_loc', pkg, launch,
                                           extra_args=' '.join(extra_args) if extra_args else None):
                self.log(f"RTAB-Map RGB-D + LiDAR Localization started with map: {map_path}")
                self.update_button_states()
        else:
            self.log("RTAB-Map RGB-D + LiDAR localization launch file not found!")
            QMessageBox.warning(self, "Error", "RTAB-Map RGB-D + LiDAR localization launch file not found!")

    def on_stop_rgbd_lidar_loc(self):
        """Stop RTAB-Map RGB-D + LiDAR localization"""
        if self.node.stop_launch_file('rgbd_lidar_loc'):
            self.log("RTAB-Map RGB-D + LiDAR Localization stopped")
            self.update_button_states()

    # ==================== LIO-SAM Tab ====================

    def on_start_liosam_mapping(self):
        """Start LIO-SAM SLAM mapping"""
        launch_info = self.node.launch_files.get('lio_sam_mapping')
        if launch_info:
            pkg, launch = launch_info
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')

            if self.node.start_launch_file('lio_sam_mapping', pkg, launch,
                                           extra_args=' '.join(extra_args) if extra_args else None):
                self.log("LIO-SAM Mapping started")
                self.update_button_states()
        else:
            self.log("LIO-SAM mapping launch file not found!")
            QMessageBox.warning(self, "Error", "LIO-SAM mapping launch file not found!\n\nInstall livox_lio_sam package.")

    def on_stop_liosam_mapping(self):
        """Stop LIO-SAM SLAM mapping"""
        if self.node.stop_launch_file('lio_sam_mapping'):
            self.log("LIO-SAM Mapping stopped")
            self.update_button_states()

    def on_browse_liosam_map(self):
        """Browse for LIO-SAM PCD map file"""
        default_path = str(self.workspace_path / "maps" / "lio_sam")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select LIO-SAM Global Map (.pcd)",
            default_path,
            "PCD Files (*.pcd);;All Files (*)"
        )
        if file_path:
            self.txtLioSamMapPath.setText(file_path)
            self.log(f"LIO-SAM map selected: {file_path}")

    def on_start_liosam_loc(self):
        """Start LIO-SAM localization"""
        launch_info = self.node.launch_files.get('lio_sam_loc')
        if launch_info:
            pkg, launch = launch_info
            map_path = self.txtLioSamMapPath.text()

            if not map_path.endswith('.pcd'):
                self.log(f"Invalid map path: {map_path}")
                QMessageBox.warning(self, "Error", "Please select a .pcd file.")
                return

            if not os.path.exists(map_path):
                self.log(f"Map file not found: {map_path}")
                QMessageBox.warning(self, "Error", f"Map file not found:\n{map_path}")
                return

            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            extra_args.append(f'global_map_path:={map_path}')

            if self.node.start_launch_file('lio_sam_loc', pkg, launch,
                                           extra_args=' '.join(extra_args) if extra_args else None):
                self.log(f"LIO-SAM Localization started with map: {map_path}")
                self.update_button_states()
        else:
            self.log("LIO-SAM localization launch file not found!")
            QMessageBox.warning(self, "Error", "LIO-SAM localization launch file not found!")

    def on_stop_liosam_loc(self):
        """Stop LIO-SAM localization"""
        if self.node.stop_launch_file('lio_sam_loc'):
            self.log("LIO-SAM Localization stopped")
            self.update_button_states()

    # ==================== RTAB-Map 3D LiDAR Tab ====================

    def on_start_3dlidar_mapping(self):
        """Start RTAB-Map 3D LiDAR only mapping"""
        launch_info = self.node.launch_files.get('rtabmap_3dlidar_mapping')
        if launch_info:
            pkg, launch = launch_info
            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')
            else:
                extra_args.append('rviz:=false')

            if self.node.start_launch_file('rtabmap_3dlidar_mapping', pkg, launch,
                                           extra_args=' '.join(extra_args) if extra_args else None):
                self.log("RTAB-Map 3D LiDAR Mapping started")
                self.update_button_states()
        else:
            self.log("RTAB-Map 3D LiDAR mapping launch file not found!")
            QMessageBox.warning(self, "Error", "RTAB-Map 3D LiDAR mapping launch file not found!")

    def on_stop_3dlidar_mapping(self):
        """Stop RTAB-Map 3D LiDAR only mapping"""
        if self.node.stop_launch_file('rtabmap_3dlidar_mapping'):
            self.log("RTAB-Map 3D LiDAR Mapping stopped")
            self.update_button_states()

    def on_save_3dlidar_map(self):
        """Save RTAB-Map 3D LiDAR map"""
        self._save_rtabmap_db("3dlidar")

    def on_browse_3dlidar_map(self):
        """Browse for RTAB-Map 3D LiDAR map file"""
        default_path = str(self.workspace_path / "maps" / "rtabmap_3d")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Select RTAB-Map Database (.db)",
            default_path,
            "Database Files (*.db);;All Files (*)"
        )
        if file_path:
            self.txt3dLidarMapPath.setText(file_path)
            self.log(f"3D LiDAR map selected: {file_path}")

    def on_start_3dlidar_loc(self):
        """Start RTAB-Map 3D LiDAR only localization"""
        launch_info = self.node.launch_files.get('rtabmap_3dlidar_loc')
        if launch_info:
            pkg, launch = launch_info
            map_path = self.txt3dLidarMapPath.text()

            if not self._validate_map_path(map_path):
                return

            extra_args = []
            if self.chkUseSimTime.isChecked():
                extra_args.append('use_sim_time:=true')
            if self.chkRviz.isChecked():
                extra_args.append('rviz:=true')
            else:
                extra_args.append('rviz:=false')

            self._prepare_map_for_localization(map_path)

            if self.node.start_launch_file('rtabmap_3dlidar_loc', pkg, launch,
                                           extra_args=' '.join(extra_args) if extra_args else None):
                self.log(f"RTAB-Map 3D LiDAR Localization started with map: {map_path}")
                self.update_button_states()
        else:
            self.log("RTAB-Map 3D LiDAR localization launch file not found!")
            QMessageBox.warning(self, "Error", "RTAB-Map 3D LiDAR localization launch file not found!")

    def on_stop_3dlidar_loc(self):
        """Stop RTAB-Map 3D LiDAR only localization"""
        if self.node.stop_launch_file('rtabmap_3dlidar_loc'):
            self.log("RTAB-Map 3D LiDAR Localization stopped")
            self.update_button_states()

    # ==================== Helper Methods ====================

    def _validate_map_path(self, map_path):
        """Validate map file path"""
        if not map_path.endswith('.db'):
            self.log(f"Invalid map path: {map_path}")
            QMessageBox.warning(
                self, "Error",
                "Please select a .db file.\n\n"
                "Use Browse button to select the map file."
            )
            return False

        if not os.path.exists(map_path):
            self.log(f"Map file not found: {map_path}")
            QMessageBox.warning(
                self, "Error",
                f"Map file not found:\n{map_path}"
            )
            return False

        return True

    def _prepare_map_for_localization(self, map_path):
        """Copy map to default RTAB-Map location"""
        import shutil
        default_db = Path.home() / ".ros" / "rtabmap.db"
        shutil.copy2(map_path, str(default_db))
        self.log(f"Map copied to: {default_db}")

    def _save_rtabmap_db(self, prefix):
        """Save RTAB-Map database"""
        try:
            default_db = Path.home() / ".ros" / "rtabmap.db"

            if not default_db.exists():
                QMessageBox.warning(self, "Error",
                                    "No map database found!\n\nMake sure mapping is running.")
                return

            save_dir = self.workspace_path / "maps" / "rtabmap_3d"
            save_dir.mkdir(parents=True, exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = save_dir / f"rtabmap3d_{prefix}_{timestamp}.db"

            import shutil
            shutil.copy2(str(default_db), str(save_path))

            self.log(f"Map saved: {save_path}")
            QMessageBox.information(self, "Success", f"Map saved to:\n{save_path}")

        except Exception as e:
            self.log(f"Failed to save map: {e}")
            QMessageBox.warning(self, "Error", f"Failed to save map:\n{e}")

    # ==================== Common ====================

    def on_stop_all(self):
        """Stop all running processes"""
        reply = QMessageBox.question(
            self,
            "Confirm",
            "Stop all running processes?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.node.cleanup()
            if self.bag_process is not None:
                self.on_stop_bag()
            self.log("All processes stopped")
            self.update_button_states()

    def _format_time(self, seconds):
        """Format seconds to MM:SS"""
        minutes = int(seconds // 60)
        secs = int(seconds % 60)
        return f"{minutes:02d}:{secs:02d}"

    def _update_bag_button_states(self):
        """Update ROS bag button states"""
        bag_running = self.bag_process is not None and self.bag_process.poll() is None

        self.btnPlayBag.setEnabled(not bag_running)
        self.btnPauseBag.setEnabled(bag_running)
        self.btnStopBag.setEnabled(bag_running)

        if self.bag_paused:
            self.btnPauseBag.setText("Resume")
        else:
            self.btnPauseBag.setText("Pause")

        if bag_running and self.bag_duration > 0 and self.bag_start_time is not None:
            elapsed_ms = self.bag_start_time.msecsTo(QDateTime.currentDateTime())
            rate = self.spinBagRate.value()
            elapsed_sec = (elapsed_ms / 1000.0) * rate

            if self.chkBagLoop.isChecked():
                elapsed_sec = elapsed_sec % self.bag_duration

            progress = min(100, int((elapsed_sec / self.bag_duration) * 100))
            self.progressBag.setValue(progress)

            current_time = self._format_time(min(elapsed_sec, self.bag_duration))
            total_time = self._format_time(self.bag_duration)
            self.progressBag.setFormat(f"{progress}% - {current_time} / {total_time}")
        elif not bag_running:
            if self.bag_process is None:
                self.progressBag.setValue(0)
                self.progressBag.setFormat("0% - 00:00 / 00:00")

    def update_button_states(self):
        """Update all button states based on running processes"""
        if self.node is None:
            return

        # Style definitions
        style_ready = "background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;"
        style_running = "background-color: #2196F3; color: white; font-weight: bold; padding: 10px;"
        style_loc = "background-color: #2196F3; color: white; font-weight: bold; padding: 10px;"
        style_save = "background-color: #FF9800; color: white; font-weight: bold; padding: 10px;"

        # Check running states
        orbbec_running = self.node.is_process_running('orbbec_camera')
        rgbd_mapping_running = self.node.is_process_running('rgbd_mapping')
        rgbd_loc_running = self.node.is_process_running('rgbd_loc')
        rgbd_lidar_mapping_running = self.node.is_process_running('rgbd_lidar_mapping')
        rgbd_lidar_loc_running = self.node.is_process_running('rgbd_lidar_loc')

        # Orbbec Camera
        self.btnStartOrbbec.setEnabled(not orbbec_running)
        self.btnStopOrbbec.setEnabled(orbbec_running)
        if orbbec_running:
            self.lblOrbbecStatus.setText("Status: Running")
            self.btnStartOrbbec.setStyleSheet(style_running)
        else:
            self.lblOrbbecStatus.setText("Status: Stopped")
            self.btnStartOrbbec.setStyleSheet(style_ready)

        # RTAB-Map RGB-D Mapping
        rgbd_mapping_configured = self.node.launch_files.get('rgbd_mapping') is not None
        self.btnStartRgbdMapping.setEnabled(
            rgbd_mapping_configured and not rgbd_mapping_running and not rgbd_loc_running)
        self.btnStopRgbdMapping.setEnabled(rgbd_mapping_running)
        self.btnSaveRgbdMap.setEnabled(rgbd_mapping_running)
        if rgbd_mapping_running:
            self.btnStartRgbdMapping.setStyleSheet(style_running)
        else:
            self.btnStartRgbdMapping.setStyleSheet(style_ready)
        self.btnSaveRgbdMap.setStyleSheet(style_save)

        # RTAB-Map RGB-D Localization
        rgbd_loc_configured = self.node.launch_files.get('rgbd_loc') is not None
        self.btnStartRgbdLoc.setEnabled(
            rgbd_loc_configured and not rgbd_loc_running and not rgbd_mapping_running)
        self.btnStopRgbdLoc.setEnabled(rgbd_loc_running)
        if rgbd_loc_running:
            self.btnStartRgbdLoc.setStyleSheet(style_running)
        else:
            self.btnStartRgbdLoc.setStyleSheet(style_loc)

        # RTAB-Map RGB-D + LiDAR Mapping
        rgbd_lidar_mapping_configured = self.node.launch_files.get('rgbd_lidar_mapping') is not None
        self.btnStartRgbdLidarMapping.setEnabled(
            rgbd_lidar_mapping_configured and not rgbd_lidar_mapping_running and not rgbd_lidar_loc_running)
        self.btnStopRgbdLidarMapping.setEnabled(rgbd_lidar_mapping_running)
        self.btnSaveRgbdLidarMap.setEnabled(rgbd_lidar_mapping_running)
        if rgbd_lidar_mapping_running:
            self.btnStartRgbdLidarMapping.setStyleSheet(style_running)
        else:
            self.btnStartRgbdLidarMapping.setStyleSheet(style_ready)
        self.btnSaveRgbdLidarMap.setStyleSheet(style_save)

        # RTAB-Map RGB-D + LiDAR Localization
        rgbd_lidar_loc_configured = self.node.launch_files.get('rgbd_lidar_loc') is not None
        self.btnStartRgbdLidarLoc.setEnabled(
            rgbd_lidar_loc_configured and not rgbd_lidar_loc_running and not rgbd_lidar_mapping_running)
        self.btnStopRgbdLidarLoc.setEnabled(rgbd_lidar_loc_running)
        if rgbd_lidar_loc_running:
            self.btnStartRgbdLidarLoc.setStyleSheet(style_running)
        else:
            self.btnStartRgbdLidarLoc.setStyleSheet(style_loc)

        # LIO-SAM
        liosam_mapping_running = self.node.is_process_running('lio_sam_mapping')
        liosam_loc_running = self.node.is_process_running('lio_sam_loc')

        liosam_mapping_configured = self.node.launch_files.get('lio_sam_mapping') is not None
        self.btnStartLioSamMapping.setEnabled(
            liosam_mapping_configured and not liosam_mapping_running and not liosam_loc_running)
        self.btnStopLioSamMapping.setEnabled(liosam_mapping_running)
        if liosam_mapping_running:
            self.btnStartLioSamMapping.setStyleSheet(style_running)
        else:
            self.btnStartLioSamMapping.setStyleSheet(style_ready)

        liosam_loc_configured = self.node.launch_files.get('lio_sam_loc') is not None
        self.btnStartLioSamLoc.setEnabled(
            liosam_loc_configured and not liosam_loc_running and not liosam_mapping_running)
        self.btnStopLioSamLoc.setEnabled(liosam_loc_running)
        if liosam_loc_running:
            self.btnStartLioSamLoc.setStyleSheet(style_running)
        else:
            self.btnStartLioSamLoc.setStyleSheet(style_loc)

        # RTAB-Map 3D LiDAR
        rtabmap_3dlidar_mapping_running = self.node.is_process_running('rtabmap_3dlidar_mapping')
        rtabmap_3dlidar_loc_running = self.node.is_process_running('rtabmap_3dlidar_loc')

        rtabmap_3dlidar_mapping_configured = self.node.launch_files.get('rtabmap_3dlidar_mapping') is not None
        self.btnStart3dLidarMapping.setEnabled(
            rtabmap_3dlidar_mapping_configured and not rtabmap_3dlidar_mapping_running and not rtabmap_3dlidar_loc_running)
        self.btnStop3dLidarMapping.setEnabled(rtabmap_3dlidar_mapping_running)
        self.btnSave3dLidarMap.setEnabled(rtabmap_3dlidar_mapping_running)
        if rtabmap_3dlidar_mapping_running:
            self.btnStart3dLidarMapping.setStyleSheet(style_running)
        else:
            self.btnStart3dLidarMapping.setStyleSheet(style_ready)
        self.btnSave3dLidarMap.setStyleSheet(style_save)

        rtabmap_3dlidar_loc_configured = self.node.launch_files.get('rtabmap_3dlidar_loc') is not None
        self.btnStart3dLidarLoc.setEnabled(
            rtabmap_3dlidar_loc_configured and not rtabmap_3dlidar_loc_running and not rtabmap_3dlidar_mapping_running)
        self.btnStop3dLidarLoc.setEnabled(rtabmap_3dlidar_loc_running)
        if rtabmap_3dlidar_loc_running:
            self.btnStart3dLidarLoc.setStyleSheet(style_running)
        else:
            self.btnStart3dLidarLoc.setStyleSheet(style_loc)

        # ROS Bag
        self._update_bag_button_states()

    def on_scan_lidar_topics(self):
        """Scan for available LaserScan topics"""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list', '-t'],
                capture_output=True, text=True, timeout=15
            )
            if result.returncode == 0:
                topics = []
                for line in result.stdout.strip().split('\n'):
                    if 'sensor_msgs/msg/LaserScan' in line:
                        # Format: "/topic_name [type]"
                        topic_name = line.split()[0]
                        topics.append(topic_name)

                if topics:
                    # Save current selection
                    current = self.cmbLidarTopic.currentText()

                    # Update combo box
                    self.cmbLidarTopic.clear()
                    self.cmbLidarTopic.addItems(topics)

                    # Restore selection if still available
                    idx = self.cmbLidarTopic.findText(current)
                    if idx >= 0:
                        self.cmbLidarTopic.setCurrentIndex(idx)

                    self.log(f"Found {len(topics)} LaserScan topic(s)")
                else:
                    self.log("No LaserScan topics found!")
                    QMessageBox.warning(
                        self, "Warning",
                        "No LaserScan topics found!\n\n"
                        "Make sure LiDAR driver is running."
                    )
            else:
                self.log("Failed to list ROS2 topics")
        except Exception as e:
            self.log(f"Error scanning topics: {e}")

    def closeEvent(self, event):
        """Handle window close event"""
        reply = QMessageBox.question(
            self,
            "Confirm Exit",
            "Stop all processes and exit?",
            QMessageBox.Yes | QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            if self.node:
                self.node.cleanup()
            if self.bag_process is not None:
                self.on_stop_bag()
            event.accept()
        else:
            event.ignore()
