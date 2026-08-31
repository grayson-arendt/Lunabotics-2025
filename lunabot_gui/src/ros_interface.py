#!/usr/bin/env python3
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PyQt5.QtCore import QCoreApplication
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Bool, Float32, Float64, Float64MultiArray

from lunabot_logger import Logger
from lunabot_msgs.action import Depositing, Excavation
from lunabot_msgs.msg import ControlState, PowerMonitor
from lunabot_msgs.srv import LaunchSystem, StopSystem

try:
    from cv_bridge import CvBridge
    import cv2
    import numpy as np
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


# Service / process timeout constants (seconds)
_TIMEOUT_SERVICE_WAIT = 5.0
_TIMEOUT_SERVICE_WAIT_LONG = 10.0
_TIMEOUT_SERVICE_RESPONSE = 30.0
_TIMEOUT_STOP_RESPONSE = 10.0
_TIMEOUT_PROCESS_TERMINATE = 5

_DISPLAY_NAMES = {
    'pointlio':     'Point-LIO',
    'mapping':      'RTAB-Map',
    'nav2':         'Navigation2',
    'hardware':     'Hardware',
}

class RobotInterface:
    def __init__(self, mode_param):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('lunabot_gui')
        self._shutdown = False
        self.log = Logger(self.node)

        self.node.declare_parameter('network.robot_host', '192.168.0.250')
        self.node.declare_parameter('network.robot_user', 'codetc')
        self.node.declare_parameter('network.robot_workspace', '~/lunabot_ws')
        
        # mode_param uses True=sim convention (from the 'mode' ROS param default)
        if isinstance(mode_param, bool):
            self.is_real_mode = not mode_param
        elif isinstance(mode_param, str):
            mode_lower = mode_param.lower()
            self.is_real_mode = (mode_lower == 'false' or mode_lower == 'real')
        else:
            self.is_real_mode = False
        
        self.robot_mode_type = 'real' if self.is_real_mode else 'sim'
        self.log.info(f'ROS interface running in {self.robot_mode_type.upper()} mode')
        
        self.robot_host = self.node.get_parameter('network.robot_host').value
        self.robot_user = self.node.get_parameter('network.robot_user').value
        self.robot_workspace = self.node.get_parameter('network.robot_workspace').value
        self.is_remote = self.is_real_mode  # remote SSH launch only applies in real mode
        
        if self.is_remote:
            self.log.info(f'Remote robot: {self.robot_user}@{self.robot_host}:{self.robot_workspace}')
        
        self.bridge = CvBridge() if CV_AVAILABLE else None
        
        self.bandwidth_total = 0.0
        self.bandwidth_rx = 0.0
        self.bandwidth_tx = 0.0
        self.bandwidth_avg_total = 0.0
        self.bandwidth_avg_rx = 0.0
        self.bandwidth_avg_tx = 0.0

        self.power_voltage = 0.0
        self.power_current = 0.0
        self.power_watts = 0.0
        self.power_temp = 0.0
        self.power_energy_wh = 0.0
        self.power_alert_status = "Unknown"

        self.front_camera_image = None
        self.rear_camera_image = None
        self.fisheye_camera_image = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.orientation_roll = 0.0
        self.orientation_pitch = 0.0
        self.orientation_yaw = 0.0
        self.bucket_position = 0.0
        self.actuator_position = -5.25

        self.realsense_enabled = False
        self.front_camera_sub = None
        self.rear_camera_sub = None

        self.hardware_enabled = False
        self.pointlio_enabled = False
        self.mapping_enabled = False
        self.nav2_enabled = False

        self.robot_mode = "MANUAL"
        self.is_excavating = False
        self.is_depositing = False
        self.is_navigating = False
        self.robot_disabled = False
        self.vibration_duty_cycle = 0.0
        
        # Launch process tracking (True = remote via service, Process = local subprocess)
        self.launch_processes = {
            'hardware': None,
            'actions': None,
            'pointlio': None,
            'mapping': None,
            'nav2': None
        }
        
        self.excavation_client = ActionClient(self.node, Excavation, 'auto_excavation_action')
        self.depositing_client = ActionClient(self.node, Depositing, 'auto_depositing_action')
        
        self.excavation_goal_handle = None
        self.depositing_goal_handle = None
        self.navigation_client_process = None
        
        self.emergency_stop_pub = self.node.create_publisher(Bool, '/emergency_stop', 10)
        self.mode_switch_pub = self.node.create_publisher(Bool, '/mode_switch', 10)
        self.position_cmd_pub = self.node.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.camera_cmd_pub = self.node.create_publisher(Float64MultiArray, '/camera_controller/commands', 10)
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.excavation_angle_pub = self.node.create_publisher(Float64, '/excavation_angle', 10)
        
        self.launch_client = self.node.create_client(LaunchSystem, 'launch_system')
        self.stop_client = self.node.create_client(StopSystem, 'stop_system')
        
        self._create_subscriptions()
        
        # Callbacks for UI updates (set by GUI)
        self.on_robot_state_update = None
        self.on_control_state_update = None
        self.on_realsense_toggle = None
        self.on_log = None
    
    def _create_subscriptions(self):
        self.node.create_subscription(
            Float32, '/bandwidth/total_mbps',
            lambda msg: setattr(self, 'bandwidth_total', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/rx_mbps',
            lambda msg: setattr(self, 'bandwidth_rx', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/tx_mbps',
            lambda msg: setattr(self, 'bandwidth_tx', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/avg_total_mbps',
            lambda msg: setattr(self, 'bandwidth_avg_total', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/avg_rx_mbps',
            lambda msg: setattr(self, 'bandwidth_avg_rx', msg.data), 10)
        self.node.create_subscription(
            Float32, '/bandwidth/avg_tx_mbps',
            lambda msg: setattr(self, 'bandwidth_avg_tx', msg.data), 10)

        if CV_AVAILABLE:
            if self.realsense_enabled:
                self._enable_realsense_subscriptions()
            self.node.create_subscription(
                CompressedImage, '/camera_fisheye/color/image_compressed',
                lambda msg: self._camera_callback(msg, 'fisheye', 'fisheye_camera_image'), 1)
        
        self.node.create_subscription(
            Odometry, '/odometry/filtered', self._odom_callback, 10)

        self.node.create_subscription(
            ControlState, '/control_state', self._control_state_callback, 10)

        self.node.create_subscription(
            PowerMonitor, '/power_monitor', self._power_monitor_callback, 10)

        self.node.create_subscription(
            Bool, '/manual_mode', self._manual_mode_callback, 10)

        self.node.create_subscription(
            Bool, '/robot_disabled', self._robot_disabled_callback, 10)

        self.node.create_subscription(
            Float32, '/vibration_duty_cycle', self._vibration_duty_cycle_callback, 10)

        self.node.create_subscription(
            Float64, '/actuator_position',
            lambda msg: setattr(self, 'actuator_position', msg.data), 10)
        self.node.create_subscription(
            Float64, '/bucket_angle',
            lambda msg: setattr(self, 'bucket_position', msg.data), 10)

        self.node.create_subscription(
            JointState, '/joint_states', self._joint_states_callback, 10)
        
        # /rosout publishers use RELIABLE + TRANSIENT_LOCAL
        rosout_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1000
        )
        
        self.node.create_subscription(
            Log, '/rosout', self._log_callback, rosout_qos)
    
    def _camera_callback(self, msg, camera_name, attr_name):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            setattr(self, attr_name, cv_image)
        except Exception as e:
            self.log.failure(f'{camera_name.capitalize()} camera error: {e}')
    
    def _enable_realsense_subscriptions(self):
        if not CV_AVAILABLE:
            return
        
        # Queue size = 1 for minimal latency (we only care about latest frame)
        if self.front_camera_sub is None:
            self.front_camera_sub = self.node.create_subscription(
                CompressedImage, '/camera_front/color/image_compressed',
                lambda msg: self._camera_callback(msg, 'front', 'front_camera_image'), 1)
            self.log.info('Front RealSense camera subscription enabled')
        
        if self.rear_camera_sub is None:
            self.rear_camera_sub = self.node.create_subscription(
                CompressedImage, '/camera_back/color/image_compressed',
                lambda msg: self._camera_callback(msg, 'rear', 'rear_camera_image'), 1)
            self.log.info('Rear RealSense camera subscription enabled')
    
    def _disable_realsense_subscriptions(self):
        if self.front_camera_sub is not None:
            self.node.destroy_subscription(self.front_camera_sub)
            self.front_camera_sub = None
            self.front_camera_image = None
            self.log.info('Front RealSense camera subscription disabled')
        
        if self.rear_camera_sub is not None:
            self.node.destroy_subscription(self.rear_camera_sub)
            self.rear_camera_sub = None
            self.rear_camera_image = None
            self.log.info('Rear RealSense camera subscription disabled')
    
    def toggle_realsense_cameras(self):
        self.realsense_enabled = not self.realsense_enabled
        
        if self.realsense_enabled:
            self._enable_realsense_subscriptions()
            self.log.action('RealSense cameras enabled')
        else:
            self._disable_realsense_subscriptions()
            self.log.action('RealSense cameras disabled')
        
        if self.on_realsense_toggle:
            self.on_realsense_toggle(self.realsense_enabled)
        
        return self.realsense_enabled
    
    def restart_can_interface(self):
        try:
            script_path = f'{self.robot_workspace}/src/lunabot_ros/scripts/canable_restart.sh'
            
            if self.is_remote:
                self.log.action(f'Restarting CAN interface on {self.robot_user}@{self.robot_host}')
                cmd = ['ssh', f'{self.robot_user}@{self.robot_host}', f'bash -l {script_path}']
            else:
                self.log.action(f'Restarting CAN interface: {script_path}')
                cmd = ['bash', os.path.expanduser(script_path)]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.stdout:
                for line in result.stdout.strip().split('\n'):
                    if line:
                        self.log.info(f'CAN: {line}')
            if result.stderr:
                for line in result.stderr.strip().split('\n'):
                    if line:
                        self.log.warning(f'CAN: {line}')
            
            if result.returncode == 0:
                self.log.success('CAN interface restarted successfully')
                return True
            else:
                self.log.failure(f'CAN restart failed with exit code {result.returncode}')
                return False
        except subprocess.TimeoutExpired:
            self.log.failure('CAN restart timed out')
            return False
        except Exception as e:
            self.log.failure(f'Error restarting CAN: {e}')
            return False
    
    def _odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.position_z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        self.orientation_roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            self.orientation_pitch = math.copysign(math.pi / 2, sinp)
        else:
            self.orientation_pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.orientation_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z
        if self.on_robot_state_update:
            self.on_robot_state_update()
    
    def _manual_mode_callback(self, msg):
        is_manual = msg.data
        self.robot_mode = "MANUAL" if is_manual else "AUTO"
        if self.on_robot_state_update:
            self.on_robot_state_update()
    
    def _robot_disabled_callback(self, msg):
        self.robot_disabled = msg.data
        if self.on_robot_state_update:
            self.on_robot_state_update()
    
    def _vibration_duty_cycle_callback(self, msg):
        self.vibration_duty_cycle = msg.data
        if self.on_robot_state_update:
            self.on_robot_state_update()
    
    def _joint_states_callback(self, msg):
        try:
            if not self.is_real_mode and 'base_bucket_joint' in msg.name:
                idx = msg.name.index('base_bucket_joint')
                if idx < len(msg.position):
                    self.bucket_position = msg.position[idx]
        except Exception as e:
            self.log.warning(f'Error processing joint states: {e}', throttle_duration_sec=5.0)
    
    def _log_callback(self, msg):
        if self.on_log:
            level_map = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
            level = level_map.get(msg.level, 'UNKNOWN')
            log_text = f"[{level}] {msg.name}: {msg.msg}"
            try:
                self.on_log(log_text)
            except Exception as e:
                print(f"Failed to log message from {msg.name}: {e}", file=sys.stderr)
    
    def _control_state_callback(self, msg):
        self.robot_mode = "MANUAL" if msg.mode == 0 else "AUTO"
        self.hardware_enabled = msg.hardware_enabled
        self.pointlio_enabled = msg.pointlio_enabled
        self.mapping_enabled = msg.mapping_enabled
        self.nav2_enabled = msg.nav2_enabled
        self.is_excavating = msg.is_excavating
        self.is_depositing = msg.is_depositing
        self.is_navigating = msg.is_navigating
        
        if self.on_control_state_update:
            self.on_control_state_update(msg)
    
    def _power_monitor_callback(self, msg):
        self.power_voltage = msg.voltage
        self.power_current = msg.current
        self.power_watts = msg.power
        self.power_temp = msg.temperature
        self.power_energy_wh = msg.energy_wh
        self.power_alert_status = msg.alert_status
    
    def publish_emergency_stop(self):
        msg = Bool()
        msg.data = True
        self.emergency_stop_pub.publish(msg)
    
    def publish_re_enable(self):
        msg = Bool()
        msg.data = False
        self.emergency_stop_pub.publish(msg)
    
    def publish_mode_switch(self):
        msg = Bool()
        msg.data = not (self.robot_mode == "MANUAL")
        self.mode_switch_pub.publish(msg)
        self.log.action(f'Mode switch requested: {"MANUAL" if msg.data else "AUTO"}')
    
    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
    
    def publish_bucket_position(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        self.position_cmd_pub.publish(msg)
    
    def publish_excavation_angle(self, angle):
        msg = Float64()
        msg.data = angle
        self.excavation_angle_pub.publish(msg)
    
    def publish_camera_position(self, position):
        msg = Float64MultiArray()
        msg.data = [position]
        self.camera_cmd_pub.publish(msg)
    
    def start_can_interface(self):
        try:
            script_path = f'{self.robot_workspace}/src/lunabot_ros/scripts/canable_start.sh'
            
            if self.is_remote:
                self.log.action(f'Running CAN interface on {self.robot_user}@{self.robot_host}')
                cmd = ['ssh', f'{self.robot_user}@{self.robot_host}', f'bash -l {script_path}']
            else:
                self.log.action(f'Running CAN interface script: {script_path}')
                cmd = ['bash', os.path.expanduser(script_path)]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=int(_TIMEOUT_SERVICE_WAIT_LONG))
            
            if result.stdout:
                for line in result.stdout.strip().split('\n'):
                    if line:
                        self.log.info(f'CAN: {line}')
            if result.stderr:
                for line in result.stderr.strip().split('\n'):
                    if line:
                        self.log.warning(f'CAN: {line}')
            
            if result.returncode == 0:
                self.log.success('CAN interface started successfully')
            else:
                self.log.failure(f'CAN script exited with code {result.returncode}')
        except subprocess.TimeoutExpired:
            self.log.failure('CAN script timed out')
        except Exception as e:
            self.log.failure(f'Failed to start CAN interface: {str(e)}')
    
    def _kill_proc(self, process, name):
        try:
            pgid = os.getpgid(process.pid)
        except ProcessLookupError:
            return
        try:
            os.killpg(pgid, signal.SIGINT)   # SIGINT = graceful ROS 2 shutdown
            process.wait(timeout=_TIMEOUT_PROCESS_TERMINATE)
            self.log.info(f'Terminated {name} process')
        except subprocess.TimeoutExpired:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.log.warning(f'Killed {name} process (force)')
        except Exception as e:
            self.log.warning(f'Error terminating {name}: {e}')

    def _wait_for_service_response(self, future, timeout_sec):
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout_sec:
            QCoreApplication.processEvents()
            time.sleep(0.01)
        if future.done() and future.result() is not None:
            return future.result()
        return None

    def _set_system_enabled(self, system_name, value):
        flag_map = {
            'pointlio':     'pointlio_enabled',
            'mapping':      'mapping_enabled',
            'nav2':         'nav2_enabled',
            'hardware':     'hardware_enabled',
        }
        attr = flag_map.get(system_name)
        if attr is not None:
            setattr(self, attr, value)

    def launch_hardware(self):
        if self.launch_processes['hardware'] is not None:
            self.log.warning('Hardware already running')
            return
        
        try:
            if self.is_remote:
                request = LaunchSystem.Request()
                request.system_name = 'hardware'
                request.use_sim = False

                if not self.launch_client.wait_for_service(timeout_sec=_TIMEOUT_SERVICE_WAIT):
                    self.log.failure('Launch manager service not available')
                    return

                future = self.launch_client.call_async(request)
                response = self._wait_for_service_response(future, _TIMEOUT_SERVICE_RESPONSE)
                if response is not None:
                    if response.success:
                        self.log.success(f'Hardware launched: {response.message}')
                        self.launch_processes['hardware'] = True
                    else:
                        self.log.failure(f'Failed to launch hardware: {response.message}')
                        return
                else:
                    self.log.failure('Service call failed for hardware launch')
                    return
            else:
                self.launch_processes['hardware'] = subprocess.Popen(
                    ['ros2', 'launch', 'lunabot_bringup', 'hardware_launch.py'],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    start_new_session=True)

            self.hardware_enabled = True
            self.log.success('Hardware launched')

            if self.on_robot_state_update:
                self.on_robot_state_update()
        except Exception as e:
            self.log.failure(f'Failed to launch hardware: {str(e)}')
    
    def launch_system(self, system_name):
        """Launch a system (PointLIO, mapping, Nav2)"""
        display = _DISPLAY_NAMES.get(system_name, system_name)
        self.log.action(f'Launching {display}')

        if self.launch_processes[system_name] is not None:
            self.log.warning(f'{display} already running')
            return

        try:
            run_on_robot = self.is_real_mode and self.is_remote
            use_sim = not self.is_real_mode

            if run_on_robot:
                request = LaunchSystem.Request()
                request.system_name = system_name
                request.use_sim = use_sim

                if not self.launch_client.wait_for_service(timeout_sec=_TIMEOUT_SERVICE_WAIT_LONG):
                    self.log.failure('Launch manager service not available')
                    return

                future = self.launch_client.call_async(request)
                response = self._wait_for_service_response(future, _TIMEOUT_SERVICE_RESPONSE)
                if response is not None:
                    if response.success:
                        self.log.success(f'{display} launched')
                        self.launch_processes[system_name] = True
                    else:
                        self.log.failure(f'Failed to launch {display}: {response.message}')
                        return
                else:
                    self.log.failure(f'Service call timed out for {display}')
                    return

                self._set_system_enabled(system_name, True)
            else:
                use_sim_arg = 'true' if use_sim else 'false'
                launch_file = f'{system_name}_launch.py'
                if system_name == 'nav2':
                    launch_file = 'nav2_stack_launch.py'

                self.launch_processes[system_name] = subprocess.Popen(
                    ['ros2', 'launch', 'lunabot_bringup', launch_file, f'use_sim:={use_sim_arg}'],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    start_new_session=True
                )
                self._set_system_enabled(system_name, True)

            location = "via launch manager" if run_on_robot else "locally"
            self.log.success(f'{display} launched {location}')

            if self.on_robot_state_update:
                self.on_robot_state_update()
        except Exception as e:
            self.log.failure(f'Failed to launch {display}: {e}')
    
    def stop_system(self, system_name):
        display = _DISPLAY_NAMES.get(system_name, system_name)
        self.log.action(f'Stopping {display}')

        try:
            if self.launch_processes[system_name] is None:
                self.log.warning(f'{display} is not running')
                return

            # If launch_processes is True, it's a remote process via service
            if self.launch_processes[system_name] is True and self.is_remote:
                request = StopSystem.Request()
                request.system_name = system_name

                if not self.stop_client.wait_for_service(timeout_sec=_TIMEOUT_SERVICE_WAIT):
                    self.log.failure('Stop service not available')
                    return

                future = self.stop_client.call_async(request)
                response = self._wait_for_service_response(future, _TIMEOUT_STOP_RESPONSE)
                if response is not None:
                    if response.success:
                        self.log.success(f'{display} stopped')
                    else:
                        self.log.warning(f'Stop warning: {response.message}')
                else:
                    self.log.failure(f'Service call timed out for stopping {display}')

            elif isinstance(self.launch_processes[system_name], subprocess.Popen):
                self._kill_proc(self.launch_processes[system_name], system_name)

            self.launch_processes[system_name] = None
            self._set_system_enabled(system_name, False)

            self.log.success(f'{display} stopped')

            if self.on_robot_state_update:
                self.on_robot_state_update()
        except Exception as e:
            self.log.failure(f'Error stopping {display}: {e}')
    
    
    def _get_workspace_setup_cmd(self):
        ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
        if ament_prefix:
            install_paths = ament_prefix.split(':')
            workspace_install = None
            for path in install_paths:
                if 'lunabot' in path and 'install' in path:
                    workspace_install = path.split('/install/')[0] + '/install'
                    break
            if not workspace_install and install_paths:
                workspace_install = install_paths[0].split('/install/')[0] + '/install'
            return f'source {workspace_install}/setup.bash'
        return 'source install/setup.bash'

    def launch_rviz(self):
        self.log.action('Launching RViz2')
        try:
            config_path = '$(ros2 pkg prefix lunabot_config)/share/lunabot_config/rviz/robot_view.rviz'
            subprocess.Popen(
                f'{self._get_workspace_setup_cmd()} && rviz2 -d {config_path}',
                shell=True, executable='/bin/bash')
            self.log.success('RViz2 launched')
        except Exception as e:
            self.log.failure(f'Failed to launch RViz2: {e}')
    
    def send_excavate_goal(self, response_callback, result_callback, cancel_callback):
        self.log.action('Sending excavation goal')
        goal_msg = Excavation.Goal()
        send_goal_future = self.excavation_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self._handle_excavate_response(future, response_callback, result_callback))
        return send_goal_future
    
    def _handle_excavate_response(self, future, response_callback, result_callback):
        goal_handle = future.result()
        self.excavation_goal_handle = goal_handle
        response_callback(goal_handle)
        if goal_handle.accepted:
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_callback)
    
    def cancel_excavate_goal(self, cancel_callback):
        if self.excavation_goal_handle is not None and self.excavation_goal_handle.accepted:
            self.log.action('Canceling excavation goal')
            cancel_future = self.excavation_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(cancel_callback)
        else:
            self.log.warning('No active excavation goal to cancel')
            cancel_callback(None)
    
    def send_deposit_goal(self, response_callback, result_callback, cancel_callback):
        self.log.action('Sending depositing goal')
        goal_msg = Depositing.Goal()
        send_goal_future = self.depositing_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self._handle_deposit_response(future, response_callback, result_callback))
        return send_goal_future
    
    def _handle_deposit_response(self, future, response_callback, result_callback):
        goal_handle = future.result()
        self.depositing_goal_handle = goal_handle
        response_callback(goal_handle)
        if goal_handle.accepted:
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_callback)
    
    def cancel_deposit_goal(self, cancel_callback):
        if self.depositing_goal_handle is not None and self.depositing_goal_handle.accepted:
            self.log.action('Canceling depositing goal')
            cancel_future = self.depositing_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(cancel_callback)
        else:
            self.log.warning('No active depositing goal to cancel')
            cancel_callback(None)
    
    def launch_navigation_client(self):
        self.log.action('Starting navigation client')
        try:
            use_sim_arg = 'false' if self.is_real_mode else 'true'
            self.navigation_client_process = subprocess.Popen(
                f'{self._get_workspace_setup_cmd()} && ros2 run lunabot_nav navigation_client'
                f' --ros-args -p use_sim_time:={use_sim_arg}',
                shell=True, executable='/bin/bash', preexec_fn=os.setsid)
            self.log.success('Navigation client launched')
            return True
        except Exception as e:
            self.log.failure(f'Failed to launch navigation client: {e}')
            return False
    
    def stop_navigation_client(self):
        if self.navigation_client_process:
            try:
                os.killpg(os.getpgid(self.navigation_client_process.pid), signal.SIGTERM)
                self.navigation_client_process = None
                self.log.success('Navigation client stopped')
            except Exception as e:
                self.log.failure(f'Failed to stop navigation client: {e}')
    
    def shutdown(self):
        if self._shutdown:
            return
        self._shutdown = True

        for name, process in self.launch_processes.items():
            if isinstance(process, subprocess.Popen):
                self._kill_proc(process, name)
            elif process is True and self.is_remote:
                # Remote process started via service -> send stop request
                try:
                    display = _DISPLAY_NAMES.get(name, name)
                    if self.stop_client.service_is_ready():
                        request = StopSystem.Request()
                        request.system_name = name
                        future = self.stop_client.call_async(request)
                        start = time.time()
                        while not future.done() and (time.time() - start) < 2.0:
                            rclpy.spin_once(self.node, timeout_sec=0.05)
                        self.log.info(f'Stop request sent for {display}')
                except Exception as e:
                    self.log.warning(f'Error sending stop for {name}: {e}')

        self.stop_navigation_client()

        # Destroy node (don't shutdown rclpy as it may be used by other components)
        try:
            self.node.destroy_node()
        except Exception:
            pass
