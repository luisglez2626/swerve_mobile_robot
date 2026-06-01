# File: ~/ros2_ws/src/swerve_robot/swerve_gui/swerve_gui/gui_node.py

import sys
import os
import math
import csv
import subprocess
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from nav2_msgs.action import NavigateToPose

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QSlider, QLabel, QPushButton, 
                             QGroupBox, QComboBox)
from PyQt6.QtCore import Qt, QPointF, pyqtSignal, QThread
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont

def euler_from_quaternion(x, y, z, w):
    """Convert a quaternion into euler angles (roll, pitch, yaw)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class VirtualJoystick(QWidget):
    joystick_moved = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.joystick_center = QPointF(100, 100)
        self.handle_pos = QPointF(100, 100)
        self.radius = 80
        self.handle_radius = 20
        self.pressed = False

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        painter.setPen(QPen(Qt.GlobalColor.black, 2))
        painter.setBrush(QBrush(QColor(220, 220, 220)))
        painter.drawEllipse(self.joystick_center, self.radius, self.radius)
        
        painter.setBrush(QBrush(QColor(50, 100, 200)))
        painter.drawEllipse(self.handle_pos, self.handle_radius, self.handle_radius)

    def update_position(self, pos):
        vec = pos - self.joystick_center
        dist = math.hypot(vec.x(), vec.y())
        
        if dist > self.radius:
            vec = vec * (self.radius / dist)
            self.handle_pos = self.joystick_center + vec
        else:
            self.handle_pos = pos

        normalized_x = -vec.y() / self.radius 
        normalized_y = -vec.x() / self.radius 
        
        self.joystick_moved.emit(normalized_x, normalized_y)
        self.update()

    def mousePressEvent(self, event):
        self.pressed = True
        self.update_position(event.position())

    def mouseMoveEvent(self, event):
        if self.pressed:
            self.update_position(event.position())

    def mouseReleaseEvent(self, event):
        self.pressed = False
        self.handle_pos = self.joystick_center
        self.joystick_moved.emit(0.0, 0.0)
        self.update()


class ROS2Thread(QThread):
    pose_signal = pyqtSignal(float, float, float)
    vel_signal = pyqtSignal(float, float, float)
    wheel_signal = pyqtSignal(list)
    nav_status_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        rclpy.init()
        self.node = rclpy.create_node('swerve_gui_node')
        self.node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joint_sub = self.node.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.tf_timer = self.node.create_timer(0.1, self.tf_callback)

        self.data_buffer = deque(maxlen=750)
        self.current_cmd = {'vx': 0.0, 'vy': 0.0, 'omega': 0.0}

    def run(self):
        rclpy.spin(self.node)

    def tf_callback(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = euler_from_quaternion(
                t.transform.rotation.x, t.transform.rotation.y, 
                t.transform.rotation.z, t.transform.rotation.w)
            self.pose_signal.emit(x, y, yaw)
        except TransformException:
            pass 

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        self.vel_signal.emit(vx, vy, omega)
        
        sim_time = self.node.get_clock().now().nanoseconds / 1e9
        self.data_buffer.append({
            'timestamp': sim_time,
            'cmd_vx': self.current_cmd['vx'],
            'cmd_vy': self.current_cmd['vy'],
            'cmd_omega': self.current_cmd['omega'],
            'odom_vx': vx,
            'odom_vy': vy,
            'odom_omega': omega
        })

    def joint_callback(self, msg):
        wheel_joints = ['fl_wheel_joint', 'fr_wheel_joint', 'rl_wheel_joint', 'rr_wheel_joint']
        speeds = []
        for joint in wheel_joints:
            if joint in msg.name:
                idx = msg.name.index(joint)
                speeds.append(msg.velocity[idx])
        if len(speeds) == 4:
            self.wheel_signal.emit(speeds)

    def publish_twist(self, linear_x, linear_y, angular_z):
        self.current_cmd['vx'] = float(linear_x)
        self.current_cmd['vy'] = float(linear_y)
        self.current_cmd['omega'] = float(angular_z)
        
        msg = Twist()
        msg.linear.x = self.current_cmd['vx']
        msg.linear.y = self.current_cmd['vy']
        msg.angular.z = self.current_cmd['omega']
        self.cmd_pub.publish(msg)

    def send_nav2_goal(self, x, y, yaw):
        self.nav_status_signal.emit("Waiting for Nav2 server...")
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.nav_status_signal.emit("ERROR: Nav2 Server Offline.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.nav_status_signal.emit(f"Sending goal to ({x:.2f}, {y:.2f})...")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.nav_status_signal.emit("Goal Rejected by Nav2.")
            return

        self.nav_status_signal.emit("Navigating to Goal...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4: 
            self.nav_status_signal.emit("Goal Reached Successfully!")
        elif status == 5: 
            self.nav_status_signal.emit("Goal Canceled.")
        elif status == 6: 
            self.nav_status_signal.emit("Goal Aborted (Blocked/Failed).")

    def save_log(self, filename="swerve_log.csv"):
        data_copy = list(self.data_buffer)
        if not data_copy:
            return False
            
        keys = data_copy[0].keys()
        with open(filename, 'w', newline='') as output_file:
            dict_writer = csv.DictWriter(output_file, fieldnames=keys)
            dict_writer.writeheader()
            dict_writer.writerows(data_copy)
        return True

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Swerve Mobile Robot - Operator Dashboard")
        self.resize(850, 450)
        
        self.waypoints = {
            "Select Destination...": None,
            "Corner Level 4 South West": (-12.0, -12.0, 0.0),
            "Corner Level 4 South East": (12.0, -12.0, 0.0),
            "Corner Level 4 North West": (-12.0, 12.0, 0.0),
            "Corner Level 4 North East": (12.0, 12.0, 0.0),
            "Alley East": (0.0, -9.0, 0.0),
            "Alley North East": (9.0, -9.0, 0.0),
            "Corner Level 3 North West": (9.0, 9.0, 0.0),
            "Corner Level 3 North East": (9.0, -6.0, 0.0)
        }

        self.ros_thread = ROS2Thread()
        self.ros_thread.pose_signal.connect(self.update_pose_ui)
        self.ros_thread.vel_signal.connect(self.update_vel_ui)
        self.ros_thread.wheel_signal.connect(self.update_wheel_ui)
        self.ros_thread.nav_status_signal.connect(self.update_nav_status)
        self.ros_thread.start()

        self.current_v_x = 0.0
        self.current_v_y = 0.0
        self.current_omega = 0.0
        self.max_speed = 1.0 
        self.max_turn = 2.0  

        self.init_ui()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # ---------------- LEFT: Teleoperation ----------------
        teleop_group = QGroupBox("Manual Teleoperation")
        teleop_layout = QVBoxLayout()
        
        self.joystick = VirtualJoystick()
        self.joystick.joystick_moved.connect(self.on_joystick_moved)
        teleop_layout.addWidget(self.joystick, alignment=Qt.AlignmentFlag.AlignCenter)

        yaw_layout = QHBoxLayout()
        yaw_layout.addWidget(QLabel("Yaw:"))
        self.yaw_slider = QSlider(Qt.Orientation.Horizontal)
        self.yaw_slider.setMinimum(-100)
        self.yaw_slider.setMaximum(100)
        self.yaw_slider.setValue(0)
        self.yaw_slider.valueChanged.connect(self.on_yaw_changed)
        self.yaw_slider.sliderReleased.connect(self.on_yaw_released)
        yaw_layout.addWidget(self.yaw_slider)
        teleop_layout.addLayout(yaw_layout)

        self.btn_save_log = QPushButton("Save Last 15s Kinematic Log")
        self.btn_save_log.clicked.connect(self.on_save_log_clicked)
        teleop_layout.addWidget(self.btn_save_log)

        self.btn_save_map = QPushButton("Save SLAM Map")
        self.btn_save_map.clicked.connect(self.on_save_map_clicked)
        teleop_layout.addWidget(self.btn_save_map)
        
        teleop_group.setLayout(teleop_layout)
        main_layout.addWidget(teleop_group)

        # ---------------- MIDDLE: Navigation ----------------
        nav_group = QGroupBox("Autonomous Navigation")
        nav_layout = QVBoxLayout()
        
        self.lbl_nav_status = QLabel("Status: Idle")
        self.lbl_nav_status.setStyleSheet("font-weight: bold; color: blue;")
        nav_layout.addWidget(self.lbl_nav_status)
        
        self.btn_home = QPushButton("🏠 SEND HOME")
        self.btn_home.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold; padding: 10px;")
        self.btn_home.clicked.connect(self.on_home_clicked)
        nav_layout.addWidget(self.btn_home)

        nav_layout.addWidget(QLabel("Send to Waypoint:"))
        self.combo_waypoints = QComboBox()
        self.combo_waypoints.addItems(self.waypoints.keys())
        nav_layout.addWidget(self.combo_waypoints)

        self.btn_send_wp = QPushButton("Deploy to Waypoint")
        self.btn_send_wp.clicked.connect(self.on_waypoint_clicked)
        nav_layout.addWidget(self.btn_send_wp)
        
        nav_layout.addStretch()
        nav_group.setLayout(nav_layout)
        main_layout.addWidget(nav_group)

        # ---------------- RIGHT: Live Dashboard ----------------
        dash_group = QGroupBox("Live Robot Diagnostics")
        dash_layout = QHBoxLayout()
        
        mono_font = QFont("Courier")
        mono_font.setStyleHint(QFont.StyleHint.Monospace)

        # Column 1: Global Position
        col1 = QVBoxLayout()
        col1.addWidget(QLabel("<b>Global Position</b>"))
        self.lbl_x = QLabel("X:   0.00 m")
        self.lbl_y = QLabel("Y:   0.00 m")
        self.lbl_yaw = QLabel("Yaw: 0.00 rad")
        for lbl in [self.lbl_x, self.lbl_y, self.lbl_yaw]:
            lbl.setFont(mono_font)
            lbl.setMinimumWidth(120)
            col1.addWidget(lbl)
        col1.addStretch()

        # Column 2: Chassis Velocity
        col2 = QVBoxLayout()
        col2.addWidget(QLabel("<b>Chassis Velocity</b>"))
        self.lbl_vx = QLabel("Vx:  0.00 m/s")
        self.lbl_vy = QLabel("Vy:  0.00 m/s")
        self.lbl_omega = QLabel("Wz:  0.00 rad/s")
        for lbl in [self.lbl_vx, self.lbl_vy, self.lbl_omega]:
            lbl.setFont(mono_font)
            lbl.setMinimumWidth(130)
            col2.addWidget(lbl)
        col2.addStretch()

        # Column 3: Wheel Speeds
        col3 = QVBoxLayout()
        col3.addWidget(QLabel("<b>Wheel Speeds</b>"))
        self.lbl_w_fl = QLabel("FL:  0.00")
        self.lbl_w_fr = QLabel("FR:  0.00")
        self.lbl_w_rl = QLabel("RL:  0.00")
        self.lbl_w_rr = QLabel("RR:  0.00")
        for lbl in [self.lbl_w_fl, self.lbl_w_fr, self.lbl_w_rl, self.lbl_w_rr]:
            lbl.setFont(mono_font)
            lbl.setMinimumWidth(90)
            col3.addWidget(lbl)
        col3.addStretch()

        dash_layout.addLayout(col1)
        dash_layout.addLayout(col2)
        dash_layout.addLayout(col3)
        dash_group.setLayout(dash_layout)
        main_layout.addWidget(dash_group)

    def update_pose_ui(self, x, y, yaw):
        self.lbl_x.setText(f"X:  {x:5.2f} m")
        self.lbl_y.setText(f"Y:  {y:5.2f} m")
        self.lbl_yaw.setText(f"Yaw:{yaw:5.2f} rad")

    def update_vel_ui(self, vx, vy, omega):
        self.lbl_vx.setText(f"Vx: {vx:5.2f} m/s")
        self.lbl_vy.setText(f"Vy: {vy:5.2f} m/s")
        self.lbl_omega.setText(f"Wz: {omega:5.2f} rad/s")

    def update_wheel_ui(self, speeds):
        self.lbl_w_fl.setText(f"FL: {speeds[0]:5.2f}")
        self.lbl_w_fr.setText(f"FR: {speeds[1]:5.2f}")
        self.lbl_w_rl.setText(f"RL: {speeds[2]:5.2f}")
        self.lbl_w_rr.setText(f"RR: {speeds[3]:5.2f}")

    def update_nav_status(self, status_msg):
        self.lbl_nav_status.setText(f"Status: {status_msg}")

    def on_home_clicked(self):
        self.ros_thread.send_nav2_goal(0.0, 0.0, 0.0)

    def on_waypoint_clicked(self):
        selection = self.combo_waypoints.currentText()
        wp = self.waypoints.get(selection)
        if wp is not None:
            self.ros_thread.send_nav2_goal(wp[0], wp[1], wp[2])

    def on_joystick_moved(self, x, y):
        self.current_v_x = x * self.max_speed
        self.current_v_y = y * self.max_speed
        self.ros_thread.publish_twist(self.current_v_x, self.current_v_y, self.current_omega)

    def on_yaw_changed(self, value):
        self.current_omega = -(value / 100.0) * self.max_turn
        self.ros_thread.publish_twist(self.current_v_x, self.current_v_y, self.current_omega)

    def on_yaw_released(self):
        self.yaw_slider.setValue(0)
        self.current_omega = 0.0
        self.ros_thread.publish_twist(self.current_v_x, self.current_v_y, self.current_omega)

    def on_save_log_clicked(self):
        if self.ros_thread.save_log():
            self.lbl_nav_status.setText("Log saved to swerve_log.csv")

    def on_save_map_clicked(self):
        self.lbl_nav_status.setText("Saving map...")
        QApplication.processEvents()
        try:
            map_dir = os.path.expanduser("~/ros2_ws/src/swerve_robot/swerve_navigation/maps")
            os.makedirs(map_dir, exist_ok=True)
            map_path = os.path.join(map_dir, "maze_map")
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path],
                check=True, capture_output=True, text=True
            )
            self.lbl_nav_status.setText(f"Map saved!")
        except Exception:
            self.lbl_nav_status.setText(f"Map save failed.")

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.ros_thread.wait()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()