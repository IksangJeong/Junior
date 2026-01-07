#!/usr/bin/env python3
"""
5-DOF Robot Arm Pick System
- Color-based object detection
- Inverse kinematics for 5-DOF arm
- Serial communication with Arduino

Hardware:
- Raspberry Pi 4 (vision processing)
- Arduino (servo control)
- MG996R Servos x5 + Gripper
- 2D USB Camera (Eye-in-Hand)

Joint Configuration:
- J1 (pin 2): Base rotation (좌우)
- J2 (pin 4): Tilt (앞뒤)
- J3 (pin 6): Tilt (앞뒤)
- J4 (pin 8): Tilt (앞뒤)
- J5 (pin 10): Wrist rotation (손목 회전)
- Gripper (pin 12): Gripper (집게)
"""

import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time
import math
import platform
from enum import Enum
from scipy.optimize import minimize

# ============================================================================
# CONFIGURATION
# ============================================================================

# Serial Configuration
SERIAL_PORT = None  # Auto-detect (set manually if needed: '/dev/ttyUSB0', '/dev/cu.usbserial-*', 'COM3')
SERIAL_BAUD = 115200
SERIAL_TIMEOUT = 1.0


def find_arduino_port():
    """Arduino 시리얼 포트 자동 감지 (macOS/Linux/Windows)"""
    system = platform.system()
    ports = list(serial.tools.list_ports.comports())

    for port in ports:
        device = port.device
        desc = port.description.lower() if port.description else ""

        # Arduino 관련 키워드 확인
        if any(kw in desc for kw in ['arduino', 'ch340', 'ch341', 'ftdi', 'usb serial']):
            return device

        # 플랫폼별 패턴 매칭
        if system == 'Darwin':  # macOS
            if 'cu.usbserial' in device or 'cu.usbmodem' in device:
                return device
        elif system == 'Linux':
            if 'ttyUSB' in device or 'ttyACM' in device:
                return device
        elif system == 'Windows':
            if device.startswith('COM'):
                return device

    # 마지막으로 아무 포트나 반환
    if ports:
        return ports[0].device
    return None

# Camera Configuration
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# Color Detection (HSV range for RED)
# Red wraps around in HSV, so we use two ranges
COLOR_LOWER_1 = np.array([0, 100, 100])
COLOR_UPPER_1 = np.array([10, 255, 255])
COLOR_LOWER_2 = np.array([160, 100, 100])
COLOR_UPPER_2 = np.array([180, 255, 255])

# Object Detection
MIN_CONTOUR_AREA = 500
KNOWN_OBJECT_DIAMETER_MM = 40.0  # Known ball diameter for depth estimation

# Camera Parameters (approximate, calibrate for accuracy)
FOCAL_LENGTH_PX = 500.0  # Approximate focal length in pixels

# Robot Arm DH Parameters (MEASURE YOUR ROBOT AND UPDATE)
# Format: [a, alpha, d, theta_offset] for each joint
# Units: mm for lengths, radians for angles
# This is for 5-DOF arm: Base rotation + 3 tilt joints + wrist rotation
DH_PARAMS = [
    [0,      math.pi/2,   50,   0],    # J1: Base (좌우 회전)
    [100,    0,           0,    0],    # J2: Tilt (앞뒤)
    [100,    0,           0,    0],    # J3: Tilt (앞뒤)
    [80,     0,           0,    0],    # J4: Tilt (앞뒤)
    [0,      0,           50,   0],    # J5: Wrist rotation + gripper offset
]

# Joint Limits (degrees)
JOINT_LIMITS = [
    (0, 180),   # J1: Base
    (0, 180),   # J2: Tilt
    (0, 180),   # J3: Tilt
    (0, 180),   # J4: Tilt
    (0, 180),   # J5: Wrist
]

# Number of joints (excluding gripper)
NUM_JOINTS = 5

# Gripper
GRIPPER_OPEN = 90
GRIPPER_CLOSED = 30

# Safety
WORKSPACE_X = (-200, 200)
WORKSPACE_Y = (-200, 200)
WORKSPACE_Z = (0, 300)


# ============================================================================
# STATE MACHINE
# ============================================================================

class RobotState(Enum):
    IDLE = 0
    SEARCH = 1
    TRACK = 2
    APPROACH = 3
    ALIGN = 4
    GRAB = 5
    DONE = 6


# ============================================================================
# SERIAL COMMUNICATION
# ============================================================================

class SerialComm:
    def __init__(self, port=SERIAL_PORT, baud=SERIAL_BAUD):
        self.port = port
        self.baud = baud
        self.ser = None

    def connect(self):
        # 자동 감지
        if self.port is None:
            self.port = find_arduino_port()

        if self.port is None:
            print("[Error] Arduino를 찾을 수 없습니다")
            available = [p.device for p in serial.tools.list_ports.comports()]
            print(f"[Info] 사용 가능한 포트: {available if available else '없음'}")
            return False

        try:
            print(f"[Serial] {self.port} 연결 시도...")
            self.ser = serial.Serial(self.port, self.baud, timeout=SERIAL_TIMEOUT)
            time.sleep(2)  # Wait for Arduino reset
            # Read welcome message
            while self.ser.in_waiting:
                print(f"[Arduino] {self.ser.readline().decode().strip()}")
            print(f"[Serial] {self.port} 연결 성공!")
            return True
        except Exception as e:
            print(f"[Error] Serial connection failed: {e}")
            return False

    def disconnect(self):
        if self.ser:
            self.ser.close()

    def send_joints(self, j1, j2, j3, j4, j5, grip=None):
        """Send joint angles to Arduino (5 joints)"""
        cmd = f"J1:{int(j1)},J2:{int(j2)},J3:{int(j3)},J4:{int(j4)},J5:{int(j5)}"
        if grip is not None:
            cmd += f",G:{int(grip)}"
        cmd += "\n"

        if self.ser:
            self.ser.write(cmd.encode())
            time.sleep(0.05)
            response = self.ser.readline().decode().strip()
            return response
        return None

    def send_home(self):
        """Send HOME command"""
        if self.ser:
            self.ser.write(b"HOME\n")
            return self.ser.readline().decode().strip()
        return None

    def send_stop(self):
        """Send STOP command"""
        if self.ser:
            self.ser.write(b"STOP\n")
            return self.ser.readline().decode().strip()
        return None

    def get_status(self):
        """Get current position"""
        if self.ser:
            self.ser.write(b"STATUS\n")
            return self.ser.readline().decode().strip()
        return None


# ============================================================================
# ROBOT VIEW (로봇 시점 표시)
# ============================================================================

class RobotView:
    """로봇이 보는 시점을 향상된 UI로 표시하는 클래스"""

    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.window_name = "Robot View"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

    def draw(self, frame, detection, state, joints, gripper):
        """향상된 시각화 그리기"""
        view = frame.copy()

        # 1. 그리드 오버레이
        self._draw_grid(view)

        # 2. 중앙 십자선
        self._draw_crosshair(view)

        # 3. 탐지 결과 표시
        if detection:
            self._draw_detection(view, detection)

        # 4. 상태 패널
        self._draw_status_panel(view, state, detection)

        # 5. 관절 각도 표시
        self._draw_joint_display(view, joints, gripper)

        # 6. 깊이 인디케이터 (탐지 시)
        if detection:
            self._draw_depth_bar(view, detection)

        cv2.imshow(self.window_name, view)

    def _draw_grid(self, frame):
        """정렬용 그리드"""
        color = (50, 50, 50)
        # 수직선
        for i in range(1, 4):
            x = self.width * i // 4
            cv2.line(frame, (x, 0), (x, self.height), color, 1)
        # 수평선
        for i in range(1, 4):
            y = self.height * i // 4
            cv2.line(frame, (0, y), (self.width, y), color, 1)

    def _draw_crosshair(self, frame):
        """중앙 타겟 십자선"""
        cx, cy = self.width // 2, self.height // 2
        color = (0, 255, 255)  # Yellow
        cv2.line(frame, (cx - 30, cy), (cx + 30, cy), color, 2)
        cv2.line(frame, (cx, cy - 30), (cx, cy + 30), color, 2)
        cv2.circle(frame, (cx, cy), 5, color, 1)

    def _draw_detection(self, frame, detection):
        """탐지된 객체 표시"""
        cx, cy, radius = detection
        # 외곽 원
        cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
        # 중심점
        cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)
        # 중앙까지 연결선
        center = (self.width // 2, self.height // 2)
        cv2.line(frame, (int(cx), int(cy)), center, (0, 255, 0), 1)

    def _draw_status_panel(self, frame, state, detection):
        """상태 정보 패널 (좌상단)"""
        cv2.rectangle(frame, (5, 5), (200, 90), (0, 0, 0), -1)
        cv2.rectangle(frame, (5, 5), (200, 90), (0, 255, 255), 1)

        cv2.putText(frame, f"State: {state}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        status = "TARGET LOCKED" if detection else "SEARCHING..."
        color = (0, 255, 0) if detection else (0, 165, 255)
        cv2.putText(frame, status, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if detection:
            cx, cy, radius = detection
            cv2.putText(frame, f"Pos: ({int(cx)}, {int(cy)})", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    def _draw_joint_display(self, frame, joints, gripper):
        """관절 각도 표시 (하단)"""
        y = self.height - 25
        cv2.rectangle(frame, (5, y - 15), (self.width - 5, self.height - 5), (0, 0, 0), -1)

        grip_text = 'OPEN' if gripper > 60 else 'CLOSED'
        text = f"J1:{int(joints[0])} J2:{int(joints[1])} J3:{int(joints[2])} J4:{int(joints[3])} J5:{int(joints[4])} | Grip: {grip_text}"
        cv2.putText(frame, text, (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    def _draw_depth_bar(self, frame, detection):
        """깊이 바 인디케이터 (우측)"""
        cx, cy, radius = detection
        # 깊이 추정
        depth_mm = (FOCAL_LENGTH_PX * KNOWN_OBJECT_DIAMETER_MM) / (radius * 2) if radius > 0 else 0

        bar_x = self.width - 40
        bar_top, bar_bottom = 100, self.height - 100
        bar_height = bar_bottom - bar_top

        # 배경
        cv2.rectangle(frame, (bar_x - 5, bar_top - 20), (bar_x + 25, bar_bottom + 25), (0, 0, 0), -1)

        # 0-500mm 범위
        ratio = min(depth_mm / 500.0, 1.0)
        fill_h = int(bar_height * ratio)

        # 채움 바
        cv2.rectangle(frame, (bar_x, bar_bottom - fill_h), (bar_x + 20, bar_bottom), (255, 165, 0), -1)
        # 외곽선
        cv2.rectangle(frame, (bar_x, bar_top), (bar_x + 20, bar_bottom), (0, 255, 255), 1)

        # 레이블
        cv2.putText(frame, "DEPTH", (bar_x - 15, bar_top - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(frame, f"{int(depth_mm)}mm", (bar_x - 10, bar_bottom + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 165, 0), 1)

    def close(self):
        """윈도우 닫기"""
        cv2.destroyWindow(self.window_name)


# ============================================================================
# COLOR DETECTOR
# ============================================================================

class ColorDetector:
    def __init__(self):
        self.lower1 = COLOR_LOWER_1
        self.upper1 = COLOR_UPPER_1
        self.lower2 = COLOR_LOWER_2
        self.upper2 = COLOR_UPPER_2
        self.min_area = MIN_CONTOUR_AREA

    def detect(self, frame):
        """Detect colored object and return (cx, cy, radius) or None"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create mask (handle red wrap-around)
        mask1 = cv2.inRange(hsv, self.lower1, self.upper1)
        mask2 = cv2.inRange(hsv, self.lower2, self.upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None, mask

        # Find largest contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < self.min_area:
            return None, mask

        # Get centroid and radius
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None, mask

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        _, radius = cv2.minEnclosingCircle(largest)

        return (cx, cy, radius), mask

    def estimate_depth(self, radius_px):
        """Estimate depth using known object size"""
        if radius_px <= 0:
            return None
        diameter_px = radius_px * 2
        depth = (FOCAL_LENGTH_PX * KNOWN_OBJECT_DIAMETER_MM) / diameter_px
        return depth


# ============================================================================
# KINEMATICS (5-DOF)
# ============================================================================

class Kinematics:
    def __init__(self):
        self.dh_params = DH_PARAMS
        self.joint_limits = JOINT_LIMITS
        self.num_joints = NUM_JOINTS

    def dh_matrix(self, a, alpha, d, theta):
        """Create DH transformation matrix"""
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d   ],
            [0,   0,      0,     1   ]
        ])

    def forward_kinematics(self, joint_angles):
        """Calculate end-effector pose from joint angles (degrees)"""
        T = np.eye(4)

        for i, (a, alpha, d, offset) in enumerate(self.dh_params):
            theta = math.radians(joint_angles[i]) + offset
            T_i = self.dh_matrix(a, alpha, d, theta)
            T = T @ T_i

        return T

    def get_position(self, joint_angles):
        """Get end-effector position (x, y, z) from joint angles"""
        T = self.forward_kinematics(joint_angles)
        return T[0, 3], T[1, 3], T[2, 3]

    def inverse_kinematics(self, target_pos, initial_guess=None):
        """
        Numerical IK solver using optimization.
        target_pos: (x, y, z) target position in mm
        Returns: joint angles in degrees, or None if failed
        """
        if initial_guess is None:
            initial_guess = [90] * self.num_joints

        def cost_function(q):
            pos = self.get_position(q)
            error = np.array([
                pos[0] - target_pos[0],
                pos[1] - target_pos[1],
                pos[2] - target_pos[2]
            ])
            return np.sum(error ** 2)

        # Bounds from joint limits
        bounds = [(lim[0], lim[1]) for lim in self.joint_limits]

        result = minimize(
            cost_function,
            initial_guess,
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 1000, 'ftol': 1e-6}
        )

        if result.success and result.fun < 100:  # Error threshold
            return result.x
        return None

    def is_in_workspace(self, x, y, z):
        """Check if position is within workspace"""
        return (WORKSPACE_X[0] <= x <= WORKSPACE_X[1] and
                WORKSPACE_Y[0] <= y <= WORKSPACE_Y[1] and
                WORKSPACE_Z[0] <= z <= WORKSPACE_Z[1])


# ============================================================================
# COORDINATE TRANSFORM
# ============================================================================

class CoordinateTransform:
    def __init__(self):
        # Camera intrinsic matrix (approximate, should be calibrated)
        self.camera_matrix = np.array([
            [FOCAL_LENGTH_PX, 0, CAMERA_WIDTH / 2],
            [0, FOCAL_LENGTH_PX, CAMERA_HEIGHT / 2],
            [0, 0, 1]
        ])

        # Hand-eye transform (camera to gripper, should be calibrated)
        # Default: camera is aligned with gripper, offset in Z
        self.T_cam_to_gripper = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 50],  # 50mm offset
            [0, 0, 0, 1]
        ])

    def pixel_to_camera(self, u, v, depth):
        """Convert pixel coordinates to camera frame"""
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        return np.array([x, y, z, 1])

    def camera_to_base(self, point_cam, T_gripper_to_base):
        """Transform point from camera frame to robot base frame"""
        # Point in gripper frame
        point_gripper = self.T_cam_to_gripper @ point_cam
        # Point in base frame
        point_base = T_gripper_to_base @ point_gripper
        return point_base[:3]


# ============================================================================
# MAIN CONTROLLER
# ============================================================================

class RobotController:
    def __init__(self):
        self.state = RobotState.IDLE
        self.serial = SerialComm()
        self.detector = ColorDetector()
        self.kinematics = Kinematics()
        self.transform = CoordinateTransform()
        self.robot_view = RobotView(CAMERA_WIDTH, CAMERA_HEIGHT)

        self.cap = None
        self.current_joints = [90] * NUM_JOINTS  # 5 joints
        self.target_object = None
        self.track_count = 0
        self.gripper_state = GRIPPER_OPEN

    def initialize(self):
        """Initialize camera and serial connection"""
        # Camera
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)

        if not self.cap.isOpened():
            print("[Error] Camera failed to open")
            return False

        # Serial
        if not self.serial.connect():
            print("[Warning] Running without Arduino connection")

        # Go home
        self.go_home()
        return True

    def go_home(self):
        """Move to home position"""
        self.current_joints = [90] * NUM_JOINTS
        self.gripper_state = GRIPPER_OPEN
        if self.serial.ser:
            self.serial.send_home()

    def update_state_machine(self, detection):
        """Update state machine based on detection"""
        if self.state == RobotState.IDLE:
            # Waiting for start command
            pass

        elif self.state == RobotState.SEARCH:
            if detection:
                self.target_object = detection
                self.track_count = 1
                self.state = RobotState.TRACK
                print("[State] SEARCH -> TRACK")

        elif self.state == RobotState.TRACK:
            if detection:
                self.target_object = detection
                self.track_count += 1
                if self.track_count >= 10:  # Stable for 10 frames
                    self.state = RobotState.APPROACH
                    print("[State] TRACK -> APPROACH")
            else:
                self.track_count = 0
                self.state = RobotState.SEARCH
                print("[State] TRACK -> SEARCH (lost)")

        elif self.state == RobotState.APPROACH:
            if detection:
                self.target_object = detection
                cx, cy, radius = detection
                depth = self.detector.estimate_depth(radius)

                if depth and depth < 100:  # Close enough
                    self.state = RobotState.ALIGN
                    print("[State] APPROACH -> ALIGN")
                else:
                    # Move toward object
                    self.approach_object(detection)
            else:
                self.state = RobotState.SEARCH
                print("[State] APPROACH -> SEARCH (lost)")

        elif self.state == RobotState.ALIGN:
            if detection:
                cx, cy, radius = detection
                center_x = CAMERA_WIDTH // 2
                center_y = CAMERA_HEIGHT // 2

                # Check if centered
                if abs(cx - center_x) < 20 and abs(cy - center_y) < 20:
                    self.state = RobotState.GRAB
                    print("[State] ALIGN -> GRAB")
                else:
                    # Fine adjustment
                    self.align_to_object(detection)
            else:
                self.state = RobotState.SEARCH
                print("[State] ALIGN -> SEARCH (lost)")

        elif self.state == RobotState.GRAB:
            # Close gripper
            self.gripper_state = GRIPPER_CLOSED
            self.serial.send_joints(*self.current_joints, self.gripper_state)
            time.sleep(1)  # Wait for gripper
            self.state = RobotState.DONE
            print("[State] GRAB -> DONE")

        elif self.state == RobotState.DONE:
            print("[State] Pick complete!")
            self.state = RobotState.IDLE

    def approach_object(self, detection):
        """Move toward detected object"""
        cx, cy, radius = detection
        depth = self.detector.estimate_depth(radius)

        if not depth:
            return

        # Convert to camera coordinates
        point_cam = self.transform.pixel_to_camera(cx, cy, depth)

        # Get current gripper pose
        T_gripper = self.kinematics.forward_kinematics(self.current_joints)

        # Transform to base frame
        target_base = self.transform.camera_to_base(point_cam, T_gripper)

        # Check workspace
        if not self.kinematics.is_in_workspace(*target_base):
            print("[Warning] Target outside workspace")
            return

        # Solve IK
        new_joints = self.kinematics.inverse_kinematics(
            target_base, self.current_joints
        )

        if new_joints is not None:
            self.current_joints = list(new_joints)
            self.serial.send_joints(*self.current_joints, self.gripper_state)

    def align_to_object(self, detection):
        """Fine adjustment to center object"""
        cx, cy, _ = detection
        center_x = CAMERA_WIDTH // 2
        center_y = CAMERA_HEIGHT // 2

        # Simple proportional control
        dx = (cx - center_x) * 0.1
        dy = (cy - center_y) * 0.1

        # Adjust joints for fine control
        # J1 (base) for left/right, J3 for up/down
        self.current_joints[0] = np.clip(self.current_joints[0] + dx, 0, 180)
        self.current_joints[2] = np.clip(self.current_joints[2] - dy, 0, 180)

        self.serial.send_joints(*self.current_joints, self.gripper_state)

    def run(self):
        """Main loop"""
        print("=" * 50)
        print("5-DOF Robot Arm Pick System")
        print("=" * 50)
        print("Controls:")
        print("  SPACE - Start/Stop picking")
        print("  H     - Go home")
        print("  G     - Toggle gripper")
        print("  Q/ESC - Quit")
        print("=" * 50)

        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue

            # Detect object
            detection, mask = self.detector.detect(frame)

            # Update state machine
            self.update_state_machine(detection)

            # Draw visualization
            self.draw_ui(frame, detection, mask)

            # Robot View 업데이트 (향상된 시각화)
            self.robot_view.draw(
                frame,
                detection,
                self.state.name,
                self.current_joints,
                self.gripper_state
            )

            # Show windows
            cv2.imshow('Camera', frame)
            cv2.imshow('Mask', mask)

            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:  # Q or ESC
                break
            elif key == ord(' '):  # Space - toggle run
                if self.state == RobotState.IDLE:
                    self.state = RobotState.SEARCH
                    print("[State] IDLE -> SEARCH")
                else:
                    self.state = RobotState.IDLE
                    print("[State] -> IDLE")
            elif key == ord('h'):  # Home
                self.go_home()
                self.state = RobotState.IDLE
                print("[State] -> HOME")
            elif key == ord('g'):  # Toggle gripper
                if self.gripper_state == GRIPPER_OPEN:
                    self.gripper_state = GRIPPER_CLOSED
                else:
                    self.gripper_state = GRIPPER_OPEN
                self.serial.send_joints(*self.current_joints, self.gripper_state)
                print(f"[Gripper] {'CLOSED' if self.gripper_state == GRIPPER_CLOSED else 'OPEN'}")

        self.cleanup()

    def draw_ui(self, frame, detection, mask):
        """Draw UI overlay"""
        # State
        state_text = f"State: {self.state.name}"
        cv2.putText(frame, state_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2)

        # Detection
        if detection:
            cx, cy, radius = detection
            cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
            cv2.circle(frame, (int(cx), int(cy)), 3, (0, 0, 255), -1)

            depth = self.detector.estimate_depth(radius)
            if depth:
                cv2.putText(frame, f"Depth: {depth:.0f}mm", (int(cx) + 10, int(cy)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Crosshair (center)
        center_x = CAMERA_WIDTH // 2
        center_y = CAMERA_HEIGHT // 2
        cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (255, 0, 0), 1)
        cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (255, 0, 0), 1)

        # Joint angles
        joint_text = f"Joints: {[int(j) for j in self.current_joints]}"
        cv2.putText(frame, joint_text, (10, CAMERA_HEIGHT - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def cleanup(self):
        """Cleanup resources"""
        self.go_home()
        if self.cap:
            self.cap.release()
        self.serial.disconnect()
        self.robot_view.close()
        cv2.destroyAllWindows()
        print("Shutdown complete")


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    controller = RobotController()
    if controller.initialize():
        controller.run()
