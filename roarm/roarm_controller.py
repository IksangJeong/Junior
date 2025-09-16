#!/usr/bin/env python3
"""
RoArm-M2-S Advanced Controller Module
RoArm-M2-S 로봇팔 고급 제어 모듈

주요 기능:
- HTTP/JSON API 통신
- 좌표 기반 제어
- 관절 각도 제어
- 그리퍼 제어
- 궤적 계획
- 안전 범위 검증
"""

import json
import time
import math
import requests
import threading
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum
import asyncio
import websockets

class ControlMode(Enum):
    """제어 모드"""
    JOINT = 1      # 관절 각도 제어
    COORDINATE = 2  # 좌표 제어
    GRIPPER = 3    # 그리퍼 제어

@dataclass
class Position3D:
    """3D 위치"""
    x: float
    y: float
    z: float

@dataclass
class JointAngles:
    """관절 각도"""
    joint1: float  # 베이스 회전
    joint2: float  # 어깨
    joint3: float  # 팔꿈치
    joint4: float  # 그리퍼

@dataclass
class WorkspaceLimit:
    """작업 공간 제한"""
    x_min: float = 100
    x_max: float = 500
    y_min: float = -400
    y_max: float = 400
    z_min: float = 80
    z_max: float = 400

class RoArmController:
    """RoArm-M2-S 고급 제어 클래스"""

    def __init__(self, robot_ip: str = "192.168.4.1", port: int = 80):
        self.robot_ip = robot_ip
        self.port = port
        self.base_url = f"http://{robot_ip}:{port}"

        # 현재 상태
        self.current_position = Position3D(200, 0, 150)
        self.current_joints = JointAngles(0, 0, 0, 0)
        self.gripper_closed = False

        # 설정값
        self.move_step = 10  # 이동 단위 (mm) - 부드러운 움직임을 위해 감소
        self.joint_step = 5  # 관절 이동 단위 (도)
        self.workspace = WorkspaceLimit()
        self.default_move_time = 500  # 기본 이동 시간 (ms) - 속도 향상

        # 안전 설정
        self.max_speed = 100  # mm/s
        self.acceleration_limit = 50  # mm/s²

        # 통신 설정
        self.timeout = 2  # 타임아웃 단축으로 빠른 응답
        self.retry_count = 1  # 재시도 횟수 감소로 속도 향상

        # 상태 모니터링
        self.is_connected = False
        self.last_command_time = 0
        self.command_queue = []

    def connect(self) -> bool:
        """로봇팔 연결 확인"""
        try:
            # RoArm-M2-S 웹 인터페이스 연결 테스트
            response = requests.get(f"{self.base_url}/", timeout=self.timeout)
            self.is_connected = response.status_code == 200
            if self.is_connected:
                print(f"RoArm-M2-S 연결 성공: {self.robot_ip}")
                return True
            else:
                print(f"RoArm-M2-S 연결 실패: HTTP {response.status_code}")
                return False
        except Exception as e:
            print(f"RoArm-M2-S 연결 실패: {e}")
            self.is_connected = False
            return False

    def send_command(self, command: Dict[str, Any]) -> bool:
        """로봇팔에 JSON 명령 전송"""
        if not self.is_connected:
            if not self.connect():
                return False

        for attempt in range(self.retry_count):
            try:
                # RoArm-M2-S 실제 API 엔드포인트: /js?json={JSON_DATA}
                json_data = json.dumps(command)
                url = f"{self.base_url}/js?json={json_data}"

                response = requests.get(url, timeout=self.timeout)

                if response.status_code == 200:
                    self.last_command_time = time.time()
                    return True
                else:
                    print(f"명령 전송 실패: HTTP {response.status_code}")

            except Exception as e:
                print(f"명령 전송 실패 (시도 {attempt + 1}/{self.retry_count}): {e}")

                if attempt < self.retry_count - 1:
                    time.sleep(0.1)  # 재시도 대기시간 단축

        return False

    def validate_position(self, position: Position3D) -> bool:
        """위치가 작업 공간 내에 있는지 확인"""
        return (self.workspace.x_min <= position.x <= self.workspace.x_max and
                self.workspace.y_min <= position.y <= self.workspace.y_max and
                self.workspace.z_min <= position.z <= self.workspace.z_max)

    def validate_joints(self, joints: JointAngles) -> bool:
        """관절 각도가 유효 범위 내에 있는지 확인"""
        return (-180 <= joints.joint1 <= 180 and
                -90 <= joints.joint2 <= 90 and
                -90 <= joints.joint3 <= 90 and
                0 <= joints.joint4 <= 90)

    def calculate_move_time(self, target_pos: Position3D) -> int:
        """이동 시간 계산"""
        distance = math.sqrt(
            (target_pos.x - self.current_position.x)**2 +
            (target_pos.y - self.current_position.y)**2 +
            (target_pos.z - self.current_position.z)**2
        )

        # 속도 제한 고려
        time_needed = max(distance / self.max_speed * 1000, self.default_move_time)
        return int(min(time_needed, 5000))  # 최대 5초

    def move_to_position(self, x: float, y: float, z: float, move_time: Optional[int] = None) -> bool:
        """지정된 좌표로 이동"""
        target_pos = Position3D(x, y, z)

        if not self.validate_position(target_pos):
            print(f"위치가 작업 공간을 벗어남: ({x}, {y}, {z})")
            return False

        # index.html에서 확인한 실제 좌표 제어 명령 (T:104) - 속도 최적화
        command = {
            "T": 104,
            "x": int(x),
            "y": int(y),
            "z": int(z),
            "t": 3.14,  # 기본 회전값
            "spd": 0.8  # 속도 향상 (0.25 → 0.8)
        }

        if self.send_command(command):
            self.current_position = target_pos
            print(f"이동 명령 전송: ({x}, {y}, {z})")
            return True

        return False

    def move_joints(self, joint1: float, joint2: float, joint3: float, joint4: Optional[float] = None, move_time: int = None) -> bool:
        """관절 각도로 이동 (라디안 단위)"""
        if joint4 is None:
            joint4 = self.current_joints.joint4

        # index.html에서 확인한 실제 관절 제어 명령 (T:102)
        command = {
            "T": 102,
            "base": joint1,      # 라디안 단위
            "shoulder": joint2,  # 라디안 단위
            "elbow": joint3,     # 라디안 단위
            "hand": joint4,      # 라디안 단위
            "spd": 0,
            "acc": 0
        }

        if self.send_command(command):
            target_joints = JointAngles(joint1, joint2, joint3, joint4)
            self.current_joints = target_joints
            print(f"관절 이동 명령 전송: base={joint1:.3f}, shoulder={joint2:.3f}, elbow={joint3:.3f}, hand={joint4:.3f}")
            return True

        return False

    def control_gripper(self, close: bool, force: int = 50) -> bool:
        """그리퍼 제어 - T:102 명령 사용 (직접 각도 제어)"""
        # information.md 문서 기준 T:102 명령 사용
        # hand 파라미터로 그리퍼 직접 제어
        # 닫힘: 1.08 라디안 (그리퍼가 잡는 상태)
        # 열림: 3.14 라디안 (기본 열린 상태)

        gripper_angle = 3.14 if close else 1.08  # 반대로 변경: 닫기=열림, 열기=닫힘

        # T:102 명령으로 모든 관절 제어 (현재 위치 유지, hand만 변경)
        command = {
            "T": 102,
            "base": self.current_joints.joint1,      # 현재 베이스 각도 유지
            "shoulder": self.current_joints.joint2,  # 현재 어깨 각도 유지
            "elbow": self.current_joints.joint3,     # 현재 팔꿈치 각도 유지
            "hand": gripper_angle,                   # 그리퍼만 변경
            "spd": 0,   # 최대 속도
            "acc": 10   # 부드러운 가감속
        }

        # 상태 변경 시에만 로그 출력
        if self.gripper_closed != close:
            # 실제 동작과 일치하도록 로그 수정
            actual_action = "열기" if close else "닫기"
            print(f"📡 그리퍼 T:102 명령: {actual_action} (hand={gripper_angle:.2f})")

        if self.send_command(command):
            self.gripper_closed = close
            self.current_joints.joint4 = gripper_angle
            # 실제 동작: close=True일 때 그리퍼 열림(3.14), close=False일 때 그리퍼 닫힘(1.08)
            status = "열림" if close else "닫힘"  # 반대로 표시
            print(f"✅ 그리퍼 {status} (각도: {gripper_angle:.2f} rad)")
            return True

        print(f"❌ 그리퍼 명령 전송 실패")
        return False

    def move_relative(self, dx: float, dy: float, dz: float) -> bool:
        """상대적 위치 이동"""
        new_x = self.current_position.x + dx
        new_y = self.current_position.y + dy
        new_z = self.current_position.z + dz

        return self.move_to_position(new_x, new_y, new_z)

    def move_forward(self, distance: Optional[float] = None) -> bool:
        """앞으로 이동 (X+)"""
        if distance is None:
            distance = self.move_step
        return self.move_relative(distance, 0, 0)

    def move_backward(self, distance: Optional[float] = None) -> bool:
        """뒤로 이동 (X-)"""
        if distance is None:
            distance = self.move_step
        return self.move_relative(-distance, 0, 0)

    def move_left(self, distance: Optional[float] = None) -> bool:
        """왼쪽으로 이동 (Y-)"""
        if distance is None:
            distance = self.move_step
        return self.move_relative(0, -distance, 0)

    def move_right(self, distance: Optional[float] = None) -> bool:
        """오른쪽으로 이동 (Y+)"""
        if distance is None:
            distance = self.move_step
        return self.move_relative(0, distance, 0)

    def move_up(self, distance: Optional[float] = None) -> bool:
        """위로 이동 (Z+)"""
        if distance is None:
            distance = self.move_step
        return self.move_relative(0, 0, distance)

    def move_down(self, distance: Optional[float] = None) -> bool:
        """아래로 이동 (Z-)"""
        if distance is None:
            distance = self.move_step
        return self.move_relative(0, 0, -distance)

    def home_position(self) -> bool:
        """홈 포지션으로 이동"""
        print("홈 포지션으로 이동 중...")
        # index.html에서 확인한 실제 홈 포지션 명령
        command = {
            "T": 102,
            "base": 0,
            "shoulder": 0,
            "elbow": 1.5707965,  # π/2 라디안
            "hand": 3.14,  # 3.14 라디안 (그리퍼 열림, 180도)
            "spd": 0,
            "acc": 0
        }

        if self.send_command(command):
            # 현재 위치 업데이트
            self.current_position = Position3D(200, 0, 150)
            self.current_joints = JointAngles(0, 0, 90, 62)  # 도 단위로 추정 (그리퍼 열림, 62도)
            self.gripper_closed = False  # 그리퍼 상태 업데이트
            return True
        return False

    def safe_position(self) -> bool:
        """안전 위치로 이동"""
        print("안전 위치로 이동 중...")
        return self.move_to_position(150, 0, 200, 1500)

    def emergency_stop(self) -> bool:
        """비상 정지"""
        print("비상 정지!")
        command = {"T": 210, "cmd": 0}  # 정지 명령 (브라우저 로그에서 확인한 형식)
        return self.send_command(command)

    def get_robot_status(self) -> Optional[Dict[str, Any]]:
        """로봇팔에서 실제 상태 조회"""
        if not self.is_connected:
            return None

        try:
            # index.html에서 확인한 상태 조회 명령 (T:105)
            command = {"T": 105}
            json_data = json.dumps(command)
            url = f"{self.base_url}/js?json={json_data}"

            response = requests.get(url, timeout=self.timeout)
            if response.status_code == 200:
                return response.json()
            else:
                print(f"상태 조회 실패: HTTP {response.status_code}")
                return None

        except Exception as e:
            print(f"상태 조회 실패: {e}")
            return None

    def get_status(self) -> Dict[str, Any]:
        """현재 상태 반환 (로컬 상태 + 로봇팔 상태)"""
        status = {
            "connected": self.is_connected,
            "position": {
                "x": self.current_position.x,
                "y": self.current_position.y,
                "z": self.current_position.z
            },
            "joints": {
                "joint1": self.current_joints.joint1,
                "joint2": self.current_joints.joint2,
                "joint3": self.current_joints.joint3,
                "joint4": self.current_joints.joint4
            },
            "gripper_closed": self.gripper_closed,
            "last_command_time": self.last_command_time
        }

        # 로봇팔에서 실제 상태 가져오기
        robot_status = self.get_robot_status()
        if robot_status:
            status["robot_feedback"] = robot_status

        return status

    def execute_trajectory(self, waypoints: List[Position3D], speeds: Optional[List[int]] = None) -> bool:
        """궤적 실행"""
        if not waypoints:
            return False

        if speeds is None:
            speeds = [self.default_move_time] * len(waypoints)

        print(f"궤적 실행 시작: {len(waypoints)}개 포인트")

        for i, (waypoint, speed) in enumerate(zip(waypoints, speeds)):
            if not self.validate_position(waypoint):
                print(f"웨이포인트 {i+1}이 작업 공간을 벗어남")
                return False

            if not self.move_to_position(waypoint.x, waypoint.y, waypoint.z, speed):
                print(f"웨이포인트 {i+1} 이동 실패")
                return False

            # 이동 완료 대기
            time.sleep(speed / 1000.0 + 0.1)

        print("궤적 실행 완료")
        return True

def test_controller():
    """컨트롤러 테스트"""
    print("RoArm-M2-S Controller Test")

    controller = RoArmController()

    # 연결 테스트
    if not controller.connect():
        print("로봇팔 연결 실패 - 시뮬레이션 모드로 실행")
        controller.is_connected = True

    # 홈 포지션
    controller.home_position()
    time.sleep(2)

    # 기본 동작 테스트
    print("기본 동작 테스트...")
    controller.move_forward()
    time.sleep(1)
    controller.move_left()
    time.sleep(1)
    controller.move_up()
    time.sleep(1)

    # 그리퍼 테스트
    print("그리퍼 테스트...")
    controller.control_gripper(True)
    time.sleep(1)
    controller.control_gripper(False)
    time.sleep(1)

    # 궤적 테스트
    print("궤적 테스트...")
    waypoints = [
        Position3D(220, 0, 150),
        Position3D(220, 50, 150),
        Position3D(220, 50, 120),
        Position3D(200, 0, 150)
    ]
    controller.execute_trajectory(waypoints)

    # 상태 출력
    status = controller.get_status()
    print(f"최종 상태: {json.dumps(status, indent=2)}")

if __name__ == "__main__":
    test_controller()