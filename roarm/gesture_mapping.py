#!/usr/bin/env python3
"""
Gesture-to-Robot Mapping System
제스처-로봇팔 동작 매핑 시스템

주요 기능:
- 제스처와 로봇 동작 매핑
- 동작 큐 관리
- 안전 검증
- 사용자 설정 관리
"""

import time
import json
import threading
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass, asdict
from enum import Enum
from gesture_detector import GestureType
from roarm_controller import RoArmController, Position3D

class ActionType(Enum):
    """로봇 동작 타입"""
    MOVE = "move"
    GRIPPER = "gripper"
    HOME = "home"
    SAFE = "safe"
    STOP = "stop"

@dataclass
class GestureAction:
    """제스처 동작 정의"""
    gesture: GestureType
    action_type: ActionType
    parameters: Dict[str, Any]
    cooldown: float = 1.0  # 쿨다운 시간 (초)
    enabled: bool = True

@dataclass
class ActionResult:
    """동작 실행 결과"""
    success: bool
    message: str
    timestamp: float
    duration: float

class GestureActionMapper:
    """제스처-동작 매핑 관리 클래스"""

    def __init__(self, controller: RoArmController):
        self.controller = controller
        self.action_mappings: Dict[GestureType, GestureAction] = {}
        self.last_action_time: Dict[GestureType, float] = {}
        self.action_history: List[ActionResult] = []
        self.max_history = 100

        # 기본 매핑 설정
        self._setup_default_mappings()

        # 동작 실행을 위한 스레드
        self.action_queue = []
        self.queue_lock = threading.Lock()
        self.worker_thread = threading.Thread(target=self._action_worker, daemon=True)
        self.worker_thread.start()

        # 안전 설정
        self.safety_enabled = True
        self.emergency_stop_gesture = GestureType.FIST
        self.max_queue_size = 10

    def _setup_default_mappings(self):
        """기본 제스처 매핑 설정"""
        default_mappings = [
            GestureAction(
                gesture=GestureType.PINCH,
                action_type=ActionType.GRIPPER,
                parameters={"toggle": True},
                cooldown=1.5
            ),
            GestureAction(
                gesture=GestureType.FORWARD,
                action_type=ActionType.MOVE,
                parameters={"direction": "forward", "distance": 10},
                cooldown=0.2
            ),
            GestureAction(
                gesture=GestureType.BACKWARD,
                action_type=ActionType.MOVE,
                parameters={"direction": "backward", "distance": 10},
                cooldown=0.2
            ),
            GestureAction(
                gesture=GestureType.LEFT,
                action_type=ActionType.MOVE,
                parameters={"direction": "left", "distance": 10},
                cooldown=0.2
            ),
            GestureAction(
                gesture=GestureType.RIGHT,
                action_type=ActionType.MOVE,
                parameters={"direction": "right", "distance": 10},
                cooldown=0.2
            ),
            GestureAction(
                gesture=GestureType.UP,
                action_type=ActionType.MOVE,
                parameters={"direction": "up", "distance": 10},
                cooldown=0.2
            ),
            GestureAction(
                gesture=GestureType.DOWN,
                action_type=ActionType.MOVE,
                parameters={"direction": "down", "distance": 10},
                cooldown=0.2
            ),
            GestureAction(
                gesture=GestureType.OPEN_HAND,
                action_type=ActionType.HOME,
                parameters={},
                cooldown=3.0
            ),
            GestureAction(
                gesture=GestureType.FIST,
                action_type=ActionType.STOP,
                parameters={},
                cooldown=0.5
            )
        ]

        for mapping in default_mappings:
            self.action_mappings[mapping.gesture] = mapping

    def add_gesture_mapping(self, gesture_action: GestureAction):
        """제스처 매핑 추가"""
        self.action_mappings[gesture_action.gesture] = gesture_action
        print(f"제스처 매핑 추가: {gesture_action.gesture.value} -> {gesture_action.action_type.value}")

    def remove_gesture_mapping(self, gesture: GestureType):
        """제스처 매핑 제거"""
        if gesture in self.action_mappings:
            del self.action_mappings[gesture]
            print(f"제스처 매핑 제거: {gesture.value}")

    def enable_gesture(self, gesture: GestureType, enabled: bool = True):
        """제스처 활성화/비활성화"""
        if gesture in self.action_mappings:
            self.action_mappings[gesture].enabled = enabled
            status = "활성화" if enabled else "비활성화"
            print(f"제스처 {gesture.value} {status}")

    def can_execute_action(self, gesture: GestureType) -> bool:
        """동작 실행 가능 여부 확인"""
        if gesture not in self.action_mappings:
            return False

        action = self.action_mappings[gesture]

        # 활성화 확인
        if not action.enabled:
            return False

        # 쿨다운 확인
        current_time = time.time()
        last_time = self.last_action_time.get(gesture, 0)

        if current_time - last_time < action.cooldown:
            return False

        # 큐 크기 확인
        with self.queue_lock:
            if len(self.action_queue) >= self.max_queue_size:
                return False

        return True

    def execute_gesture(self, gesture: GestureType) -> bool:
        """제스처 실행"""
        if not self.can_execute_action(gesture):
            return False

        # 비상 정지 확인
        if self.safety_enabled and gesture == self.emergency_stop_gesture:
            return self._execute_emergency_stop()

        action = self.action_mappings[gesture]

        # 동작 큐에 추가
        with self.queue_lock:
            self.action_queue.append((gesture, action, time.time()))

        self.last_action_time[gesture] = time.time()
        return True

    def _execute_emergency_stop(self) -> bool:
        """비상 정지 실행"""
        print("🚨 비상 정지 실행!")

        # 큐 비우기
        with self.queue_lock:
            self.action_queue.clear()

        # 로봇 정지
        success = self.controller.emergency_stop()

        result = ActionResult(
            success=success,
            message="Emergency Stop",
            timestamp=time.time(),
            duration=0.1
        )
        self._add_to_history(result)

        return success

    def _action_worker(self):
        """동작 실행 워커 스레드"""
        while True:
            action_item = None

            with self.queue_lock:
                if self.action_queue:
                    action_item = self.action_queue.pop(0)

            if action_item:
                gesture, action, queue_time = action_item
                self._execute_action(gesture, action, queue_time)
            else:
                time.sleep(0.01)  # 10ms 대기

    def _execute_action(self, gesture: GestureType, action: GestureAction, queue_time: float):
        """실제 동작 실행"""
        start_time = time.time()
        success = False
        message = ""

        try:
            if action.action_type == ActionType.MOVE:
                success = self._execute_move_action(action.parameters)
                message = f"Move {action.parameters.get('direction', 'unknown')}"

            elif action.action_type == ActionType.GRIPPER:
                success = self._execute_gripper_action(action.parameters)
                message = "Gripper control"

            elif action.action_type == ActionType.HOME:
                success = self.controller.home_position()
                message = "Home position"

            elif action.action_type == ActionType.SAFE:
                success = self.controller.safe_position()
                message = "Safe position"

            elif action.action_type == ActionType.STOP:
                success = self.controller.emergency_stop()
                message = "Stop"

            else:
                message = f"Unknown action type: {action.action_type}"

        except Exception as e:
            message = f"Action execution error: {e}"

        duration = time.time() - start_time

        result = ActionResult(
            success=success,
            message=message,
            timestamp=start_time,
            duration=duration
        )

        self._add_to_history(result)

        status = "✅" if success else "❌"
        print(f"{status} {gesture.value} -> {message} ({duration:.2f}s)")

    def _execute_move_action(self, parameters: Dict[str, Any]) -> bool:
        """이동 동작 실행"""
        direction = parameters.get("direction", "")
        distance = parameters.get("distance", 15)

        if direction == "forward":
            return self.controller.move_forward(distance)
        elif direction == "backward":
            return self.controller.move_backward(distance)
        elif direction == "left":
            return self.controller.move_left(distance)
        elif direction == "right":
            return self.controller.move_right(distance)
        elif direction == "up":
            return self.controller.move_up(distance)
        elif direction == "down":
            return self.controller.move_down(distance)

        return False

    def _execute_gripper_action(self, parameters: Dict[str, Any]) -> bool:
        """그리퍼 동작 실행"""
        if parameters.get("toggle", False):
            # 토글 모드
            return self.controller.control_gripper(not self.controller.gripper_closed)
        else:
            # 직접 제어
            close = parameters.get("close", True)
            force = parameters.get("force", 50)
            return self.controller.control_gripper(close, force)

    def _add_to_history(self, result: ActionResult):
        """히스토리에 결과 추가"""
        self.action_history.append(result)

        # 히스토리 크기 제한
        if len(self.action_history) > self.max_history:
            self.action_history = self.action_history[-self.max_history:]

    def get_statistics(self) -> Dict[str, Any]:
        """통계 정보 반환"""
        if not self.action_history:
            return {"total_actions": 0}

        total_actions = len(self.action_history)
        successful_actions = sum(1 for r in self.action_history if r.success)
        success_rate = successful_actions / total_actions * 100

        recent_actions = self.action_history[-10:]
        avg_duration = sum(r.duration for r in recent_actions) / len(recent_actions)

        return {
            "total_actions": total_actions,
            "successful_actions": successful_actions,
            "success_rate": f"{success_rate:.1f}%",
            "average_duration": f"{avg_duration:.2f}s",
            "queue_size": len(self.action_queue)
        }

    def save_mappings(self, filename: str):
        """매핑 설정 저장"""
        mappings_data = {}
        for gesture, action in self.action_mappings.items():
            mappings_data[gesture.value] = asdict(action)

        with open(filename, 'w') as f:
            json.dump(mappings_data, f, indent=2)

        print(f"매핑 설정 저장: {filename}")

    def load_mappings(self, filename: str):
        """매핑 설정 로드"""
        try:
            with open(filename, 'r') as f:
                mappings_data = json.load(f)

            self.action_mappings.clear()

            for gesture_name, action_data in mappings_data.items():
                gesture = GestureType(gesture_name)
                action_type = ActionType(action_data['action_type'])

                action = GestureAction(
                    gesture=gesture,
                    action_type=action_type,
                    parameters=action_data['parameters'],
                    cooldown=action_data.get('cooldown', 1.0),
                    enabled=action_data.get('enabled', True)
                )

                self.action_mappings[gesture] = action

            print(f"매핑 설정 로드: {filename}")

        except Exception as e:
            print(f"매핑 설정 로드 실패: {e}")
            self._setup_default_mappings()

def test_gesture_mapping():
    """제스처 매핑 테스트"""
    print("Gesture Mapping System Test")

    # 컨트롤러 생성 (시뮬레이션 모드)
    controller = RoArmController()
    controller.is_connected = True

    # 매핑 시스템 생성
    mapper = GestureActionMapper(controller)

    # 테스트 제스처들
    test_gestures = [
        GestureType.FORWARD,
        GestureType.LEFT,
        GestureType.PINCH,
        GestureType.BACKWARD,
        GestureType.RIGHT,
        GestureType.OPEN_HAND,
        GestureType.FIST
    ]

    print("제스처 실행 테스트...")
    for gesture in test_gestures:
        if gesture in mapper.action_mappings:
            success = mapper.execute_gesture(gesture)
            print(f"{gesture.value}: {'성공' if success else '실패'}")
            time.sleep(1.5)

    # 통계 출력
    time.sleep(2)  # 모든 동작 완료 대기
    stats = mapper.get_statistics()
    print(f"\n실행 통계: {json.dumps(stats, indent=2)}")

    # 매핑 저장 테스트
    mapper.save_mappings("gesture_mappings.json")

if __name__ == "__main__":
    test_gesture_mapping()