"""
Hand Tracking Robot Controller
웹캠으로 손을 인식하고 로봇 손을 제어하는 프로그램
"""

import cv2
import mediapipe as mp
import numpy as np
import serial
import time
import math
from collections import deque

class HandTracker:
    def __init__(self, port='/dev/cu.usbmodem1201', baudrate=9600):
        """
        HandTracker 초기화

        Args:
            port: Arduino 시리얼 포트
            baudrate: 통신 속도
        """
        # MediaPipe 초기화
        # MediaPipe는 Google에서 개발한 머신러닝 기반 컴퓨터 비전 프레임워크
        # 21개 손 랜드마크를 실시간으로 추적하는 모델 사용
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,  # False = 비디오 스트림 모드 (프레임 간 추적)
            max_num_hands=1,          # 한 손만 감지 (성능 최적화)
            min_detection_confidence=0.8,  # 초기 감지 임계값 (0.8 = 80% 확신도)
            min_tracking_confidence=0.7    # 추적 유지 임계값 (0.7 = 약간 냮춰 부드러운 추적)
        )
        # 시행착오: 처음엔 0.5/0.5 사용 → 떨림이 심함
        # 0.9/0.9로 올렸더니 손 인식을 너무 자주 놓침
        # 현재값이 detection과 tracking의 균형점
        self.mp_drawing = mp.solutions.drawing_utils

        # Arduino 시리얼 통신 초기화
        try:
            self.arduino = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # Arduino 초기화 대기
            print(f"Arduino connected on {port}")
        except:
            print(f"Warning: Could not connect to Arduino on {port}")
            self.arduino = None

        # 손가락 관절 인덱스 정의 (MediaPipe 기준)
        # MediaPipe는 손을 21개 랜드마크로 표현:
        # 0: 손목 (wrist)
        # 1-4: 엄지 (CMC, MCP, IP, TIP)
        # 5-8: 검지 (MCP, PIP, DIP, TIP)
        # 9-12: 중지, 13-16: 약지, 17-20: 새끼
        self.finger_tips = [4, 8, 12, 16, 20]  # 엄지, 검지, 중지, 약지, 소지 끝
        self.finger_pip = [3, 6, 10, 14, 18]   # PIP 관절 (엄지는 IP 관절)
        self.finger_mcp = [2, 5, 9, 13, 17]    # 각 손가락 MCP 관절
        self.wrist = 0  # 손목 랜드마크

        # 스무딩을 위한 각도 히스토리
        # deque는 큐 자료구조로 FIFO(First In First Out) 방식
        # maxlen=5: 최근 5개 값만 유지, 오래된 값 자동 삭제
        self.angle_history = [deque(maxlen=5) for _ in range(5)]
        self.last_angles = [90] * 5  # 초기 각도 (중간값 = 반쯤 접힌 상태)

        # 모터 제어 최적화
        self.min_angle_change = 5  # 최소 각도 변화량 (떨림 방지)
        self.send_interval = 0.1   # 전송 간격 (초) - 10Hz, 서보모터 반응속도 고려
        self.last_send_time = 0

        # 제스처 인식
        self.gesture_history = deque(maxlen=10)
        self.current_gesture = "Unknown"

        # 엄지 적응형 임계값
        self.thumb_calibration_frames = 30  # 보정용 프레임 수
        self.thumb_baseline_angles = deque(maxlen=self.thumb_calibration_frames)
        self.thumb_calibrated = False

        # 손가락별 특성 설정
        # 실제 손가락의 물리적 특성을 반영한 파라미터
        # curl_threshold: 접힘으로 판단하는 각도 임계값 (작을수록 더 접혀야 함)
        # sensitivity: 민감도 (1.0 = 100%) - 약지, 새끼는 연동되기 쉬워서 낮춤
        #
        # 재밌는 사실: 약지와 새끼손가락은 힘줄이 연결되어 독립 움직임이 어려움!
        # 그래서 sensitivity를 낮춰서 오인식 방지
        self.finger_characteristics = {
            0: {"name": "thumb", "curl_threshold": 120, "sensitivity": 1.0},
            1: {"name": "index", "curl_threshold": 120, "sensitivity": 1.0},
            2: {"name": "middle", "curl_threshold": 110, "sensitivity": 0.9},
            3: {"name": "ring", "curl_threshold": 105, "sensitivity": 0.8},  # 약지는 독립성 낮음
            4: {"name": "pinky", "curl_threshold": 100, "sensitivity": 0.7}  # 새끼도 마찬가지
        }

        # 개별 손가락 제어 (컨텍스트 기반 보정 제거됨)

        # 즉시 제어 시스템 (학습 과정 제거)
        self.extended_state_learned = True  # 즉시 제어 활성화
        # 손가락별 기본 최대 펼침 거리 (경험적 기본값)
        self.max_finger_distances = [0.12, 0.15, 0.16, 0.15, 0.13]  # 엄지, 검지, 중지, 약지, 소지

        # 손가락 신뢰도 추적 (겹침 감지용)
        self.finger_confidence_history = [deque(maxlen=10) for _ in range(5)]

    def calculate_distance(self, point1, point2):
        """두 점 사이의 거리 계산

        유클리드 거리 공식: d = √[(x₂-x₁)² + (y₂-y₁)²]
        MediaPipe는 정규화된 좌표(0-1)를 사용하므로 실제 거리가 아닌 상대적 거리

        처음엔 3D 거리도 고려했는데, z축 값이 불안정해서 2D만 사용하기로 결정
        실험해보니 2D 거리만으로도 충분히 정확했음
        """
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    def calculate_palm_center(self, landmarks):
        """손바닥 중심점 계산

        수학적 원리: 중심점(centroid) = Σ(points) / n
        손바닥의 기하학적 중심을 구하기 위해 5개 주요 점의 평균 계산

        여러 방법 시도했던 흔적:
        1. 처음엔 손목만 사용 → 부정확
        2. 모든 21개 랜드마크 평균 → 손가락 끝이 포함되어 부적합
        3. 현재: 손목 + 4개 MCP 관절 → 가장 안정적인 손바닥 중심

        엄지 MCP(2번)를 제외한 이유: 엄지는 손바닥에서 벗어나 있어서 중심 계산 왜곡
        """
        # 손목과 각 손가락 MCP 관절들의 중심점
        wrist = landmarks[0]
        mcp_points = [landmarks[i] for i in [5, 9, 13, 17]]  # 검지, 중지, 약지, 새끼 MCP

        # 모든 점들의 평균으로 손바닥 중심 계산
        all_points = [wrist] + mcp_points
        center_x = sum(p.x for p in all_points) / len(all_points)
        center_y = sum(p.y for p in all_points) / len(all_points)
        center_z = sum(getattr(p, 'z', 0) for p in all_points) / len(all_points)

        # 손바닥 중심점 객체 생성 (MediaPipe 랜드마크와 같은 형식)
        class PalmCenter:
            def __init__(self, x, y, z):
                self.x = x
                self.y = y
                self.z = z

        return PalmCenter(center_x, center_y, center_z)

    def get_finger_curl_target(self, landmarks, finger_index):
        """각 손가락별 접힘 타겟 지점 계산"""
        if finger_index == 0:  # 엄지
            # 엄지는 검지 MCP(5번)와 새끼 MCP(17번) 사이의 손바닥 영역으로 접힘
            index_mcp = landmarks[5]  # 검지 MCP
            pinky_mcp = landmarks[17]  # 새끼 MCP

            # 두 점의 중점을 엄지 접힘 타겟으로 설정
            class ThumbTarget:
                def __init__(self, x, y, z):
                    self.x = x
                    self.y = y
                    self.z = z

            center_x = (index_mcp.x + pinky_mcp.x) / 2
            center_y = (index_mcp.y + pinky_mcp.y) / 2
            center_z = (getattr(index_mcp, 'z', 0) + getattr(pinky_mcp, 'z', 0)) / 2

            return ThumbTarget(center_x, center_y, center_z)
        else:
            # 나머지 손가락들은 손바닥 중심으로 접힘
            return self.calculate_palm_center(landmarks)

    def calculate_distance_based_curl(self, landmarks, finger_index):
        """거리 기반 손가락 접힘 정도 계산

        핵심 아이디어: 손가락이 접힐수록 끝점이 손바닥 중심에 가까워진다

        수학적 모델:
        curl_ratio = 1 - (현재거리 / 최대거리)
        - curl_ratio = 0: 완전히 펼쳐짐
        - curl_ratio = 1: 완전히 접힘

        경험적 계수 선정 과정:
        - 엄지 1.8배: 엄지는 다른 손가락보다 짧아서 계수 조정
        - 나머지 2.0배: 여러 사람 손으로 테스트해서 얻은 평균값
        - 처음엔 고정값 사용했다가 비율 방식으로 변경 → 손 크기 무관하게 작동!
        """
        tip = landmarks[self.finger_tips[finger_index]]
        mcp = landmarks[self.finger_mcp[finger_index]]
        curl_target = self.get_finger_curl_target(landmarks, finger_index)

        # 1. 손가락 끝에서 접힘 타겟까지의 거리
        tip_to_target = self.calculate_distance(tip, curl_target)

        # 2. MCP에서 접힘 타겟까지의 거리 (기준 거리)
        mcp_to_target = self.calculate_distance(mcp, curl_target)

        # 3. 자연 상태에서의 예상 거리 (MCP 거리의 1.5-2배)
        if finger_index == 0:  # 엄지
            expected_extended_distance = mcp_to_target * 1.8
        else:
            expected_extended_distance = mcp_to_target * 2.0

        # 4. 접힘 비율 계산 (0 = 완전히 펼쳐짐, 1 = 완전히 접힘)
        curl_ratio = 1 - (tip_to_target / (expected_extended_distance + 0.001))
        curl_ratio = max(0, min(1, curl_ratio))

        # 5. Z축 깊이 고려 (손가락이 손바닥 안쪽으로 들어감)
        if hasattr(tip, 'z') and hasattr(curl_target, 'z'):
            z_diff = getattr(curl_target, 'z', 0) - getattr(tip, 'z', 0)
            if z_diff > 0.01:  # 손가락이 타겟보다 안쪽에 있음
                curl_ratio += 0.2  # 접힘 보너스

        return min(1.0, curl_ratio)

    def calculate_thumb_palm_contact(self, landmarks):
        """엄지의 손바닥 접촉 감지 (향상된 버전)"""
        thumb_tip = landmarks[4]
        thumb_mcp = landmarks[2]
        index_mcp = landmarks[5]  # 검지 MCP (엄지 접힘 타겟)

        # 1. 엄지 끝과 검지 MCP 사이의 거리
        tip_to_index_mcp = self.calculate_distance(thumb_tip, index_mcp)

        # 2. 엄지 MCP와 검지 MCP 사이의 거리 (기준 거리)
        mcp_to_index_mcp = self.calculate_distance(thumb_mcp, index_mcp)

        # 3. 접촉 임계값 (MCP 거리의 60% 이하면 접촉)
        contact_threshold = mcp_to_index_mcp * 0.6

        # 4. 접촉 정도 계산 (0 = 멀리 떨어짐, 1 = 완전 접촉)
        if tip_to_index_mcp <= contact_threshold:
            contact_ratio = 1 - (tip_to_index_mcp / contact_threshold)
        else:
            contact_ratio = 0

        # 5. Z축 깊이로 추가 검증 (엄지가 손바닥 안쪽으로)
        if hasattr(thumb_tip, 'z') and hasattr(index_mcp, 'z'):
            z_diff = getattr(index_mcp, 'z', 0) - getattr(thumb_tip, 'z', 0)
            if z_diff > 0.015:  # 엄지가 더 안쪽에 있음
                contact_ratio += 0.3

        return min(1.0, contact_ratio)

    def detect_finger_overlap(self, hand_landmarks, finger_index):
        """손가락 겹침 감지 및 신뢰도 계산"""
        # MediaPipe에서 제공하는 visibility나 presence 정보 활용
        # (실제로는 hand_landmarks.landmark에서 각 점의 visibility 확인)
        landmarks = hand_landmarks.landmark

        if hasattr(landmarks[self.finger_tips[finger_index]], 'visibility'):
            visibility = landmarks[self.finger_tips[finger_index]].visibility
        else:
            visibility = 1.0  # 기본값

        # 신뢰도 히스토리에 추가
        self.finger_confidence_history[finger_index].append(visibility)

        # 평균 신뢰도 계산
        avg_confidence = sum(self.finger_confidence_history[finger_index]) / len(self.finger_confidence_history[finger_index])

        # 겹침 여부 판단 (신뢰도가 낮으면 겹침)
        is_overlapped = avg_confidence < 0.7

        return is_overlapped, avg_confidence

    def estimate_overlapped_finger(self, landmarks, finger_index):
        """겹친 손가락의 상태를 인접 손가락으로 추정"""
        # 인접 손가락들의 상태 확인
        adjacent_fingers = []

        if finger_index == 1:  # 검지
            adjacent_fingers = [2]  # 중지
        elif finger_index == 2:  # 중지
            adjacent_fingers = [1, 3]  # 검지, 약지
        elif finger_index == 3:  # 약지
            adjacent_fingers = [2, 4]  # 중지, 새끼
        elif finger_index == 4:  # 새끼
            adjacent_fingers = [3]  # 약지

        # 인접 손가락들의 평균 접힘 정도로 추정
        if adjacent_fingers:
            adjacent_curls = []
            for adj_finger in adjacent_fingers:
                curl_ratio = self.calculate_distance_based_curl(landmarks, adj_finger)
                adjacent_curls.append(curl_ratio)

            # 평균값에 약간의 보정 적용
            estimated_curl = sum(adjacent_curls) / len(adjacent_curls)
            # 겹친 손가락은 보통 더 접혀있을 가능성이 높음
            estimated_curl = min(1.0, estimated_curl * 1.1)

            return estimated_curl

        return 0.5  # 기본값

    def learn_fully_extended_state(self, landmarks):
        """완전히 펼쳐진 손 상태 학습"""
        if self.extended_state_learned:
            return

        # 완전 펼침 상태인지 확인
        is_fully_extended, finger_extensions = self.detect_fully_extended_state(landmarks)

        if is_fully_extended:
            # 각 손가락의 최대 펼침 거리 측정
            distances = []
            for i in range(5):
                curl_target = self.get_finger_curl_target(landmarks, i)
                tip = landmarks[self.finger_tips[i]]
                distance = self.calculate_distance(tip, curl_target)
                distances.append(distance)

            # 완전 펼침 상태 데이터에 추가
            self.extended_state_data.append(distances)

            # 충분한 데이터가 모이면 최대 거리 계산
            if len(self.extended_state_data) >= self.extended_state_frames:
                # 각 손가락별로 최대값들의 평균 계산
                for finger_idx in range(5):
                    finger_distances = [data[finger_idx] for data in self.extended_state_data]
                    # 최대값들의 평균 (완전히 펼쳐진 상태의 기준)
                    max_distance = sum(finger_distances) / len(finger_distances)
                    self.max_finger_distances[finger_idx] = max_distance

                self.extended_state_learned = True
                print("Fully extended state learned!")

    def calculate_pure_individual_control(self, landmarks, finger_index):
        """순수 개별 손가락 제어 (즉시 제어, 학습 과정 없음)

        선형 매핑 공식:
        motor_angle = (1 - curl_ratio) × 180

        각도 반전 이유:
        - 아두이노 서보: 180도 = 펼침, 0도 = 접힘
        - curl_ratio: 0 = 펼침, 1 = 접힘
        - 따라서 (1 - curl_ratio)로 반전

        처음엔 복잡한 학습 알고리즘 만들었는데...
        테스트해보니 즉시 제어가 더 반응이 좋아서 단순화했음
        때로는 단순한 게 최고! KISS 원칙(Keep It Simple, Stupid)
        """
        curl_target = self.get_finger_curl_target(landmarks, finger_index)
        tip = landmarks[self.finger_tips[finger_index]]
        current_distance = self.calculate_distance(tip, curl_target)

        max_distance = self.max_finger_distances[finger_index]

        # 기본 최대 거리 대비 현재 접힘 정도 계산
        if max_distance > 0:
            # 현재 거리 / 최대 거리 = 펼침 비율
            extension_ratio = current_distance / max_distance
            # 접힘 비율 = 1 - 펼림 비율
            curl_ratio = 1 - extension_ratio
            curl_ratio = max(0, min(1, curl_ratio))
        else:
            curl_ratio = 0.5

        # 모터 각도로 직접 변환 (아두이노: 180 = 완전 펼침, 0 = 완전 접힘)
        motor_angle = (1 - curl_ratio) * 180  # 각도 반전
        return max(0, min(180, motor_angle))

    def multi_stage_verification(self, hand_landmarks, finger_index):
        """다단계 검증 시스템"""
        landmarks = hand_landmarks.landmark

        # 1단계: 관절 각도 기반 판단
        if finger_index == 0:
            angle_score = self.get_adaptive_thumb_angle(self.calculate_thumb_3d_angle(landmarks)) / 180
        else:
            angle_score = self.calculate_multi_joint_angle(landmarks, finger_index) / 180

        # 2단계: 거리 기반 검증
        if finger_index == 0:
            distance_score = self.calculate_thumb_palm_contact(landmarks)
        else:
            distance_score = self.calculate_distance_based_curl(landmarks, finger_index)

        # 3단계: 겹침 보상
        is_overlapped, confidence = self.detect_finger_overlap(hand_landmarks, finger_index)
        if is_overlapped and confidence < 0.5:
            # 심각한 겹침 -> 추정값 사용
            overlap_score = self.estimate_overlapped_finger(landmarks, finger_index)
            overlap_weight = 0.8  # 추정값에 높은 가중치
        else:
            overlap_score = distance_score
            overlap_weight = confidence  # 신뢰도에 따른 가중치

        # 4단계: 자연 상태 대비 보정
        relative_score = self.calculate_relative_curl(landmarks, finger_index)

        # 최종 점수 계산 (가중 평균)
        if self.rest_state_learned:
            final_score = (
                angle_score * 0.3 +
                distance_score * 0.3 +
                overlap_score * overlap_weight * 0.2 +
                relative_score * 0.2
            )
        else:
            # 휴식 상태 미학습 시
            final_score = (
                angle_score * 0.4 +
                distance_score * 0.4 +
                overlap_score * overlap_weight * 0.2
            )

        # 180도 스케일로 변환 (모터 제어용)
        motor_angle = (1 - final_score) * 180
        return max(0, min(180, motor_angle))

    def calculate_angle_3points(self, a, b, c):
        """세 점으로 이루어진 각도 계산 (b가 꼭짓점)

        벡터 내적 공식을 이용한 각도 계산:
        cos(θ) = (BA · BC) / (|BA| × |BC|)

        여기서:
        - BA · BC = 벡터의 내적 (dot product)
        - |BA| = 벡터 BA의 크기 (magnitude)
        - θ = 두 벡터 사이의 각도

        삽질의 역사:
        1. 처음엔 atan2 사용 → 각도 범위 문제로 실패
        2. 외적(cross product)도 시도 → 2D에선 불필요하다는 걸 깨달음
        3. 결국 내적만으로 해결! 가장 간단한 방법이 최고였음

        cos_angle 범위 제한(-1, 1): 부동소수점 오차로 acos 에러 방지
        """
        # 벡터 BA, BC 계산
        ba = [a.x - b.x, a.y - b.y]
        bc = [c.x - b.x, c.y - b.y]

        # 내적과 외적으로 각도 계산
        dot_product = ba[0] * bc[0] + ba[1] * bc[1]
        magnitude_ba = math.sqrt(ba[0]**2 + ba[1]**2)
        magnitude_bc = math.sqrt(bc[0]**2 + bc[1]**2)

        if magnitude_ba == 0 or magnitude_bc == 0:
            return 180  # 벡터 길이가 0이면 일직선으로 간주

        cos_angle = dot_product / (magnitude_ba * magnitude_bc)
        cos_angle = max(-1, min(1, cos_angle))  # 범위 제한

        angle = math.acos(cos_angle) * 180 / math.pi
        return angle

    def calculate_3d_angle(self, a, b, c):
        """3D 공간에서 세 점으로 이루어진 각도 계산 (Z축 포함)

        3차원 벡터 내적 공식:
        BA · BC = ba_x×bc_x + ba_y×bc_y + ba_z×bc_z

        MediaPipe의 z값은 깊이 추정값이라 불안정함
        - 카메라에서 가까울수록 음수
        - 멀수록 양수
        - 단일 카메라라 정확도가 떨어짐

        그래서 getattr(a, 'z', 0) 사용: z값이 없으면 0으로 처리
        실제론 z값이 노이즈가 많아서 가중치를 낮춰 사용하는게 좋음
        """
        # 3D 벡터 BA, BC 계산
        ba = [a.x - b.x, a.y - b.y, getattr(a, 'z', 0) - getattr(b, 'z', 0)]
        bc = [c.x - b.x, c.y - b.y, getattr(c, 'z', 0) - getattr(b, 'z', 0)]

        # 3D 내적 계산
        dot_product = ba[0] * bc[0] + ba[1] * bc[1] + ba[2] * bc[2]
        magnitude_ba = math.sqrt(ba[0]**2 + ba[1]**2 + ba[2]**2)
        magnitude_bc = math.sqrt(bc[0]**2 + bc[1]**2 + bc[2]**2)

        if magnitude_ba == 0 or magnitude_bc == 0:
            return 180

        cos_angle = dot_product / (magnitude_ba * magnitude_bc)
        cos_angle = max(-1, min(1, cos_angle))

        angle = math.acos(cos_angle) * 180 / math.pi
        return angle

    def calculate_thumb_3d_angle(self, landmarks):
        """엄지 전용 3D 각도 계산

        엄지는 특별한 처리가 필요한 이유:
        1. 다른 손가락과 다른 관절 구조 (CMC-MCP-IP-TIP)
        2. 움직임 평면이 다름 (손바닥과 수직 방향)
        3. 접힘 방향이 손바닥 안쪽이 아닌 검지 방향

        여러 번의 시행착오:
        - v1: 단순 각도 계산 → 엄지 움직임 제대로 감지 못함
        - v2: 손바닥 평면과의 각도 → 개선되었지만 부족
        - v3: 손바닥 평면 거리 + Z축 변화 복합 계산 → 현재 버전
        """
        cmc = landmarks[1]   # CMC 관절 (엄지 특유)
        mcp = landmarks[2]   # MCP 관절
        ip = landmarks[3]    # IP 관절
        tip = landmarks[4]   # 끝점

        # 1. 기본 3D 각도 (MCP-IP-TIP)
        basic_angle = self.calculate_3d_angle(mcp, ip, tip)

        # 2. Z축 변화량으로 접힘 정도 계산
        z_tip = getattr(tip, 'z', 0)
        z_mcp = getattr(mcp, 'z', 0)
        z_diff = z_tip - z_mcp

        # 3. 손바닥 평면 계산 (손목, 검지MCP, 소지MCP로 평면 정의)
        wrist = landmarks[0]
        index_mcp = landmarks[5]
        pinky_mcp = landmarks[17]

        # 손바닥 평면의 법선 벡터 계산
        # 선형대수학: 두 벡터의 외적은 그 평면에 수직인 벡터를 생성
        # normal = v1 × v2 = (v1_y×v2_z - v1_z×v2_y, v1_z×v2_x - v1_x×v2_z, v1_x×v2_y - v1_y×v2_x)
        v1 = [index_mcp.x - wrist.x, index_mcp.y - wrist.y, getattr(index_mcp, 'z', 0) - getattr(wrist, 'z', 0)]
        v2 = [pinky_mcp.x - wrist.x, pinky_mcp.y - wrist.y, getattr(pinky_mcp, 'z', 0) - getattr(wrist, 'z', 0)]

        # 외적으로 법선 벡터 계산 (오른손 법칙 적용)
        normal = [
            v1[1] * v2[2] - v1[2] * v2[1],  # i 성분
            v1[2] * v2[0] - v1[0] * v2[2],  # j 성분
            v1[0] * v2[1] - v1[1] * v2[0]   # k 성분
        ]

        # 4. 엄지 끝점에서 손바닥 평면까지의 거리
        tip_vector = [tip.x - wrist.x, tip.y - wrist.y, getattr(tip, 'z', 0) - getattr(wrist, 'z', 0)]
        normal_magnitude = math.sqrt(normal[0]**2 + normal[1]**2 + normal[2]**2)

        if normal_magnitude > 0:
            # 정규화된 법선 벡터
            normal_unit = [n / normal_magnitude for n in normal]
            # 점-평면 거리 계산
            plane_distance = abs(sum(tip_vector[i] * normal_unit[i] for i in range(3)))
        else:
            plane_distance = 0

        # 5. 복합 각도 계산
        # Z축 변화와 평면 거리를 고려한 보정
        if z_diff > 0.02:  # 엄지가 앞으로 나온 경우
            basic_angle *= 0.8  # 각도 감소 (더 접힌 것으로 판단)
        elif z_diff < -0.02:  # 엄지가 뒤로 들어간 경우
            basic_angle *= 1.2  # 각도 증가 (더 펴진 것으로 판단)

        # 손바닥 평면 거리로 추가 보정
        if plane_distance < 0.05:  # 손바닥에 가까운 경우
            basic_angle = min(basic_angle, 120)  # 접힌 상태로 제한

        return basic_angle

    def calibrate_thumb_baseline(self, thumb_angle):
        """엄지 기준각 보정"""
        if not self.thumb_calibrated:
            self.thumb_baseline_angles.append(thumb_angle)

            if len(self.thumb_baseline_angles) >= self.thumb_calibration_frames:
                # 중간값들의 평균으로 기준각 설정 (극값 제거)
                sorted_angles = sorted(self.thumb_baseline_angles)
                mid_start = len(sorted_angles) // 4
                mid_end = 3 * len(sorted_angles) // 4
                self.thumb_baseline = sum(sorted_angles[mid_start:mid_end]) / (mid_end - mid_start)
                self.thumb_calibrated = True
                print(f"Thumb calibrated! Baseline angle: {self.thumb_baseline:.1f}")

    def get_adaptive_thumb_angle(self, raw_angle):
        """엄지 적응형 각도 계산"""
        # 보정 단계
        self.calibrate_thumb_baseline(raw_angle)

        if not self.thumb_calibrated:
            return raw_angle  # 보정 중에는 원본 각도 사용

        # 기준각 대비 상대 각도 계산
        relative_angle = raw_angle - self.thumb_baseline

        # 적응형 스케일링
        if relative_angle > 0:  # 펼쳐진 방향
            # 펼쳐진 각도는 더 민감하게 반응
            scaled_angle = self.thumb_baseline + (relative_angle * 1.3)
        else:  # 접힌 방향
            # 접힌 각도는 덜 민감하게 반응 (오인식 방지)
            scaled_angle = self.thumb_baseline + (relative_angle * 0.8)

        # 범위 제한
        scaled_angle = max(0, min(180, scaled_angle))

        return scaled_angle

    def calculate_multi_joint_angle(self, landmarks, finger_index):
        """다중 관절 각도 계산으로 정확한 손가락 접힘 감지

        손가락 관절 구조 (엄지 제외):
        - MCP (Metacarpophalangeal): 손허리뼈-손가락뼈 관절
        - PIP (Proximal Interphalangeal): 첫마디뼈-가운데마디뾈 관절
        - DIP (Distal Interphalangeal): 가운데마디뾈-끝마디뾈 관절

        왜 다중 관절을 봐야 하는가?
        - 단일 각도만 보면 부분적인 접힘을 놓침
        - 예: 주먹 쥘 때 모든 관절이 접히지만, 가리킬 때는 MCP만 펴짐
        - 가중평균으로 자연스러운 손가락 움직임 모델링

        실험을 통해 얻은 가중치:
        - 관절 각도 60%: 실제 접힘 상태를 잘 반영
        - 전체 각도 40%: 스무딩 효과와 안정성
        """
        if finger_index == 0:  # 엄지는 별도 처리
            return self.get_adaptive_thumb_angle(self.calculate_thumb_3d_angle(landmarks))

        # DIP 관절 인덱스 (MediaPipe 기준)
        finger_dip = [0, 7, 11, 15, 19]  # 엄지는 DIP가 없음

        mcp = landmarks[self.finger_mcp[finger_index]]
        pip = landmarks[self.finger_pip[finger_index]]
        dip = landmarks[finger_dip[finger_index]]
        tip = landmarks[self.finger_tips[finger_index]]

        # 1. MCP-PIP 각도 (첫 번째 관절)
        mcp_pip_angle = self.calculate_angle_3points(mcp, pip, dip)

        # 2. PIP-DIP 각도 (두 번째 관절)
        pip_dip_angle = self.calculate_angle_3points(pip, dip, tip)

        # 3. 전체 손가락 각도 (MCP-PIP-TIP)
        overall_angle = self.calculate_angle_3points(mcp, pip, tip)

        # 4. 3D 깊이 기반 보정
        z_tip = getattr(tip, 'z', 0)
        z_mcp = getattr(mcp, 'z', 0)
        z_curl = z_tip - z_mcp

        # 5. 복합 각도 계산
        # 두 관절 각도의 가중 평균 + 전체 각도
        joint_weight = 0.6  # 관절 각도의 가중치
        overall_weight = 0.4  # 전체 각도의 가중치

        # 관절들이 모두 구부러져야 완전히 접힌 것으로 판단
        joint_avg = (mcp_pip_angle + pip_dip_angle) / 2
        combined_angle = (joint_avg * joint_weight) + (overall_angle * overall_weight)

        # 6. 깊이 보정 적용
        if z_curl > 0.03:  # 손가락이 앞으로 나와있음 (덜 접힌 상태)
            combined_angle *= 1.1
        elif z_curl < -0.02:  # 손가락이 말려들어감 (더 접힌 상태)
            combined_angle *= 0.85

        # 7. 손가락별 특성 반영
        finger_char = self.finger_characteristics[finger_index]
        combined_angle *= finger_char["sensitivity"]

        return max(0, min(180, combined_angle))

    def detect_3d_curl(self, landmarks, finger_index):
        """3D 공간에서 손가락의 말려들어감 감지"""
        if finger_index == 0:  # 엄지는 별도 처리
            return 0

        tip = landmarks[self.finger_tips[finger_index]]
        mcp = landmarks[self.finger_mcp[finger_index]]
        wrist = landmarks[0]

        # 손바닥 평면에서 손가락 끝까지의 거리 계산
        palm_to_tip_2d = math.sqrt((tip.x - wrist.x)**2 + (tip.y - wrist.y)**2)
        palm_to_mcp_2d = math.sqrt((mcp.x - wrist.x)**2 + (mcp.y - wrist.y)**2)

        # 정상적으로 펼쳐진 상태라면 tip이 mcp보다 멀어야 함
        expected_extension = palm_to_mcp_2d * 1.5  # 예상 펼쳐진 거리

        # 실제 거리와 예상 거리의 비율
        curl_ratio = palm_to_tip_2d / (expected_extension + 0.001)

        # Z축 변화도 고려
        z_curl = getattr(tip, 'z', 0) - getattr(mcp, 'z', 0)

        # 말려들어감 점수 계산 (0~1, 1이 완전히 말려들어감)
        curl_score = max(0, 1 - curl_ratio)
        if z_curl < -0.02:  # Z축으로도 말려들어감
            curl_score *= 1.2

        return min(1.0, curl_score)

    def detect_fully_extended_state(self, landmarks):
        """완전히 펼쳐진 손 상태 감지"""
        # 모든 손가락이 최대한 펼쳐진 상태인지 확인
        finger_extensions = []

        for i in range(5):
            curl_target = self.get_finger_curl_target(landmarks, i)
            tip = landmarks[self.finger_tips[i]]
            mcp = landmarks[self.finger_mcp[i]]

            # 손가락 끝과 타겟 사이의 거리
            tip_to_target = self.calculate_distance(tip, curl_target)
            # MCP와 타겟 사이의 거리 (기준)
            mcp_to_target = self.calculate_distance(mcp, curl_target)

            # 펼침 비율 (tip이 mcp보다 멀리 있을수록 펼쳐진 상태)
            if mcp_to_target > 0:
                extension_ratio = tip_to_target / mcp_to_target
            else:
                extension_ratio = 1.0

            finger_extensions.append(extension_ratio)

        # 모든 손가락이 충분히 펼쳐져 있는지 확인
        # 엄지: 1.5배 이상, 나머지: 1.8배 이상
        thresholds = [1.5, 1.8, 1.8, 1.8, 1.8]
        is_fully_extended = all(ext >= thresh for ext, thresh in zip(finger_extensions, thresholds))

        return is_fully_extended, finger_extensions

    def apply_temporal_consistency(self, new_angles):
        """시간적 일관성을 위한 각도 보정"""
        if not hasattr(self, 'previous_angles'):
            self.previous_angles = new_angles[:]
            return new_angles

        consistent_angles = []
        for i, (new_angle, prev_angle) in enumerate(zip(new_angles, self.previous_angles)):
            angle_change = abs(new_angle - prev_angle)

            # 급격한 변화 감지 (손가락별 임계값 적용)
            finger_char = self.finger_characteristics[i]
            max_change = 15 * finger_char["sensitivity"]  # 손가락별 최대 변화량

            if angle_change > max_change:
                # 급격한 변화일 때 이전 값에 가중치 적용
                smoothed_angle = prev_angle * 0.7 + new_angle * 0.3
            else:
                smoothed_angle = new_angle

            consistent_angles.append(smoothed_angle)

        self.previous_angles = consistent_angles[:]
        return consistent_angles

    def calculate_finger_angles(self, hand_landmarks):
        """
        순수 개별 손가락 제어 시스템 (즉시 제어)
        컨텍스트 무관한 완전 독립 제어, 학습 과정 없음

        Returns:
            list: 각 손가락의 구부림 각도 (0-180)
        """
        angles = []
        landmarks = hand_landmarks.landmark

        for i in range(5):
            # 순수 개별 제어 시스템 사용 (즉시 제어)
            angle = self.calculate_pure_individual_control(landmarks, i)
            angles.append(angle)

        # 시간적 일관성 보정만 적용 (컨텍스트 보정 제거됨)
        angles = self.apply_temporal_consistency(angles)

        return angles

    def smooth_angles(self, new_angles):
        """각도 스무딩 처리로 떨림 방지

        이동평균 필터(Moving Average Filter) 적용:
        smoothed = Σ(angles) / n

        deque(maxlen=5) 사용 이유:
        - 최근 5프레임만 유지 → 메모리 효율적
        - 자동으로 오래된 값 제거
        - 5프레임: 30fps에서 약 0.16초 (적당한 반응속도)

        min_angle_change = 5도:
        - 5도 미만 변화는 노이즈로 간주
        - 서보모터 분해능(1도)보다 크게 설정
        - 시각적으로 자연스러운 임계값

        처음엔 칼만 필터도 시도했는데 오버엔지니어링이었음...
        단순 이동평균이 제일 효과적!
        """
        smoothed_angles = []

        for i, angle in enumerate(new_angles):
            # 히스토리에 추가
            self.angle_history[i].append(angle)

            # 이동평균 계산
            if len(self.angle_history[i]) > 1:
                smoothed = sum(self.angle_history[i]) / len(self.angle_history[i])
            else:
                smoothed = angle

            # 급격한 변화 제한 (떨림 방지)
            diff = abs(smoothed - self.last_angles[i])
            if diff < self.min_angle_change:
                smoothed = self.last_angles[i]  # 변화량이 작으면 이전값 유지

            smoothed_angles.append(int(smoothed))
            self.last_angles[i] = smoothed

        return smoothed_angles

    def get_finger_curl_percentage(self, landmarks, finger_index):
        """개별 손가락 접힘 비율 계산 (0-100%, 즉시 계산)"""
        curl_target = self.get_finger_curl_target(landmarks, finger_index)
        tip = landmarks[self.finger_tips[finger_index]]
        current_distance = self.calculate_distance(tip, curl_target)

        max_distance = self.max_finger_distances[finger_index]

        if max_distance > 0:
            extension_ratio = current_distance / max_distance
            curl_percentage = (1 - extension_ratio) * 100
            return max(0, min(100, curl_percentage))
        else:
            return 50

    def detect_gesture(self, hand_landmarks, finger_angles):
        """단순 제스처 인식 (개별 손가락 상태 표시용)"""
        landmarks = hand_landmarks.landmark

        # 개별 손가락 접힘 비율 계산
        curl_percentages = []
        for i in range(5):
            percentage = self.get_finger_curl_percentage(landmarks, i)
            curl_percentages.append(percentage)

        # 간단한 제스처 분류 (참고용)
        extended_count = sum(1 for p in curl_percentages if p < 30)

        if extended_count == 0:
            gesture = "Closed"
        elif extended_count == 5:
            gesture = "Open"
        elif extended_count == 1 and curl_percentages[1] < 30:  # 검지만 펴짐
            gesture = "Pointing"
        elif extended_count == 2 and curl_percentages[1] < 30 and curl_percentages[2] < 30:  # 검지+중지
            gesture = "Victory"
        else:
            gesture = f"{extended_count}_fingers"

        return gesture


    def send_to_arduino(self, finger_angles):
        """
        최적화된 Arduino 데이터 전송
        전송 간격 제어로 모터 과부하 방지

        Args:
            finger_angles: 5개 손가락 각도 리스트

        시리얼 통신 프로토콜:
        - 형식: "T,I,M,R,P\n" (CSV 형식)
        - 범위: 0-180 (서보 각도)
        - 속도: 9600 baud (Arduino 기본값)
        - 주기: 100ms (10Hz) - 서보모터 반응속도 고려

        처음에 JSON 형식도 고려했지만 CSV가 더 간단하고 빠름
        Arduino 쪽에서 parseInt()로 쉽게 파싱 가능
        """
        current_time = time.time()

        # 전송 간격 제어 (0.1초 = 100ms)
        # 너무 빠른 전송은 서보모터가 따라가지 못함
        if current_time - self.last_send_time < self.send_interval:
            return

        if self.arduino and self.arduino.is_open:
            # 데이터 포맷: "T,I,M,R,P\n"
            # T=엄지, I=검지, M=중지, R=약지, P=소지
            data = f"{finger_angles[0]},{finger_angles[1]},{finger_angles[2]},"
            data += f"{finger_angles[3]},{finger_angles[4]}\n"

            self.arduino.write(data.encode())  # UTF-8 인코딩으로 바이트 전송
            print(f"Sent: {data.strip()}")
            self.last_send_time = current_time

    def run(self):
        """
        메인 실행 루프
        """
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        print("Hand tracking started. Press 'q' to quit.")
        print("Immediate control mode - No learning required!")
        print("Move your fingers and the robot will respond immediately.")

        while cap.isOpened():
            success, image = cap.read()
            if not success:
                print("Failed to read from camera")
                continue

            # 이미지 전처리
            # 거울 효과로 자연스러운 사용자 경험
            # 사용자가 손을 오른쪽으로 움직이면 화면에서도 오른쪽으로
            image = cv2.flip(image, 1)  # 좌우 반전 (거울 효과)

            # BGR → RGB 변환 (OpenCV는 BGR, MediaPipe는 RGB 사용)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # 성능 최적화: 읽기 전용 플래그로 메모리 복사 방지
            image_rgb.flags.writeable = False

            # 손 인식 - MediaPipe의 핵심 처리 부분
            # 내부적으로 CNN 기반 모델이 손의 21개 포인트를 추정
            results = self.hands.process(image_rgb)

            # 이미지를 다시 BGR로 변환
            image_rgb.flags.writeable = True
            image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

            # 손이 감지되면
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # 손 랜드마크 그리기
                    self.mp_drawing.draw_landmarks(
                        image,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                        self.mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2)
                    )

                    # 손가락 각도 계산
                    raw_angles = self.calculate_finger_angles(hand_landmarks)

                    # 각도 스무딩 처리
                    finger_angles = self.smooth_angles(raw_angles)

                    # 제스처 인식
                    gesture = self.detect_gesture(hand_landmarks, finger_angles)

                    # Arduino로 전송
                    self.send_to_arduino(finger_angles)

                    # 개별 손가락 접힘 비율 표시
                    curl_percentages = []
                    for i in range(5):
                        percentage = self.get_finger_curl_percentage(hand_landmarks.landmark, i)
                        curl_percentages.append(percentage)

                    # 첫 번째 줄: 손가락 이름
                    names_text = "Thumb  Index  Middle  Ring  Pinky"
                    cv2.putText(image, names_text, (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    # 두 번째 줄: 접힘 비율 (%)
                    percentages_text = f"{curl_percentages[0]:3.0f}%   {curl_percentages[1]:3.0f}%    {curl_percentages[2]:3.0f}%    {curl_percentages[3]:3.0f}%   {curl_percentages[4]:3.0f}%"
                    cv2.putText(image, percentages_text, (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # 제스처 정보 표시 (참고용)
                    gesture_text = f"Gesture: {gesture}"
                    cv2.putText(image, gesture_text, (10, 90),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 255, 100), 1)

                    # 엄지 보정 상태 표시
                    if not self.thumb_calibrated:
                        calib_progress = len(self.thumb_baseline_angles)
                        calib_text = f"Thumb calibrating... {calib_progress}/{self.thumb_calibration_frames}"
                        cv2.putText(image, calib_text, (10, 120),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    else:
                        cv2.putText(image, "Thumb calibrated!", (10, 120),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                    # 즉시 제어 모드 표시
                    cv2.putText(image, "Immediate Control Mode - Ready!", (10, 150),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            else:
                cv2.putText(image, "No hand detected", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 프레임 표시
            cv2.imshow('Hand Tracking Robot Controller', image)

            # 'q' 키로 종료
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

        # 종료 처리
        cap.release()
        cv2.destroyAllWindows()
        if self.arduino:
            self.arduino.close()
        self.hands.close()
        print("Program terminated.")

def main():
    """
    프로그램 진입점
    """
    print("=== Hand Tracking Robot Controller ===")
    print("Make sure your Arduino is connected and the port is correct.")
    print("Default port: /dev/cu.usbmodem1201")
    print()

    # 포트 설정 (필요시 변경)
    # Windows: 'COM3', 'COM4' 등
    # Mac: '/dev/cu.usbmodem*' 또는 '/dev/tty.usbserial-*'
    # Linux: '/dev/ttyUSB0' 또는 '/dev/ttyACM0' 등
    port = '/dev/cu.usbmodem1301'  # Arduino Uno/Mega 기본 포트

    try:
        tracker = HandTracker(port=port)
        tracker.run()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()