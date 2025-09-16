# RoArm-M2-S 손 제스처 제어 시스템

웹캠을 통한 손 제스처 인식으로 Waveshare RoArm-M2-S 로봇팔을 실시간 제어하는 시스템입니다.

## 🎯 주요 기능

### 제스처 명령
- **엄지+검지 붙이기**: 그리퍼 열기/닫기
- **손바닥 앞쪽**: 로봇팔 앞으로 이동 (X+)
- **손바닥 뒤쪽**: 로봇팔 뒤로 이동 (X-)
- **손바닥 왼쪽**: 로봇팔 왼쪽으로 이동 (Y-)
- **손바닥 오른쪽**: 로봇팔 오른쪽으로 이동 (Y+)
- **손바닥 펼치기**: 홈 포지션으로 이동
- **주먹**: 비상 정지

### 시스템 특징
- 실시간 손 제스처 인식 (MediaPipe 기반)
- 안정화 필터링으로 오인식 방지
- 작업 공간 안전 범위 검증
- 시각적 피드백 및 상태 모니터링
- 키보드 단축키 지원
- 제스처 매핑 커스터마이징

## 🛠️ 설치 및 설정

### 1. 필요 조건
- Python 3.8 이상
- 웹캠
- RoArm-M2-S 로봇팔
- Wi-Fi 연결 (로봇팔과 통신용)

### 2. 의존성 설치
```bash
pip install -r requirements.txt
```

### 3. RoArm-M2-S 설정
1. RoArm-M2-S 전원 연결
2. Wi-Fi AP 모드로 설정 (기본값: 192.168.4.1)
3. 컴퓨터를 로봇팔 Wi-Fi에 연결

## 🚀 사용법

### 기본 실행
```bash
python main_gesture_control.py
```

### 옵션 설정
```bash
# 로봇 IP 주소 지정
python main_gesture_control.py --robot-ip 192.168.1.100

# 카메라 인덱스 변경
python main_gesture_control.py --camera 1

# 해상도 설정
python main_gesture_control.py --width 1280 --height 720

# 디버그 모드
python main_gesture_control.py --debug
```

### 키보드 단축키
- **SPACE**: 일시정지/재개
- **H**: 홈 포지션 이동
- **S**: 안전 위치 이동
- **E**: 비상 정지
- **C**: 그리퍼 토글
- **L**: 랜드마크 표시 토글
- **D**: 디버그 정보 토글
- **Q/ESC**: 프로그램 종료

## 📁 프로젝트 구조

```
roarm/
├── main_gesture_control.py    # 메인 애플리케이션
├── gesture_detector.py        # 고급 제스처 인식 모듈
├── roarm_controller.py        # RoArm-M2-S 제어 모듈
├── gesture_mapping.py         # 제스처-동작 매핑 시스템
├── hand_gesture_control.py    # 기본 제스처 제어 (단순 버전)
├── requirements.txt           # 필요 라이브러리
└── README.md                  # 이 파일
```

## 🔧 모듈별 기능

### 1. gesture_detector.py
- MediaPipe 기반 손 랜드마크 인식
- 정밀한 핀치 제스처 감지
- 손바닥 방향 인식
- 제스처 안정성 필터링
- 실시간 시각적 피드백

### 2. roarm_controller.py
- HTTP/JSON API를 통한 로봇팔 통신
- 좌표 기반 위치 제어
- 관절 각도 제어
- 작업 공간 안전 범위 검증
- 궤적 계획 및 실행

### 3. gesture_mapping.py
- 제스처와 로봇 동작 매핑
- 동작 큐 관리
- 쿨다운 시간 제어
- 사용자 설정 저장/로드
- 실행 통계 추적

### 4. main_gesture_control.py
- 전체 시스템 통합
- 실시간 GUI 인터페이스
- 키보드 입력 처리
- FPS 및 성능 모니터링
- 시스템 상태 표시

## ⚙️ 설정 커스터마이징

### 제스처 매핑 수정
```python
from gesture_mapping import GestureActionMapper, GestureAction, ActionType
from gesture_detector import GestureType

# 새로운 매핑 추가
custom_action = GestureAction(
    gesture=GestureType.PINCH,
    action_type=ActionType.GRIPPER,
    parameters={"force": 70},  # 그리퍼 힘 조절
    cooldown=1.0
)

mapper.add_gesture_mapping(custom_action)
```

### 이동 거리 조절
```python
# roarm_controller.py에서
controller.move_step = 10  # 기본 15mm에서 10mm로 변경
```

### 제스처 감도 조절
```python
# gesture_detector.py에서
detector.pinch_threshold = 0.03  # 핀치 감도 (기본 0.04)
detector.gesture_filter.confidence_threshold = 0.7  # 신뢰도 임계값
```

## 🔍 테스트 및 디버깅

### 개별 모듈 테스트
```bash
# 제스처 인식만 테스트
python gesture_detector.py

# 로봇 제어만 테스트
python roarm_controller.py

# 매핑 시스템 테스트
python gesture_mapping.py
```

### 시뮬레이션 모드
로봇팔이 연결되지 않은 상태에서도 제스처 인식을 테스트할 수 있습니다. 프로그램이 자동으로 시뮬레이션 모드로 전환됩니다.

## 🚨 안전 주의사항

1. **작업 공간 확인**: 로봇팔 주변에 장애물이 없는지 확인
2. **비상 정지**: 주먹 제스처나 'E' 키로 언제든 정지 가능
3. **초기 위치**: 프로그램 시작 시 자동으로 홈 포지션으로 이동
4. **종료 절차**: 프로그램 종료 시 자동으로 안전 위치로 이동

## 📊 성능 최적화

### 카메라 설정
- 해상도: 640x480 (권장) - 실시간 처리 최적화
- FPS: 30fps (권장)
- 조명: 균등한 조명 환경

### 시스템 요구사항
- CPU: Intel i5 이상 권장
- RAM: 8GB 이상
- GPU: 선택사항 (MediaPipe CPU 최적화)

## 🛠️ 문제해결

### 자주 발생하는 문제

1. **카메라 인식 실패**
   ```bash
   # 다른 카메라 인덱스 시도
   python main_gesture_control.py --camera 1
   ```

2. **로봇팔 연결 실패**
   - Wi-Fi 연결 상태 확인
   - IP 주소 확인 (기본값: 192.168.4.1)
   - 로봇팔 전원 상태 확인

3. **제스처 인식 불안정**
   - 조명 환경 개선
   - 카메라와 손 사이 거리 조절 (30-50cm 권장)
   - 배경 단순화

4. **성능 저하**
   - 해상도 낮추기 (320x240)
   - FPS 제한 (15fps)
   - 불필요한 프로그램 종료

## 📝 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 🤝 기여

버그 리포트, 기능 제안, 코드 기여를 환영합니다!

## 📞 지원

문제가 발생하면 GitHub Issues를 통해 문의해 주세요.

---

**즐거운 로봇팔 제어 되세요! 🤖👋**