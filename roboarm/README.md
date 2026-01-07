# 5-DOF Robot Arm Pick System

색상 기반 물체 인식 및 자동 픽업 시스템

## 하드웨어

- **로봇팔**: 로보콘 5자유도 로봇팔 (MG996R 서보 x5 + 그리퍼)
- **컨트롤러**: Arduino
- **영상처리**: Raspberry Pi 4
- **카메라**: USB 웹캠 (Eye-in-Hand)

## 핀 배치

| 조인트 | 핀 | 동작 |
|--------|-----|------|
| J1 (Base) | 2 | 좌우 회전 |
| J2 | 4 | 앞뒤 기울임 |
| J3 | 6 | 앞뒤 기울임 |
| J4 | 8 | 앞뒤 기울임 |
| J5 (Wrist) | 10 | 손목 회전 |
| Gripper | 12 | 집게 |

## 파일 구조

```
roboarm/
├── arduino/
│   └── robot_arm.ino    # 아두이노 서보 제어
├── src/
│   └── main.py          # 라즈베리파이 메인 코드
├── requirements.txt
└── README.md
```

## 설치

### Arduino
1. `arduino/robot_arm.ino`를 Arduino IDE로 열기
2. 보드에 업로드

### Raspberry Pi / Linux
```bash
pip install -r requirements.txt
python src/main.py
```

### macOS
```bash
# 1. Python 패키지 설치
pip install -r requirements.txt

# 2. USB 드라이버 설치 (CH340/CH341 칩 사용시)
brew install --cask ch340g-ch34g-ch34x-mac-os-x-driver

# 3. 시리얼 포트 확인 (Arduino 연결 후)
ls /dev/cu.usb*

# 4. 실행
python src/main.py
```

**macOS 참고사항**:
- 시리얼 포트는 자동 감지됩니다 (`/dev/cu.usbserial-*` 또는 `/dev/cu.usbmodem*`)
- 카메라 권한: 시스템 환경설정 > 개인 정보 보호 및 보안 > 카메라에서 터미널 허용
- Arduino 없이 카메라만으로도 테스트 가능

### Windows
```bash
pip install -r requirements.txt
python src/main.py
```
- 시리얼 포트 자동 감지 (COM3, COM4 등)

## 사용법

### 키보드 조작
| 키 | 동작 |
|----|------|
| SPACE | 픽업 시작/중지 |
| H | 홈 위치 |
| G | 그리퍼 열기/닫기 |
| Q / ESC | 종료 |

### 시리얼 명령어 (Arduino)
```
J1:90,J2:45,J3:60,J4:90,J5:90,G:30   # 조인트 각도 설정
HOME                                   # 홈 위치로 이동
STOP                                   # 정지
STATUS                                 # 현재 위치 출력
```

## 상태 머신

```
IDLE → SEARCH → TRACK → APPROACH → ALIGN → GRAB → DONE
```

| 상태 | 설명 |
|------|------|
| IDLE | 대기 |
| SEARCH | 물체 탐색 |
| TRACK | 물체 추적 (10프레임 안정화) |
| APPROACH | 물체 방향으로 이동 |
| ALIGN | 미세 조정 |
| GRAB | 그리퍼 닫기 |
| DONE | 완료 |

## 설정 변경

`src/main.py` 상단에서 설정 가능:

```python
# 시리얼 포트 (None = 자동 감지)
SERIAL_PORT = None  # 또는 '/dev/ttyUSB0', '/dev/cu.usbserial-1420', 'COM3'

# 카메라
CAMERA_INDEX = 0

# 색상 감지 (HSV - 빨간색)
COLOR_LOWER_1 = np.array([0, 100, 100])
COLOR_UPPER_1 = np.array([10, 255, 255])

# 물체 크기 (깊이 추정용)
KNOWN_OBJECT_DIAMETER_MM = 40.0

# DH 파라미터 (로봇팔 링크 길이)
DH_PARAMS = [...]
```

## 화면 구성

실행 시 3개의 창이 표시됩니다:

| 창 | 설명 |
|----|------|
| **Camera** | 원본 카메라 영상 + 기본 오버레이 |
| **Robot View** | 로봇 시점 (향상된 UI: 그리드, 십자선, 깊이 바, 관절 각도) |
| **Mask** | 색상 감지 마스크 (이진 이미지) |

## 캘리브레이션

정확한 픽업을 위해 다음 값들을 실제 측정값으로 수정:

1. **DH 파라미터**: 로봇팔 링크 길이 (mm)
2. **FOCAL_LENGTH_PX**: 카메라 초점 거리
3. **KNOWN_OBJECT_DIAMETER_MM**: 목표 물체 직경
