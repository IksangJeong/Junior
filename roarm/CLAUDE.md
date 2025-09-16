# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a real-time hand gesture control system for the Waveshare RoArm-M2-S robotic arm using computer vision and MediaPipe. The system provides two control modes: discrete gesture commands and continuous hand position tracking.

## Development Commands

### Running the System
```bash
# Main application (integrated system)
python main_gesture_control.py

# Continuous hand control only
python continuous_hand_control.py

# Basic gesture control
python hand_gesture_control.py

# With options
python main_gesture_control.py --robot-ip 192.168.1.100 --camera 1 --debug
```

### Testing Individual Components
```bash
# Test gesture detection only
python gesture_detector.py

# Test robot controller only
python roarm_controller.py

# Test gesture mapping system
python gesture_mapping.py

# Test camera
python test_camera.py
```

### Dependencies
```bash
pip install -r requirements.txt
```

## Architecture Overview

### Core System Components

**Two Control Modes:**
1. **Discrete Gesture Control** (`gesture_detector.py` + `gesture_mapping.py`): Traditional gesture recognition with specific commands (pinch, fist, open hand, etc.)
2. **Continuous Position Control** (`continuous_hand_control.py`): Real-time hand position tracking where hand movements directly control robot joint angles

**Integration Layer:**
- `main_gesture_control.py`: Integrates both control modes with GUI, keyboard shortcuts, and system management
- `roarm_controller.py`: Hardware abstraction layer for RoArm-M2-S communication via HTTP/JSON API

### Control Flow Architecture

```
Camera Feed → MediaPipe → HandLandmarks → {
    ├─ Gesture Detection → Action Mapping → Robot Commands
    └─ Position Tracking → Joint Angles → Continuous Control
}
```

### Key Design Patterns

**Hand Position to Robot Mapping (continuous_hand_control.py):**
- X-axis (left/right) → Base rotation (10°-170°)
- Y-axis (up/down) → Elbow movement (-63° to 180°)
- Z-axis (depth) → Shoulder movement (-70° to 70°)
- Pinch gesture → Gripper control (3.14 rad closed, 1.08 rad open)

**Robot Communication:**
- HTTP GET requests to `http://192.168.4.1/js?json={JSON_DATA}`
- Command types: T:102 (joint control), T:104 (coordinate control), T:106 (gripper)
- Network setup: Connect to RoArm-M2 WiFi (password: 12345678)

**Safety Systems:**
- Workspace limits validation in `roarm_controller.py`
- Emergency stop via fist gesture or 'E' key
- Automatic home position on startup
- Safe position on shutdown

### State Management

**System State:**
- Control mode switching (continuous vs discrete)
- Robot connection status
- Calibration states
- Debug modes with real-time diagnostics

**Performance Optimizations:**
- Smoothing filters for continuous control
- Debouncing for gesture recognition
- FPS limiting and resolution optimization
- Parallel tool operations when possible

## Hardware Integration

### RoArm-M2-S Specifications
- 4DOF robotic arm with gripper
- WiFi AP mode (192.168.4.1) with web interface
- Joint ranges: Base (360°), Shoulder (-90° to 90°), Elbow (-63° to 180°), Gripper (135° range)
- Command format follows Information.md hardware specifications

### Network Requirements
- RoArm-M2-S must be powered and in WiFi AP mode
- Computer connected to "RoArm-M2" network
- Default IP: 192.168.4.1, fallback to user-specified IP

## Development Notes

### Debugging Features
- Real-time gripper distance monitoring in continuous mode
- MediaPipe landmark quality checking
- Network command success/failure tracking
- Visual feedback overlays for gesture recognition

### Common Issues
- Network timeouts: Check WiFi connection and robot power
- Gesture sensitivity: Adjust thresholds in FastGripperControl class
- Joint limits: Verify angles against Information.md specifications
- Camera access: Try different camera indices with --camera flag

### Extension Points
- Custom gesture mappings in `gesture_mapping.py`
- Control sensitivity adjustments in smoothing filters
- Additional control modes by extending ControlMode enum
- Custom robot actions through GestureActionMapper

The codebase supports both research/development use and production deployment with comprehensive error handling and diagnostic capabilities.