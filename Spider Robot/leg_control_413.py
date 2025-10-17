"""
스파이더 로봇 다리 제어 #413
"""
import math

# 다리 파라미터
L1 = 53  # mm (대퇴)
L2 = 38  # mm (경골)

def inverse_kinematics_413(x, z):
    """역기구학 계산 #413"""
    d = math.sqrt(x**2 + z**2)
    if d > L1 + L2:
        return None, None

    cos_a2 = (x**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_a2 = max(-1, min(1, cos_a2))
    a2 = math.acos(cos_a2)

    k1 = L1 + L2 * math.cos(a2)
    k2 = L2 * math.sin(a2)
    a1 = math.atan2(z, x) - math.atan2(k2, k1)

    return math.degrees(a1), math.degrees(a2)

def generate_step_413(step_length=43, step_height=28, points=8):
    """걸음 궤적 생성"""
    trajectory = []
    for i in range(points):
        t = i / points
        x = step_length * math.cos(2 * math.pi * t)
        z = step_height * max(0, math.sin(2 * math.pi * t))
        a1, a2 = inverse_kinematics_413(L1 + x, z)
        trajectory.append((round(x, 2), round(z, 2), a1, a2))
    return trajectory

if __name__ == "__main__":
    print(f"다리 제어 #413 (L1={L1}mm, L2={L2}mm)")
    traj = generate_step_413()
    for i, (x, z, a1, a2) in enumerate(traj):
        if a1 is not None:
            print(f"  Step {i}: ({x}, {z}) -> ({a1:.1f}°, {a2:.1f}°)")
