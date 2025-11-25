"""
데이터 분석 #459
"""
import numpy as np

def analyze_dataset_459(data):
    """데이터셋 분석"""
    stats = {
        "샘플 수": len(data),
        "평균": np.mean(data, axis=0),
        "표준편차": np.std(data, axis=0),
        "최소": np.min(data, axis=0),
        "최대": np.max(data, axis=0),
        "중앙값": np.median(data, axis=0),
    }
    return stats

def detect_outliers_459(data, threshold=2.5):
    """이상치 탐지"""
    mean = np.mean(data, axis=0)
    std = np.std(data, axis=0)
    z_scores = np.abs((data - mean) / (std + 1e-10))
    outlier_mask = np.any(z_scores > threshold, axis=1)
    return np.where(outlier_mask)[0]

if __name__ == "__main__":
    np.random.seed(459)
    data = np.random.randn(4640, 2)
    # 이상치 추가
    data[9] = [12, 12]

    stats = analyze_dataset_459(data)
    print(f"분석 #459")
    for key, val in stats.items():
        if isinstance(val, np.ndarray):
            print(f"  {key}: {np.round(val, 3)}")
        else:
            print(f"  {key}: {val}")

    outliers = detect_outliers_459(data)
    print(f"  이상치 인덱스: {outliers}")
