"""
대시보드 헬퍼 함수 #266
"""
import pandas as pd
import numpy as np

def process_data_266(df, column):
    """데이터 처리 함수 #266"""
    if column not in df.columns:
        return df
    # 기본 통계
    stats = {
        "평균": df[column].mean(),
        "중앙값": df[column].median(),
        "표준편차": df[column].std(),
        "최소": df[column].min(),
        "최대": df[column].max(),
    }
    return stats

def create_chart_data_266(categories, values):
    """차트 데이터 생성"""
    return pd.DataFrame({
        "카테고리": categories,
        "값": values,
        "비율": [v / sum(values) * 100 for v in values],
    })

if __name__ == "__main__":
    # 테스트 데이터
    np.random.seed(266)
    df = pd.DataFrame({
        "매출": np.random.randint(1000, 10000, 20),
        "직원수": np.random.randint(5, 200, 20),
    })
    stats = process_data_266(df, "매출")
    print(f"헬퍼 #266 테스트:")
    for k, v in stats.items():
        print(f"  {k}: {v:.2f}")
