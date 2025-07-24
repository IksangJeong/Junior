"""
헬퍼 함수들
"""

import pandas as pd
import numpy as np
import streamlit as st
from pathlib import Path
import os
import json
from datetime import datetime, timedelta

def load_data_with_cache(file_path, cache_key, create_sample_func=None):
    """캐싱된 데이터 로드"""
    @st.cache_data
    def _load_data():
        try:
            if Path(file_path).exists():
                return pd.read_csv(file_path)
            elif create_sample_func:
                return create_sample_func()
            else:
                return pd.DataFrame()
        except Exception as e:
            st.error(f"데이터 로드 실패: {str(e)}")
            if create_sample_func:
                return create_sample_func()
            return pd.DataFrame()
    
    return _load_data()

def safe_divide(numerator, denominator):
    """안전한 나눗셈"""
    if denominator == 0 or pd.isna(denominator):
        return 0
    return numerator / denominator

def calculate_growth_rate(current_value, previous_value):
    """성장률 계산"""
    if previous_value == 0 or pd.isna(previous_value):
        return 0
    return ((current_value - previous_value) / previous_value) * 100

def format_large_number(number):
    """큰 숫자 포맷팅"""
    if pd.isna(number):
        return "N/A"
    
    if number >= 1_000_000_000:
        return f"{number/1_000_000_000:.1f}B"
    elif number >= 1_000_000:
        return f"{number/1_000_000:.1f}M"
    elif number >= 1_000:
        return f"{number/1_000:.1f}K"
    else:
        return f"{number:.0f}"

def create_download_link(df, filename, link_text="다운로드"):
    """데이터프레임 다운로드 링크 생성"""
    csv = df.to_csv(index=False, encoding='utf-8-sig')
    return st.download_button(
        label=f"📥 {link_text}",
        data=csv,
        file_name=filename,
        mime="text/csv"
    )

def filter_dataframe(df, filters):
    """데이터프레임 필터링"""
    filtered_df = df.copy()
    
    for column, value in filters.items():
        if value != "전체" and value is not None:
            if isinstance(value, list):
                filtered_df = filtered_df[filtered_df[column].isin(value)]
            else:
                filtered_df = filtered_df[filtered_df[column] == value]
    
    return filtered_df

def get_top_n_data(df, value_col, n=10, ascending=False):
    """상위 N개 데이터 추출"""
    return df.nlargest(n, value_col) if not ascending else df.nsmallest(n, value_col)

def calculate_percentiles(df, column):
    """백분위수 계산"""
    return {
        'min': df[column].min(),
        'q25': df[column].quantile(0.25),
        'median': df[column].median(),
        'q75': df[column].quantile(0.75),
        'max': df[column].max(),
        'mean': df[column].mean(),
        'std': df[column].std()
    }

def create_age_groups(ages, bins=None, labels=None):
    """연령 그룹 생성"""
    if bins is None:
        bins = [0, 18, 35, 50, 65, 100]
    if labels is None:
        labels = ['18세 미만', '18-34세', '35-49세', '50-64세', '65세 이상']
    
    return pd.cut(ages, bins=bins, labels=labels, right=False)

def normalize_column(df, column):
    """컬럼 정규화"""
    min_val = df[column].min()
    max_val = df[column].max()
    
    if max_val == min_val:
        return pd.Series([0.5] * len(df))
    
    return (df[column] - min_val) / (max_val - min_val)

def standardize_column(df, column):
    """컬럼 표준화"""
    mean_val = df[column].mean()
    std_val = df[column].std()
    
    if std_val == 0:
        return pd.Series([0] * len(df))
    
    return (df[column] - mean_val) / std_val

def detect_outliers(df, column, method='iqr'):
    """이상치 탐지"""
    if method == 'iqr':
        Q1 = df[column].quantile(0.25)
        Q3 = df[column].quantile(0.75)
        IQR = Q3 - Q1
        
        lower_bound = Q1 - 1.5 * IQR
        upper_bound = Q3 + 1.5 * IQR
        
        return df[(df[column] < lower_bound) | (df[column] > upper_bound)]
    
    elif method == 'zscore':
        z_scores = np.abs(standardize_column(df, column))
        return df[z_scores > 3]
    
    return pd.DataFrame()

def create_time_series_features(df, date_column):
    """시계열 특성 생성"""
    df = df.copy()
    df[date_column] = pd.to_datetime(df[date_column])
    
    df['year'] = df[date_column].dt.year
    df['month'] = df[date_column].dt.month
    df['day'] = df[date_column].dt.day
    df['weekday'] = df[date_column].dt.weekday
    df['quarter'] = df[date_column].dt.quarter
    df['is_weekend'] = df['weekday'].isin([5, 6])
    
    return df

def calculate_moving_average(df, column, window=7):
    """이동평균 계산"""
    return df[column].rolling(window=window).mean()

def calculate_correlation_matrix(df, numeric_columns):
    """상관관계 매트릭스 계산"""
    return df[numeric_columns].corr()

def get_memory_usage(df):
    """메모리 사용량 확인"""
    return df.memory_usage(deep=True).sum() / 1024 / 1024  # MB

def optimize_dataframe(df):
    """데이터프레임 최적화"""
    optimized_df = df.copy()
    
    # 정수형 최적화
    for column in optimized_df.select_dtypes(include=['int']).columns:
        col_min = optimized_df[column].min()
        col_max = optimized_df[column].max()
        
        if col_min >= -128 and col_max <= 127:
            optimized_df[column] = optimized_df[column].astype('int8')
        elif col_min >= -32768 and col_max <= 32767:
            optimized_df[column] = optimized_df[column].astype('int16')
        elif col_min >= -2147483648 and col_max <= 2147483647:
            optimized_df[column] = optimized_df[column].astype('int32')
    
    # 실수형 최적화
    for column in optimized_df.select_dtypes(include=['float']).columns:
        optimized_df[column] = pd.to_numeric(optimized_df[column], downcast='float')
    
    # 문자열 최적화
    for column in optimized_df.select_dtypes(include=['object']).columns:
        num_unique_values = len(optimized_df[column].unique())
        num_total_values = len(optimized_df[column])
        
        if num_unique_values / num_total_values < 0.5:
            optimized_df[column] = optimized_df[column].astype('category')
    
    return optimized_df

def create_summary_statistics(df, numeric_columns):
    """요약 통계 생성"""
    summary = df[numeric_columns].describe()
    
    # 추가 통계 정보
    summary.loc['missing'] = df[numeric_columns].isnull().sum()
    summary.loc['missing_pct'] = (df[numeric_columns].isnull().sum() / len(df)) * 100
    
    return summary

def validate_data_quality(df):
    """데이터 품질 검증"""
    quality_report = {
        'total_rows': len(df),
        'total_columns': len(df.columns),
        'duplicate_rows': df.duplicated().sum(),
        'missing_values': df.isnull().sum().sum(),
        'memory_usage_mb': get_memory_usage(df)
    }
    
    # 컬럼별 누락값 비율
    missing_pct = (df.isnull().sum() / len(df)) * 100
    quality_report['high_missing_columns'] = missing_pct[missing_pct > 50].index.tolist()
    
    return quality_report

def create_data_profile(df):
    """데이터 프로파일 생성"""
    profile = {
        'shape': df.shape,
        'columns': df.columns.tolist(),
        'dtypes': df.dtypes.to_dict(),
        'numeric_columns': df.select_dtypes(include=[np.number]).columns.tolist(),
        'categorical_columns': df.select_dtypes(include=['object', 'category']).columns.tolist(),
        'date_columns': df.select_dtypes(include=['datetime']).columns.tolist(),
        'quality_report': validate_data_quality(df)
    }
    
    return profile

def export_data_profile(df, filename):
    """데이터 프로파일 내보내기"""
    profile = create_data_profile(df)
    
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(profile, f, ensure_ascii=False, indent=2, default=str)
    
    return filename

def load_config(config_path):
    """설정 파일 로드"""
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        return {}
    except json.JSONDecodeError:
        st.error("설정 파일 형식이 올바르지 않습니다.")
        return {}

def save_config(config, config_path):
    """설정 파일 저장"""
    try:
        with open(config_path, 'w', encoding='utf-8') as f:
            json.dump(config, f, ensure_ascii=False, indent=2)
        return True
    except Exception as e:
        st.error(f"설정 저장 실패: {str(e)}")
        return False

def create_backup(file_path):
    """파일 백업 생성"""
    if Path(file_path).exists():
        backup_path = f"{file_path}.backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        try:
            import shutil
            shutil.copy2(file_path, backup_path)
            return backup_path
        except Exception as e:
            st.error(f"백업 생성 실패: {str(e)}")
            return None
    return None

def clean_text_data(text):
    """텍스트 데이터 정제"""
    if pd.isna(text):
        return ""
    
    # 기본 정제
    text = str(text).strip()
    
    # 연속된 공백 제거
    text = " ".join(text.split())
    
    # 특수문자 정제 (필요에 따라 수정)
    text = text.replace('\n', ' ').replace('\t', ' ').replace('\r', ' ')
    
    return text

def calculate_business_days(start_date, end_date):
    """영업일 계산"""
    start_date = pd.to_datetime(start_date)
    end_date = pd.to_datetime(end_date)
    
    return pd.bdate_range(start_date, end_date).shape[0]

def get_season(date):
    """계절 구분"""
    month = pd.to_datetime(date).month
    
    if month in [12, 1, 2]:
        return '겨울'
    elif month in [3, 4, 5]:
        return '봄'
    elif month in [6, 7, 8]:
        return '여름'
    else:
        return '가을'

def create_cohort_analysis(df, customer_col, date_col, value_col=None):
    """코호트 분석 데이터 생성"""
    df = df.copy()
    df[date_col] = pd.to_datetime(df[date_col])
    
    # 첫 구매 날짜
    df['first_purchase'] = df.groupby(customer_col)[date_col].transform('min')
    
    # 기간 계산
    df['period'] = (df[date_col] - df['first_purchase']).dt.days
    
    # 코호트 그룹 생성
    df['cohort_group'] = df['first_purchase'].dt.to_period('M')
    
    # 기간 번호
    df['period_number'] = df['period'] // 30  # 월 단위
    
    # 코호트 테이블 생성
    cohort_data = df.groupby(['cohort_group', 'period_number'])[customer_col].nunique().reset_index()
    cohort_table = cohort_data.pivot(index='cohort_group', 
                                    columns='period_number', 
                                    values=customer_col)
    
    # 비율 계산
    cohort_sizes = df.groupby('cohort_group')[customer_col].nunique()
    cohort_percentages = cohort_table.divide(cohort_sizes, axis=0)
    
    return cohort_table, cohort_percentages

def check_data_freshness(file_path, max_age_hours=24):
    """데이터 신선도 확인"""
    if not Path(file_path).exists():
        return False, "파일이 존재하지 않습니다."
    
    file_modified = datetime.fromtimestamp(Path(file_path).stat().st_mtime)
    age = datetime.now() - file_modified
    
    if age.total_seconds() > max_age_hours * 3600:
        return False, f"데이터가 {age.total_seconds()/3600:.1f}시간 전에 업데이트되었습니다."
    
    return True, "데이터가 최신입니다."

def log_user_action(action, details=None):
    """사용자 액션 로그"""
    log_entry = {
        'timestamp': datetime.now().isoformat(),
        'action': action,
        'details': details or {}
    }
    
    # 세션 상태에 저장
    if 'user_logs' not in st.session_state:
        st.session_state.user_logs = []
    
    st.session_state.user_logs.append(log_entry)
    
    # 로그 파일에 저장 (옵션)
    log_file = Path('logs/user_actions.log')
    log_file.parent.mkdir(exist_ok=True)
    
    try:
        with open(log_file, 'a', encoding='utf-8') as f:
            f.write(f"{json.dumps(log_entry, ensure_ascii=False)}\n")
    except Exception:
        pass  # 로그 실패는 무시

def get_system_info():
    """시스템 정보 반환"""
    import platform
    import psutil
    
    return {
        'platform': platform.platform(),
        'python_version': platform.python_version(),
        'cpu_count': psutil.cpu_count(),
        'memory_total': psutil.virtual_memory().total // (1024**3),  # GB
        'memory_available': psutil.virtual_memory().available // (1024**3)  # GB
    }

def create_error_handler(func):
    """에러 핸들러 데코레이터"""
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            st.error(f"오류가 발생했습니다: {str(e)}")
            log_user_action('error', {'function': func.__name__, 'error': str(e)})
            return None
    return wrapper