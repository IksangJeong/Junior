import streamlit as st
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
from pathlib import Path

# 페이지 설정
st.set_page_config(page_title="기관 현황", page_icon="🏢", layout="wide")

# CSS 스타일
st.markdown("""
<style>
    .metric-card {
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        color: white;
        padding: 1.5rem;
        border-radius: 15px;
        text-align: center;
        margin: 0.5rem 0;
    }
    .info-card {
        background-color: #f8f9fa;
        padding: 1rem;
        border-radius: 10px;
        border-left: 5px solid #007bff;
        margin: 1rem 0;
    }
</style>
""", unsafe_allow_html=True)

@st.cache_data
def load_institution_data():
    """기관 현황 CSV 파일 로드"""
    possible_paths = [
        Path('data/institution_data.csv'),
        Path('../data/institution_data.csv'),
        Path('./data/institution_data.csv')
    ]
    
    for file_path in possible_paths:
        if file_path.exists():
            for encoding in ['utf-8-sig', 'utf-8', 'cp949', 'euc-kr']:
                try:
                    df = pd.read_csv(file_path, encoding=encoding)
                    st.sidebar.success(f"✅ 파일 로드: {file_path.name}")
                    
                    # 컬럼 자동 매핑
                    column_mapping = {
                        'scale': 'scale', 'Scale': 'scale', '규모': 'scale', '기관규모': 'scale',
                        'field': 'field', 'Field': 'field', '분야': 'field', '기술분야': 'field',
                        'tech_type': 'tech_type', 'Tech_type': 'tech_type', '기술종류': 'tech_type',
                        'revenue': 'revenue', 'Revenue': 'revenue', '매출액': 'revenue', '매출': 'revenue',
                        'employees': 'employees', 'Employees': 'employees', '종사자수': 'employees', '직원수': 'employees',
                        'rd_cost': 'rd_cost', 'R&D': 'rd_cost', '연구개발비': 'rd_cost', 'RD비용': 'rd_cost',
                        'researchers': 'researchers', 'Researchers': 'researchers', '연구자수': 'researchers', '연구원수': 'researchers'
                    }
                    
                    # 컬럼명 매핑 적용
                    for old_col, new_col in column_mapping.items():
                        if old_col in df.columns and new_col not in df.columns:
                            df[new_col] = df[old_col]
                    
                    # 필수 컬럼 기본값 설정
                    if 'scale' not in df.columns:
                        df['scale'] = '중기업'
                    if 'field' not in df.columns:
                        df['field'] = '감축'
                    if 'tech_type' not in df.columns:
                        df['tech_type'] = '재생에너지'
                    if 'revenue' not in df.columns:
                        df['revenue'] = 10000.0
                    if 'employees' not in df.columns:
                        df['employees'] = 50
                    if 'rd_cost' not in df.columns:
                        df['rd_cost'] = 500.0
                    if 'researchers' not in df.columns:
                        df['researchers'] = 10
                    if 'year' not in df.columns:
                        df['year'] = 2020
                    
                    return df
                except:
                    continue
    
    st.error("❌ institution_data.csv 파일을 찾을 수 없습니다.")
    return pd.DataFrame()

def filter_institution_data(df, scale, field, year):
    """기관 데이터 필터링"""
    if df.empty:
        return df
        
    filtered_df = df.copy()
    
    if scale != "전체":
        filtered_df = filtered_df[filtered_df['scale'] == scale]
    
    if field != "전체":
        filtered_df = filtered_df[filtered_df['field'] == field]
    
    filtered_df = filtered_df[filtered_df['year'] == year]
    
    return filtered_df

def create_bar_chart(data, metric, title):
    """막대차트 생성"""
    if data.empty:
        return go.Figure().add_annotation(text="데이터가 없습니다", 
                                        xref="paper", yref="paper", 
                                        x=0.5, y=0.5, showarrow=False)
    
    # 기술 종류별 집계
    agg_data = data.groupby('tech_type')[metric].sum().reset_index()
    agg_data = agg_data.sort_values(metric, ascending=True)
    
    fig = px.bar(
        agg_data,
        x=metric,
        y='tech_type',
        orientation='h',
        title=title,
        labels={metric: get_metric_label(metric), 'tech_type': '기술 종류'},
        color=metric,
        color_continuous_scale='viridis'
    )
    
    fig.update_layout(
        height=500,
        showlegend=False,
        title_x=0.5,
        xaxis_title=get_metric_label(metric),
        yaxis_title="기술 종류"
    )
    
    fig.update_traces(
        hovertemplate='<b>%{y}</b><br>' + get_metric_label(metric) + ': %{x:,.0f}<extra></extra>'
    )
    
    return fig

def create_correlation_scatter(data, x_metric, y_metric, field):
    """상관분석 산점도 생성"""
    if data.empty:
        return go.Figure().add_annotation(text="데이터가 없습니다", 
                                        xref="paper", yref="paper", 
                                        x=0.5, y=0.5, showarrow=False)
    
    fig = px.scatter(
        data,
        x=x_metric,
        y=y_metric,
        color='field',
        size='employees',
        hover_data=['scale', 'tech_type'],
        title=f"{get_metric_label(x_metric)} vs {get_metric_label(y_metric)} 상관분석",
        labels={
            x_metric: get_metric_label(x_metric),
            y_metric: get_metric_label(y_metric)
        },
        color_discrete_map={
            '감축': '#1f77b4',
            '적응': '#ff7f0e',
            '융복합': '#2ca02c'
        }
    )
    
    # 추세선 추가
    if len(data) > 1:
        z = np.polyfit(data[x_metric], data[y_metric], 1)
        p = np.poly1d(z)
        
        x_trend = np.linspace(data[x_metric].min(), data[x_metric].max(), 100)
        y_trend = p(x_trend)
        
        fig.add_trace(
            go.Scatter(
                x=x_trend,
                y=y_trend,
                mode='lines',
                name='추세선',
                line=dict(color='red', dash='dash'),
                hovertemplate='추세선<extra></extra>'
            )
        )
    
    fig.update_layout(
        height=400,
        title_x=0.5
    )
    
    return fig

def get_metric_label(metric):
    """메트릭 한글 라벨 반환"""
    labels = {
        'revenue': '매출액 (백만원)',
        'employees': '종사자 수 (명)',
        'rd_cost': '연구개발비 (백만원)',
        'researchers': '연구자 수 (명)'
    }
    return labels.get(metric, metric)

def calculate_correlation(data, x_metric, y_metric):
    """상관계수 계산"""
    if len(data) < 2:
        return 0
    return data[x_metric].corr(data[y_metric])

def main():
    st.title("🏢 기후기술 기관 현황")
    
    # 데이터 로드
    institution_data = load_institution_data()
    
    if institution_data.empty:
        st.error("기관 데이터를 로드할 수 없습니다. CSV 파일을 확인하세요.")
        return
    
    # 사이드바 컨트롤
    st.sidebar.header("🔧 필터 설정")
    
    # 연도 선택
    years = sorted(institution_data['year'].unique(), reverse=True)
    selected_year = st.sidebar.selectbox("연도", years)
    
    # 기관 규모 선택
    scales = ["전체"] + sorted(institution_data['scale'].unique().tolist())
    selected_scale = st.sidebar.selectbox("기관 규모", scales)
    
    # 기후기술 분야 선택
    fields = ["전체"] + sorted(institution_data['field'].unique().tolist())
    selected_field = st.sidebar.selectbox("기후기술 분야", fields)
    
    # 데이터 종류 선택
    metrics = {
        '매출액': 'revenue',
        '종사자 수': 'employees', 
        '연구개발비': 'rd_cost',
        '연구자 수': 'researchers'
    }
    selected_metric_name = st.sidebar.selectbox("데이터 종류", list(metrics.keys()))
    selected_metric = metrics[selected_metric_name]
    
    # 데이터 필터링
    filtered_data = filter_institution_data(institution_data, selected_scale, selected_field, selected_year)
    
    # 요약 통계
    st.subheader(f"📊 {selected_year}년 기관 현황 요약")
    
    col1, col2, col3, col4 = st.columns(4)
    
    with col1:
        total_revenue = filtered_data['revenue'].sum()
        st.markdown(f"""
        <div class="metric-card">
            <h3>{total_revenue:,.0f}</h3>
            <p>총 매출액 (백만원)</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col2:
        total_employees = filtered_data['employees'].sum()
        st.markdown(f"""
        <div class="metric-card">
            <h3>{total_employees:,}</h3>
            <p>총 종사자 수 (명)</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col3:
        total_rd = filtered_data['rd_cost'].sum()
        st.markdown(f"""
        <div class="metric-card">
            <h3>{total_rd:,.0f}</h3>
            <p>총 연구개발비 (백만원)</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col4:
        total_researchers = filtered_data['researchers'].sum()
        st.markdown(f"""
        <div class="metric-card">
            <h3>{total_researchers:,}</h3>
            <p>총 연구자 수 (명)</p>
        </div>
        """, unsafe_allow_html=True)
    
    # 메인 차트
    st.subheader(f"📈 {selected_metric_name} 현황")
    
    col1, col2 = st.columns([3, 1])
    
    with col1:
        # 막대차트
        bar_fig = create_bar_chart(
            filtered_data, 
            selected_metric, 
            f"기술 종류별 {selected_metric_name}"
        )
        st.plotly_chart(bar_fig, use_container_width=True)
    
    with col2:
        st.markdown("### 📋 상위 5개 기술")
        if not filtered_data.empty:
            top5_data = filtered_data.groupby('tech_type')[selected_metric].sum().sort_values(ascending=False).head(5)
            
            for i, (tech, value) in enumerate(top5_data.items(), 1):
                st.markdown(f"""
                <div class="info-card">
                    <strong>{i}. {tech}</strong><br>
                    {value:,.0f} {get_metric_label(selected_metric).split()[1]}
                </div>
                """, unsafe_allow_html=True)
    
    # 상관분석 섹션
    st.subheader("📊 상관분석")
    
    col1, col2 = st.columns(2)
    
    with col1:
        st.markdown("#### 매출액 vs 종사자 수")
        scatter_fig1 = create_correlation_scatter(
            filtered_data, 'revenue', 'employees', selected_field
        )
        st.plotly_chart(scatter_fig1, use_container_width=True)
        
        # 상관계수 표시
        corr1 = calculate_correlation(filtered_data, 'revenue', 'employees')
        st.metric("상관계수", f"{corr1:.3f}")
    
    with col2:
        st.markdown("#### 매출액 vs 연구개발비")
        scatter_fig2 = create_correlation_scatter(
            filtered_data, 'revenue', 'rd_cost', selected_field
        )
        st.plotly_chart(scatter_fig2, use_container_width=True)
        
        # 상관계수 표시
        corr2 = calculate_correlation(filtered_data, 'revenue', 'rd_cost')
        st.metric("상관계수", f"{corr2:.3f}")
    
    # 기관 규모별 분석
    st.subheader("🏭 기관 규모별 분석")
    
    if not filtered_data.empty:
        scale_analysis = filtered_data.groupby('scale').agg({
            'revenue': 'mean',
            'employees': 'mean',
            'rd_cost': 'mean',
            'researchers': 'mean'
        }).round(0)
        
        # 규모별 비교 차트
        scale_fig = go.Figure()
        
        for metric_name, metric_col in metrics.items():
            scale_fig.add_trace(go.Bar(
                name=metric_name,
                x=scale_analysis.index,
                y=scale_analysis[metric_col],
                hovertemplate=f'<b>%{{x}}</b><br>{metric_name}: %{{y:,.0f}}<extra></extra>'
            ))
        
        scale_fig.update_layout(
            title="기관 규모별 평균 현황",
            xaxis_title="기관 규모",
            yaxis_title="평균값",
            barmode='group',
            height=400,
            title_x=0.5
        )
        
        st.plotly_chart(scale_fig, use_container_width=True)
    
    # 홈으로 돌아가기
    if st.button("🏠 메인으로 돌아가기"):
        st.switch_page("main.py")

if __name__ == "__main__":
    main()