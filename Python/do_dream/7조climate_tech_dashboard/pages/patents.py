import streamlit as st
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
from pathlib import Path

# 페이지 설정
st.set_page_config(page_title="특허 현황", page_icon="📋", layout="wide")

# CSS 스타일
st.markdown("""
<style>
    .patent-card {
        background: linear-gradient(135deg, #ff6b6b 0%, #ee5a24 100%);
        color: white;
        padding: 1.5rem;
        border-radius: 15px;
        text-align: center;
        margin: 0.5rem 0;
    }
    .top-tech-card {
        background-color: #fff5f5;
        padding: 1rem;
        border-radius: 10px;
        border-left: 5px solid #ff6b6b;
        margin: 0.5rem 0;
    }
</style>
""", unsafe_allow_html=True)

@st.cache_data
def load_patent_data():
    """특허 데이터 로드 또는 생성"""
    try:
        data_path = Path('assets/data/processed/patent_data.csv')
        if data_path.exists():
            return pd.read_csv(data_path)
        else:
            return create_sample_patent_data()
    except Exception as e:
        st.error(f"특허 데이터 로드 실패: {str(e)}")
        return create_sample_patent_data()

def create_sample_patent_data():
    """샘플 특허 데이터 생성"""
    np.random.seed(42)
    
    # 기후기술 분야 및 세부 기술
    tech_data = {
        '감축': {
            '재생에너지': ['태양광', '풍력', '수력', '지열', '바이오매스'],
            '비재생에너지': ['원자력', 'CCUS', '청정석탄'],
            '에너지효율': ['건물효율', '산업효율', 'LED조명'],
            '수송': ['전기차', '수소차', '바이오연료'],
            '에너지저장': ['배터리', '수소저장', '압축공기']
        },
        '적응': {
            '물관리': ['홍수방어', '가뭄대응', '수자원관리'],
            '농업': ['스마트팜', '기후적응작물', '정밀농업'],
            '해양수산': ['해수면상승대응', '수산업적응'],
            '생태계': ['생물다양성보전', '생태계복원'],
            '건강': ['폭염대응', '감염병대응']
        },
        '융복합': {
            'ICT융합': ['스마트그리드', 'AI기후예측', 'IoT모니터링'],
            '바이오융합': ['바이오에너지', '바이오소재'],
            '나노융합': ['나노태양전지', '나노필터']
        }
    }
    
    data = []
    years = [2018, 2019, 2020, 2021]
    
    for year in years:
        for field, categories in tech_data.items():
            for category, techs in categories.items():
                for tech in techs:
                    # 연도별 특허 증가 트렌드 반영
                    base_patents = np.random.randint(10, 200)
                    year_multiplier = 1 + (year - 2018) * 0.1  # 연간 10% 증가
                    
                    # 분야별 가중치
                    field_weights = {'감축': 1.5, '적응': 1.0, '융복합': 1.2}
                    
                    # 인기 기술 가중치
                    popular_techs = ['태양광', '전기차', '스마트그리드', '배터리']
                    tech_weight = 2.0 if tech in popular_techs else 1.0
                    
                    patent_count = int(base_patents * year_multiplier * field_weights[field] * tech_weight)
                    
                    data.append({
                        'year': year,
                        'field': field,
                        'category': category,
                        'tech_name': tech,
                        'patent_count': patent_count,
                        'cumulative_patents': patent_count * year  # 누적 특허 (단순화)
                    })
    
    return pd.DataFrame(data)

def filter_patent_data(df, year, field):
    """특허 데이터 필터링"""
    filtered_df = df[df['year'] == year].copy()
    
    if field != "전체":
        filtered_df = filtered_df[filtered_df['field'] == field]
    
    return filtered_df

def create_patent_bar_chart(data, top_n=15):
    """특허 건수 막대차트 생성"""
    if data.empty:
        return go.Figure().add_annotation(text="데이터가 없습니다", 
                                        xref="paper", yref="paper", 
                                        x=0.5, y=0.5, showarrow=False)
    
    # 상위 N개 기술만 표시
    top_data = data.nlargest(top_n, 'patent_count')
    
    fig = px.bar(
        top_data,
        x='patent_count',
        y='tech_name',
        color='field',
        title=f"상위 {top_n}개 기술별 특허 등록 건수",
        labels={'patent_count': '특허 건수', 'tech_name': '기술명'},
        orientation='h',
        color_discrete_map={
            '감축': '#1f77b4',
            '적응': '#ff7f0e', 
            '융복합': '#2ca02c'
        }
    )
    
    fig.update_layout(
        height=600,
        title_x=0.5,
        xaxis_title="특허 건수",
        yaxis_title="기술명",
        yaxis={'categoryorder': 'total ascending'}
    )
    
    fig.update_traces(
        hovertemplate='<b>%{y}</b><br>특허 건수: %{x:,}<br>분야: %{color}<extra></extra>'
    )
    
    return fig

def create_field_comparison_chart(data):
    """분야별 특허 비교 차트"""
    if data.empty:
        return go.Figure().add_annotation(text="데이터가 없습니다", 
                                        xref="paper", yref="paper", 
                                        x=0.5, y=0.5, showarrow=False)
    
    field_summary = data.groupby('field')['patent_count'].sum().reset_index()
    
    fig = px.pie(
        field_summary,
        values='patent_count',
        names='field',
        title="기후기술 분야별 특허 비율",
        color_discrete_map={
            '감축': '#1f77b4',
            '적응': '#ff7f0e',
            '융복합': '#2ca02c'
        }
    )
    
    fig.update_traces(
        textposition='inside',
        textinfo='percent+label',
        hovertemplate='<b>%{label}</b><br>특허 건수: %{value:,}<br>비율: %{percent}<extra></extra>'
    )
    
    fig.update_layout(
        height=400,
        title_x=0.5
    )
    
    return fig

def create_yearly_trend_chart(all_data, selected_field):
    """연도별 특허 트렌드 차트"""
    if selected_field == "전체":
        trend_data = all_data.groupby('year')['patent_count'].sum().reset_index()
        title = "전체 기후기술 연도별 특허 트렌드"
    else:
        filtered_data = all_data[all_data['field'] == selected_field]
        trend_data = filtered_data.groupby('year')['patent_count'].sum().reset_index()
        title = f"{selected_field} 기술 연도별 특허 트렌드"
    
    if trend_data.empty:
        return go.Figure().add_annotation(text="트렌드 데이터가 없습니다", 
                                        xref="paper", yref="paper", 
                                        x=0.5, y=0.5, showarrow=False)
    
    fig = px.line(
        trend_data,
        x='year',
        y='patent_count',
        title=title,
        markers=True,
        labels={'year': '연도', 'patent_count': '특허 건수'}
    )
    
    fig.update_layout(
        height=300,
        title_x=0.5,
        xaxis_title="연도",
        yaxis_title="특허 건수"
    )
    
    fig.update_traces(
        line=dict(width=3),
        marker=dict(size=8),
        hovertemplate='<b>%{x}년</b><br>특허 건수: %{y:,}<extra></extra>'
    )
    
    return fig

def create_category_heatmap(data):
    """카테고리별 히트맵"""
    if data.empty:
        return go.Figure().add_annotation(text="데이터가 없습니다", 
                                        xref="paper", yref="paper", 
                                        x=0.5, y=0.5, showarrow=False)
    
    # 피벗 테이블 생성
    pivot_data = data.pivot_table(
        values='patent_count', 
        index='category', 
        columns='field', 
        aggfunc='sum', 
        fill_value=0
    )
    
    fig = px.imshow(
        pivot_data.values,
        x=pivot_data.columns,
        y=pivot_data.index,
        title="분야별 카테고리 특허 히트맵",
        labels=dict(x="기술분야", y="기술카테고리", color="특허건수"),
        color_continuous_scale='Blues'
    )
    
    fig.update_layout(
        height=400,
        title_x=0.5
    )
    
    return fig

def main():
    st.title("📋 기후기술 특허 현황")
    
    # 데이터 로드
    patent_data = load_patent_data()
    
    # 사이드바 컨트롤
    st.sidebar.header("🔧 필터 설정")
    
    # 연도 선택
    years = sorted(patent_data['year'].unique(), reverse=True)
    selected_year = st.sidebar.selectbox("연도", years)
    
    # 기술분야 선택
    fields = ["전체"] + sorted(patent_data['field'].unique().tolist())
    selected_field = st.sidebar.selectbox("기후기술 분야", fields)
    
    # 표시할 기술 수
    top_n = st.sidebar.slider("표시할 기술 수", 5, 30, 15)
    
    # 데이터 필터링
    filtered_data = filter_patent_data(patent_data, selected_year, selected_field)
    
    # 요약 통계
    st.subheader(f"📊 {selected_year}년 특허 현황 요약")
    
    col1, col2, col3, col4 = st.columns(4)
    
    with col1:
        total_patents = filtered_data['patent_count'].sum()
        st.markdown(f"""
        <div class="patent-card">
            <h3>{total_patents:,}</h3>
            <p>총 특허 등록 건수</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col2:
        avg_patents = filtered_data['patent_count'].mean() if not filtered_data.empty else 0
        st.markdown(f"""
        <div class="patent-card">
            <h3>{avg_patents:.1f}</h3>
            <p>기술당 평균 특허</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col3:
        top_tech = filtered_data.loc[filtered_data['patent_count'].idxmax(), 'tech_name'] if not filtered_data.empty else "없음"
        st.markdown(f"""
        <div class="patent-card">
            <h3>{top_tech}</h3>
            <p>최다 특허 기술</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col4:
        unique_techs = filtered_data['tech_name'].nunique()
        st.markdown(f"""
        <div class="patent-card">
            <h3>{unique_techs}</h3>
            <p>특허 보유 기술 수</p>
        </div>
        """, unsafe_allow_html=True)
    
    # 메인 차트 영역
    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.subheader("📈 기술별 특허 등록 현황")
        bar_fig = create_patent_bar_chart(filtered_data, top_n)
        st.plotly_chart(bar_fig, use_container_width=True)
    
    with col2:
        st.subheader("🎯 분야별 특허 비율")
        pie_fig = create_field_comparison_chart(filtered_data)
        st.plotly_chart(pie_fig, use_container_width=True)
    
    # 트렌드 분석
    st.subheader("📊 연도별 특허 트렌드")
    trend_fig = create_yearly_trend_chart(patent_data, selected_field)
    st.plotly_chart(trend_fig, use_container_width=True)
    
    # 히트맵 및 상세 분석
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("🔥 카테고리별 히트맵")
        heatmap_fig = create_category_heatmap(filtered_data)
        st.plotly_chart(heatmap_fig, use_container_width=True)
    
    with col2:
        st.subheader("🏆 상위 10개 기술")
        top10_data = filtered_data.nlargest(10, 'patent_count')[['tech_name', 'field', 'patent_count']]
        
        for i, (idx, row) in enumerate(top10_data.iterrows(), 1):
            st.markdown(f"""
            <div class="top-tech-card">
                <strong>{i}. {row['tech_name']}</strong><br>
                <span style="color: #666;">분야: {row['field']}</span><br>
                <span style="color: #ff6b6b; font-weight: bold;">{row['patent_count']:,}건</span>
            </div>
            """, unsafe_allow_html=True)
    
    # 상세 분석 섹션
    if st.checkbox("📋 상세 분석 보기"):
        st.subheader("📄 상세 데이터")
        
        # 정렬 옵션
        sort_options = ['특허건수', '기술명', '분야', '카테고리']
        sort_by = st.selectbox("정렬 기준", sort_options)
        ascending = st.radio("정렬 순서", ["내림차순", "오름차순"]) == "오름차순"
        
        # 정렬 적용
        sort_columns = {
            '특허건수': 'patent_count',
            '기술명': 'tech_name', 
            '분야': 'field',
            '카테고리': 'category'
        }
        
        sorted_data = filtered_data.sort_values(
            sort_columns[sort_by], 
            ascending=ascending
        )[['tech_name', 'field', 'category', 'patent_count']]
        
        # 컬럼명 한글화
        sorted_data.columns = ['기술명', '분야', '카테고리', '특허건수']
        
        st.dataframe(sorted_data, use_container_width=True)
        
        # 통계 요약
        st.subheader("📊 통계 요약")
        col1, col2, col3 = st.columns(3)
        
        with col1:
            st.metric("총 특허 건수", f"{filtered_data['patent_count'].sum():,}")
            st.metric("평균 특허 건수", f"{filtered_data['patent_count'].mean():.1f}")
        
        with col2:
            st.metric("최대 특허 건수", f"{filtered_data['patent_count'].max():,}")
            st.metric("최소 특허 건수", f"{filtered_data['patent_count'].min():,}")
        
        with col3:
            st.metric("표준편차", f"{filtered_data['patent_count'].std():.1f}")
            st.metric("중간값", f"{filtered_data['patent_count'].median():.1f}")
        
        # 데이터 다운로드
        csv = sorted_data.to_csv(index=False, encoding='utf-8-sig')
        st.download_button(
            label="📥 CSV 다운로드",
            data=csv,
            file_name=f"특허현황_{selected_year}년_{selected_field}.csv",
            mime="text/csv"
        )
    
    # 홈으로 돌아가기
    if st.button("🏠 메인으로 돌아가기"):
        st.switch_page("main.py")

if __name__ == "__main__":
    main()