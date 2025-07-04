import streamlit as st
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from pathlib import Path

# 페이지 설정
st.set_page_config(page_title="기후기술 분류체계", page_icon="🔬", layout="wide")

# CSS 스타일
st.markdown("""
<style>
    .metric-card {
        background-color: #f0f2f6;
        padding: 1rem;
        border-radius: 10px;
        border-left: 5px solid #1f77b4;
    }
    .detail-card {
        background-color: #ffffff;
        padding: 1.5rem;
        border-radius: 10px;
        box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        margin: 1rem 0;
    }
</style>
""", unsafe_allow_html=True)

@st.cache_data
def load_classification_data():
    """분류체계 CSV 파일 로드"""
    possible_paths = [
        Path('data/climate_tech_classification.csv'),
        Path('../data/climate_tech_classification.csv'),
        Path('./data/climate_tech_classification.csv')
    ]
    
    for file_path in possible_paths:
        if file_path.exists():
            for encoding in ['utf-8-sig', 'utf-8', 'cp949', 'euc-kr']:
                try:
                    df = pd.read_csv(file_path, encoding=encoding)
                    st.sidebar.success(f"✅ 파일 로드: {file_path.name}")
                    return df
                except:
                    continue
    
    st.error("❌ climate_tech_classification.csv 파일을 찾을 수 없습니다.")
    return pd.DataFrame()

@st.cache_data  
def load_detailed_data():
    """상세정보 CSV 파일 로드"""
    possible_paths = [
        Path('data/climate_tech_detailed.csv'),
        Path('../data/climate_tech_detailed.csv'),
        Path('./data/climate_tech_detailed.csv')
    ]
    
    for file_path in possible_paths:
        if file_path.exists():
            for encoding in ['utf-8-sig', 'utf-8', 'cp949', 'euc-kr']:
                try:
                    df = pd.read_csv(file_path, encoding=encoding)
                    return df
                except:
                    continue
    
    return pd.DataFrame()

def filter_data(df, field, tech_type):
    """데이터 필터링"""
    filtered_df = df.copy()
    
    if field != "전체":
        filtered_df = filtered_df[filtered_df['L1_대분류'] == field]
    
    if tech_type != "전체":
        filtered_df = filtered_df[filtered_df['L2_중분류'] == tech_type]
    
    return filtered_df

def create_pie_chart(data, level='L2'):
    """파이차트 생성"""
    if level == 'L1':
        group_col = 'L1_대분류'
        title = "기후기술 대분류"
        color_sequence = ['#1f77b4', '#ff7f0e', '#2ca02c']
    elif level == 'L2':
        group_col = 'L2_중분류'
        title = "기후기술 중분류"
        color_sequence = px.colors.qualitative.Set3
    else:
        group_col = 'L3_소분류'
        title = "기후기술 소분류"
        color_sequence = px.colors.qualitative.Pastel
    
    if data.empty:
        return go.Figure().add_annotation(text="데이터가 없습니다", 
                                        xref="paper", yref="paper", 
                                        x=0.5, y=0.5, showarrow=False)
    
    counts = data[group_col].value_counts().reset_index()
    counts.columns = ['category', 'count']
    
    fig = px.pie(
        counts,
        values='count',
        names='category',
        title=title,
        color_discrete_sequence=color_sequence
    )
    
    fig.update_traces(
        textposition='inside',
        textinfo='percent+label',
        hovertemplate='<b>%{label}</b><br>개수: %{value}<br>비율: %{percent}<extra></extra>'
    )
    
    fig.update_layout(
        showlegend=True,
        height=400,
        font=dict(size=12),
        title_x=0.5
    )
    
    return fig

def create_sunburst_chart(data):
    """선버스트 차트 생성"""
    if data.empty:
        return go.Figure().add_annotation(text="데이터가 없습니다", 
                                        xref="paper", yref="paper", 
                                        x=0.5, y=0.5, showarrow=False)
    
    fig = go.Figure(go.Sunburst(
        labels=data['L1_대분류'].tolist() + data['L2_중분류'].tolist() + data['L3_소분류'].tolist(),
        parents=[''] * len(data['L1_대분류'].unique()) + 
                data['L1_대분류'].tolist() + 
                data['L2_중분류'].tolist(),
        values=[1] * len(data),
        branchvalues="total",
        hovertemplate='<b>%{label}</b><br>상위: %{parent}<br>개수: %{value}<extra></extra>',
    ))
    
    fig.update_layout(
        title="기후기술 분류체계 (계층구조)",
        title_x=0.5,
        height=500,
        font=dict(size=12)
    )
    
    return fig

def show_detailed_info(detailed_data, selected_tech):
    """상세정보 표시"""
    if selected_tech and not detailed_data.empty:
        detail = detailed_data[detailed_data['subtitle'] == selected_tech]
        
        if not detail.empty:
            detail = detail.iloc[0]
            
            st.markdown(f"""
            <div class="detail-card">
                <h3>🔬 {detail['subtitle']}</h3>
                <p><strong>분류:</strong> {detail['category']}</p>
                <p><strong>기술정의:</strong> {detail['definition']}</p>
                
                <div style="display: flex; gap: 2rem; margin: 1rem 0;">
                    <div style="flex: 1;">
                        <h4>🔑 키워드</h4>
                        <p><strong>국문:</strong> {detail['keywords_kor']}</p>
                        <p><strong>영문:</strong> {detail['keywords_eng']}</p>
                    </div>
                    <div style="flex: 1;">
                        <h4>🌍 기술수준</h4>
                        <p><strong>선도국:</strong> {detail['leading_country']}</p>
                        <p><strong>우리나라 수준:</strong> {detail['tech_level_pct']}</p>
                        <p><strong>기술격차:</strong> {detail['tech_gap']}</p>
                    </div>
                </div>
                
                <p><strong>세부분류:</strong> {detail['classification']}</p>
            </div>
            """, unsafe_allow_html=True)

def main():
    st.title("🔬 기후기술 분류체계")
    
    # 데이터 로드
    classification_data = load_classification_data()
    detailed_data = load_detailed_data()
    
    if classification_data.empty:
        st.error("분류체계 데이터를 로드할 수 없습니다. CSV 파일을 확인하세요.")
        return
    
    # 컨트롤 패널
    st.sidebar.header("🔧 필터 설정")
    
    # 필터 옵션
    fields = ["전체"] + sorted(classification_data['L1_대분류'].unique().tolist())
    selected_field = st.sidebar.selectbox("기후기술 분야", fields)
    
    # 기술 종류 옵션 (선택된 분야에 따라 동적 변경)
    if selected_field == "전체":
        tech_types = ["전체"] + sorted(classification_data['L2_중분류'].unique().tolist())
    else:
        filtered_for_types = classification_data[classification_data['L1_대분류'] == selected_field]
        tech_types = ["전체"] + sorted(filtered_for_types['L2_중분류'].unique().tolist())
    
    selected_tech_type = st.sidebar.selectbox("기술 종류", tech_types)
    
    # 차트 유형 선택
    chart_type = st.sidebar.radio("차트 유형", ["파이차트", "선버스트차트"])
    
    # 상세정보 표시 여부
    show_details = st.sidebar.checkbox("상세정보 표시", value=False)
    
    # 데이터 필터링
    filtered_data = filter_data(classification_data, selected_field, selected_tech_type)
    
    # 메트릭 표시
    col1, col2, col3, col4 = st.columns(4)
    
    with col1:
        st.markdown(f"""
        <div class="metric-card">
            <h3>{len(filtered_data)}</h3>
            <p>총 기술 수</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col2:
        unique_l1 = filtered_data['L1_대분류'].nunique()
        st.markdown(f"""
        <div class="metric-card">
            <h3>{unique_l1}</h3>
            <p>대분류 수</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col3:
        unique_l2 = filtered_data['L2_중분류'].nunique()
        st.markdown(f"""
        <div class="metric-card">
            <h3>{unique_l2}</h3>
            <p>중분류 수</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col4:
        unique_l3 = filtered_data['L3_소분류'].nunique()
        st.markdown(f"""
        <div class="metric-card">
            <h3>{unique_l3}</h3>
            <p>소분류 수</p>
        </div>
        """, unsafe_allow_html=True)
    
    # 메인 차트 영역
    if chart_type == "파이차트":
        col1, col2 = st.columns([2, 1])
        
        with col1:
            # 차트 레벨 선택
            chart_level = st.selectbox("분류 레벨", ["L1 (대분류)", "L2 (중분류)", "L3 (소분류)"])
            level = chart_level.split()[0]
            
            # 파이차트 생성
            fig = create_pie_chart(filtered_data, level)
            st.plotly_chart(fig, use_container_width=True)
        
        with col2:
            st.subheader("📊 분류 현황")
            
            if level == 'L1':
                counts = filtered_data['L1_대분류'].value_counts()
            elif level == 'L2':
                counts = filtered_data['L2_중분류'].value_counts()
            else:
                counts = filtered_data['L3_소분류'].value_counts()
            
            for category, count in counts.items():
                st.metric(category, f"{count}개")
    
    else:  # 선버스트차트
        fig = create_sunburst_chart(filtered_data)
        st.plotly_chart(fig, use_container_width=True)
    
    # 상세정보 섹션
    if show_details:
        st.markdown("---")
        st.subheader("📋 상세정보")
        
        # 기술 선택
        available_techs = detailed_data['subtitle'].unique().tolist() if not detailed_data.empty else []
        if available_techs:
            selected_tech = st.selectbox("기술 선택", ["선택하세요"] + available_techs)
            
            if selected_tech != "선택하세요":
                show_detailed_info(detailed_data, selected_tech)
        else:
            st.info("상세정보 데이터가 없습니다.")
    
    # 데이터 테이블
    if st.checkbox("데이터 테이블 보기"):
        st.subheader("📄 원본 데이터")
        st.dataframe(filtered_data, use_container_width=True)
    
    # 홈으로 돌아가기 버튼
    if st.button("🏠 메인으로 돌아가기"):
        st.switch_page("main.py")

if __name__ == "__main__":
    main()