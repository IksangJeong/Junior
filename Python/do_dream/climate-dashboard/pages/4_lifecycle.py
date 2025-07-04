import streamlit as st
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
import sys
from pathlib import Path

# 상위 디렉토리 추가
sys.path.append(str(Path(__file__).parent.parent))
from utils.data_loader import load_lifecycle_data

# 페이지 설정
st.set_page_config(page_title="기술 수명주기", page_icon="🔄", layout="wide")

# CSS 스타일
st.markdown("""
<style>
    .lifecycle-card {
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        color: white;
        padding: 1.5rem;
        border-radius: 15px;
        text-align: center;
        margin: 0.5rem 0;
    }
    .stage-info {
        background-color: #f8f9fa;
        padding: 1rem;
        border-radius: 10px;
        border-left: 5px solid #667eea;
        margin: 1rem 0;
    }
    .legend-item {
        display: inline-block;
        margin: 0.2rem 0.5rem;
        padding: 0.3rem 0.8rem;
        border-radius: 15px;
        font-size: 0.85rem;
        cursor: pointer;
    }
</style>
""", unsafe_allow_html=True)

def filter_lifecycle_data(df, year, field, tech_type):
    """수명주기 데이터 필터링"""
    filtered_df = df[df['year'] == year].copy()
    
    if field != "전체":
        filtered_df = filtered_df[filtered_df['field'] == field]
    
    if tech_type != "전체":
        filtered_df = filtered_df[filtered_df['tech_name'] == tech_type]
    
    return filtered_df

def create_stage_summary_table(data):
    """수명주기 단계별 요약 테이블"""
    if data.empty:
        return pd.DataFrame()
    
    summary = data.groupby('lifecycle_stage').agg({
        'project_count': ['sum', 'mean', 'count'],
        'tech_name': 'nunique'
    }).round(1)
    
    summary.columns = ['총 프로젝트', '평균 프로젝트', '기술 수', '기술 종류 수']
    summary = summary.reset_index()
    
    return summary

def create_lifecycle_line_chart(data, selected_stages):
    """수명주기 라인차트 생성"""
    if data.empty:
        return go.Figure().add_annotation(text="데이터가 없습니다", 
                                        xref="paper", yref="paper", 
                                        x=0.5, y=0.5, showarrow=False)
    
    # 선택된 단계만 필터링
    if selected_stages:
        data = data[data['lifecycle_stage'].isin(selected_stages)]
    
    # 기술별 단계별 집계
    line_data = data.groupby(['tech_name', 'lifecycle_stage', 'stage_order'])['project_count'].sum().reset_index()
    
    fig = go.Figure()
    
    # 색상 팔레트
    colors = px.colors.qualitative.Set3
    
    for i, tech in enumerate(line_data['tech_name'].unique()):
        tech_data = line_data[line_data['tech_name'] == tech].sort_values('stage_order')
        
        fig.add_trace(go.Scatter(
            x=tech_data['lifecycle_stage'],
            y=tech_data['project_count'],
            mode='lines+markers',
            name=tech,
            line=dict(width=3, color=colors[i % len(colors)]),
            marker=dict(size=8),
            hovertemplate=f'<b>{tech}</b><br>단계: %{{x}}<br>프로젝트 수: %{{y}}<extra></extra>'
        ))
    
    fig.update_layout(
        title="기술별 수명주기 단계 분포",
        xaxis_title="수명주기 단계",
        yaxis_title="프로젝트 수",
        height=500,
        title_x=0.5,
        hovermode='x unified'
    )
    
    return fig

def create_stage_distribution_chart(data):
    """단계별 분포 차트"""
    if data.empty:
        return go.Figure().add_annotation(text="데이터가 없습니다", 
                                        xref="paper", yref="paper", 
                                        x=0.5, y=0.5, showarrow=False)
    
    stage_summary = data.groupby('lifecycle_stage')['project_count'].sum().reset_index()
    stage_summary = stage_summary.sort_values('project_count', ascending=True)
    
    fig = px.bar(
        stage_summary,
        x='project_count',
        y='lifecycle_stage',
        orientation='h',
        title="수명주기 단계별 총 프로젝트 수",
        labels={'project_count': '프로젝트 수', 'lifecycle_stage': '수명주기 단계'},
        color='project_count',
        color_continuous_scale='viridis'
    )
    
    fig.update_layout(
        height=400,
        title_x=0.5,
        showlegend=False
    )
    
    fig.update_traces(
        hovertemplate='<b>%{y}</b><br>프로젝트 수: %{x:,}<extra></extra>'
    )
    
    return fig

def create_field_stage_heatmap(data):
    """분야별 단계 히트맵"""
    if data.empty:
        return go.Figure().add_annotation(text="데이터가 없습니다", 
                                        xref="paper", yref="paper", 
                                        x=0.5, y=0.5, showarrow=False)
    
    # 피벗 테이블 생성
    pivot_data = data.pivot_table(
        values='project_count',
        index='lifecycle_stage',
        columns='field',
        aggfunc='sum',
        fill_value=0
    )
    
    fig = px.imshow(
        pivot_data.values,
        x=pivot_data.columns,
        y=pivot_data.index,
        title="기술분야별 수명주기 단계 히트맵",
        labels=dict(x="기술분야", y="수명주기단계", color="프로젝트수"),
        color_continuous_scale='Blues'
    )
    
    fig.update_layout(
        height=400,
        title_x=0.5
    )
    
    return fig

def create_tech_maturity_radar(data):
    """기술 성숙도 레이더 차트"""
    if data.empty:
        return go.Figure().add_annotation(text="데이터가 없습니다", 
                                        xref="paper", yref="paper", 
                                        x=0.5, y=0.5, showarrow=False)
    
    # 기술별 성숙도 계산 (후반 단계 비중으로 계산)
    maturity_stages = ['사업화준비', '시장진입', '시장확산', '성숙기']
    
    tech_maturity = []
    for tech in data['tech_name'].unique():
        tech_data = data[data['tech_name'] == tech]
        total_projects = tech_data['project_count'].sum()
        
        if total_projects > 0:
            mature_projects = tech_data[tech_data['lifecycle_stage'].isin(maturity_stages)]['project_count'].sum()
            maturity_score = (mature_projects / total_projects) * 100
        else:
            maturity_score = 0
        
        tech_maturity.append({
            'tech_name': tech,
            'maturity_score': maturity_score,
            'total_projects': total_projects
        })
    
    maturity_df = pd.DataFrame(tech_maturity)
    maturity_df = maturity_df.sort_values('maturity_score', ascending=False).head(8)
    
    fig = go.Figure()
    
    fig.add_trace(go.Scatterpolar(
        r=maturity_df['maturity_score'],
        theta=maturity_df['tech_name'],
        fill='toself',
        name='기술 성숙도',
        line_color='rgba(102, 126, 234, 0.8)',
        fillcolor='rgba(102, 126, 234, 0.3)'
    ))
    
    fig.update_layout(
        polar=dict(
            radialaxis=dict(
                visible=True,
                range=[0, 100]
            )
        ),
        title="기술별 성숙도 레이더 차트 (%)",
        title_x=0.5,
        height=400
    )
    
    return fig

def get_stage_info():
    """수명주기 단계별 정보"""
    return {
        '기초연구': '기본 원리 연구 및 이론 개발',
        '응용연구': '실용화를 위한 응용 기술 연구',
        '개발연구': '상용화 가능한 기술 개발',
        '시제품제작': '프로토타입 제작 및 테스트',
        '사업화준비': '양산 체계 구축 및 시장 분석',
        '시장진입': '초기 상용화 및 시장 진입',
        '시장확산': '시장 점유율 확대',
        '성숙기': '시장 안정화 및 기술 고도화'
    }

def main():
    st.title("🔄 기후기술 수명주기")
    
    # 데이터 로드
    lifecycle_data = load_lifecycle_data()
    
    # 사이드바 컨트롤
    st.sidebar.header("🔧 필터 설정")
    
    # 연도 선택
    years = sorted(lifecycle_data['year'].unique(), reverse=True)
    selected_year = st.sidebar.selectbox("연도", years)
    
    # 기술분야 선택
    fields = ["전체"] + sorted(lifecycle_data['field'].unique().tolist())
    selected_field = st.sidebar.selectbox("기후기술 분야", fields)
    
    # 기술 종류 선택 (동적 업데이트)
    if selected_field == "전체":
        tech_types = ["전체"] + sorted(lifecycle_data['tech_name'].unique().tolist())
    else:
        filtered_for_types = lifecycle_data[lifecycle_data['field'] == selected_field]
        tech_types = ["전체"] + sorted(filtered_for_types['tech_name'].unique().tolist())
    
    selected_tech_type = st.sidebar.selectbox("기후기술 종류", tech_types)
    
    # 수명주기 단계 선택 (다중 선택)
    all_stages = sorted(lifecycle_data['lifecycle_stage'].unique(), 
                       key=lambda x: lifecycle_data[lifecycle_data['lifecycle_stage']==x]['stage_order'].iloc[0])
    selected_stages = st.sidebar.multiselect(
        "표시할 수명주기 단계",
        all_stages,
        default=all_stages
    )
    
    # 데이터 필터링
    filtered_data = filter_lifecycle_data(lifecycle_data, selected_year, selected_field, selected_tech_type)
    
    # 요약 통계
    st.subheader(f"📊 {selected_year}년 수명주기 현황")
    
    col1, col2, col3, col4 = st.columns(4)
    
    with col1:
        total_projects = filtered_data['project_count'].sum()
        st.markdown(f"""
        <div class="lifecycle-card">
            <h3>{total_projects:,}</h3>
            <p>총 프로젝트 수</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col2:
        unique_techs = filtered_data['tech_name'].nunique()
        st.markdown(f"""
        <div class="lifecycle-card">
            <h3>{unique_techs}</h3>
            <p>기술 종류 수</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col3:
        # 평균 성숙도 계산
        mature_stages = ['사업화준비', '시장진입', '시장확산', '성숙기']
        mature_projects = filtered_data[filtered_data['lifecycle_stage'].isin(mature_stages)]['project_count'].sum()
        maturity_pct = (mature_projects / total_projects * 100) if total_projects > 0 else 0
        
        st.markdown(f"""
        <div class="lifecycle-card">
            <h3>{maturity_pct:.1f}%</h3>
            <p>평균 성숙도</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col4:
        # 가장 활발한 단계
        if not filtered_data.empty:
            most_active_stage = filtered_data.groupby('lifecycle_stage')['project_count'].sum().idxmax()
        else:
            most_active_stage = "없음"
        
        st.markdown(f"""
        <div class="lifecycle-card">
            <h3>{most_active_stage}</h3>
            <p>가장 활발한 단계</p>
        </div>
        """, unsafe_allow_html=True)
    
    # 수명주기 단계별 요약 테이블
    st.subheader("📋 수명주기 단계별 현황")
    
    col1, col2 = st.columns([3, 1])
    
    with col1:
        summary_table = create_stage_summary_table(filtered_data)
        if not summary_table.empty:
            # 컬럼명 한글화
            display_table = summary_table.copy()
            display_table.columns = ['수명주기 단계', '총 프로젝트', '평균 프로젝트', '참여 기술 수', '기술 종류']
            st.dataframe(display_table, use_container_width=True)
        else:
            st.info("표시할 데이터가 없습니다.")
    
    with col2:
        st.subheader("📖 단계별 설명")
        stage_info = get_stage_info()
        
        for stage, description in stage_info.items():
            if stage in selected_stages:
                st.markdown(f"""
                <div class="stage-info">
                    <strong>{stage}</strong><br>
                    <small>{description}</small>
                </div>
                """, unsafe_allow_html=True)
    
    # 라인차트 섹션
    st.subheader("📈 기술별 수명주기 분포")
    
    line_fig = create_lifecycle_line_chart(filtered_data, selected_stages)
    st.plotly_chart(line_fig, use_container_width=True)
    
    # 범례 커스터마이징 (기술 종류별 색상)
    if not filtered_data.empty:
        st.markdown("#### 🎨 기술 범례")
        tech_names = filtered_data['tech_name'].unique()
        colors = px.colors.qualitative.Set3
        
        legend_html = ""
        for i, tech in enumerate(tech_names):
            color = colors[i % len(colors)]
            legend_html += f"""
            <span class="legend-item" style="background-color: {color}; color: white;">
                {tech}
            </span>
            """
        
        st.markdown(legend_html, unsafe_allow_html=True)
    
    # 추가 분석 차트
    col1, col2 = st.columns(2)
    
    with col1:
        st.subheader("📊 단계별 프로젝트 분포")
        dist_fig = create_stage_distribution_chart(filtered_data)
        st.plotly_chart(dist_fig, use_container_width=True)
    
    with col2:
        st.subheader("🎯 기술 성숙도 분석")
        radar_fig = create_tech_maturity_radar(filtered_data)
        st.plotly_chart(radar_fig, use_container_width=True)
    
    # 히트맵 분석
    st.subheader("🔥 분야별 수명주기 히트맵")
    heatmap_fig = create_field_stage_heatmap(filtered_data)
    st.plotly_chart(heatmap_fig, use_container_width=True)
    
    # 연도별 트렌드 분석
    if len(lifecycle_data['year'].unique()) > 1:
        st.subheader("📈 연도별 수명주기 트렌드")
        
        # 연도별 성숙도 변화
        yearly_maturity = []
        mature_stages = ['사업화준비', '시장진입', '시장확산', '성숙기']
        
        for year in sorted(lifecycle_data['year'].unique()):
            year_data = lifecycle_data[lifecycle_data['year'] == year]
            if selected_field != "전체":
                year_data = year_data[year_data['field'] == selected_field]
            
            total_projects = year_data['project_count'].sum()
            mature_projects = year_data[year_data['lifecycle_stage'].isin(mature_stages)]['project_count'].sum()
            
            maturity_pct = (mature_projects / total_projects * 100) if total_projects > 0 else 0
            
            yearly_maturity.append({
                'year': year,
                'maturity_percentage': maturity_pct,
                'total_projects': total_projects
            })
        
        maturity_trend_df = pd.DataFrame(yearly_maturity)
        
        trend_fig = px.line(
            maturity_trend_df,
            x='year',
            y='maturity_percentage',
            title=f"연도별 기술 성숙도 트렌드 ({'전체' if selected_field == '전체' else selected_field})",
            labels={'year': '연도', 'maturity_percentage': '성숙도 (%)'},
            markers=True
        )
        
        trend_fig.update_layout(
            height=300,
            title_x=0.5
        )
        
        trend_fig.update_traces(
            line=dict(width=3),
            marker=dict(size=8),
            hovertemplate='<b>%{x}년</b><br>성숙도: %{y:.1f}%<extra></extra>'
        )
        
        st.plotly_chart(trend_fig, use_container_width=True)
    
    # 상세 분석
    if st.checkbox("📋 상세 분석 보기"):
        st.subheader("📄 상세 데이터")
        
        # 정렬 옵션
        sort_options = ['프로젝트 수', '기술명', '수명주기 단계']
        sort_by = st.selectbox("정렬 기준", sort_options)
        ascending = st.radio("정렬 순서", ["내림차순", "오름차순"]) == "오름차순"
        
        # 정렬 적용
        sort_columns = {
            '프로젝트 수': 'project_count',
            '기술명': 'tech_name',
            '수명주기 단계': 'stage_order'
        }
        
        sorted_data = filtered_data.sort_values(
            sort_columns[sort_by],
            ascending=ascending
        )[['tech_name', 'field', 'lifecycle_stage', 'project_count']]
        
        # 컬럼명 한글화
        sorted_data.columns = ['기술명', '분야', '수명주기단계', '프로젝트수']
        
        st.dataframe(sorted_data, use_container_width=True)
        
        # 데이터 다운로드
        csv = sorted_data.to_csv(index=False, encoding='utf-8-sig')
        st.download_button(
            label="📥 CSV 다운로드",
            data=csv,
            file_name=f"수명주기_{selected_year}년_{selected_field}.csv",
            mime="text/csv"
        )
        
        # 추가 통계
        st.subheader("📊 추가 통계")
        
        col1, col2, col3 = st.columns(3)
        
        with col1:
            st.metric("전체 기술 수", filtered_data['tech_name'].nunique())
            st.metric("전체 단계 수", filtered_data['lifecycle_stage'].nunique())
        
        with col2:
            avg_projects = filtered_data['project_count'].mean()
            max_projects = filtered_data['project_count'].max()
            st.metric("평균 프로젝트 수", f"{avg_projects:.1f}")
            st.metric("최대 프로젝트 수", f"{max_projects}")
        
        with col3:
            # 가장 활발한 기술
            if not filtered_data.empty:
                most_active_tech = filtered_data.groupby('tech_name')['project_count'].sum().idxmax()
                most_active_count = filtered_data.groupby('tech_name')['project_count'].sum().max()
                st.metric("가장 활발한 기술", most_active_tech)
                st.metric("해당 기술 프로젝트 수", f"{most_active_count}")
    
    # 홈으로 돌아가기
    if st.button("🏠 메인으로 돌아가기"):
        st.switch_page("main.py")

if __name__ == "__main__":
    main()