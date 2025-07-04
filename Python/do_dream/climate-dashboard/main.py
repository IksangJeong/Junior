import streamlit as st
import pandas as pd
from pathlib import Path

# 페이지 설정
st.set_page_config(
    page_title="한눈에 보는 기후기술 🌍",
    page_icon="🌍",
    layout="wide",
    initial_sidebar_state="expanded"
)

# CSS 스타일링
st.markdown("""
<style>
    .main-header {
        font-size: 3rem;
        font-weight: bold;
        text-align: center;
        color: #1f4e79;
        margin-bottom: 2rem;
    }
    .sub-header {
        font-size: 1.5rem;
        color: #2e8b57;
        margin: 1rem 0;
    }
    .card {
        background-color: #f8f9fa;
        padding: 1.5rem;
        border-radius: 10px;
        border-left: 5px solid #1f4e79;
        margin: 1rem 0;
    }
    .nav-button {
        background: linear-gradient(45deg, #1f4e79, #2e8b57);
        color: white;
        border: none;
        padding: 1rem 2rem;
        border-radius: 10px;
        font-size: 1.1rem;
        font-weight: bold;
        margin: 0.5rem;
        width: 100%;
        height: 120px;
        display: flex;
        align-items: center;
        justify-content: center;
        cursor: pointer;
        transition: transform 0.2s;
    }
    .nav-button:hover {
        transform: translateY(-5px);
        box-shadow: 0 5px 15px rgba(0,0,0,0.2);
    }
</style>
""", unsafe_allow_html=True)

def main():
    # 메인 헤더
    st.markdown('<h1 class="main-header">🌍 한눈에 보는 기후기술</h1>', unsafe_allow_html=True)
    
    # 프로젝트 소개
    st.markdown("""
    <div class="card">
        <h3>📊 프로젝트 개요</h3>
        <p>이 대시보드는 한국의 기후기술 현황을 한눈에 파악할 수 있도록 구성된 종합 분석 도구입니다.</p>
        <p><strong>주요 기능:</strong></p>
        <ul>
            <li>🔬 기후기술 분류체계 및 상세정보</li>
            <li>🏢 기관별 매출액, 종사자 수, 연구개발비 현황</li>
            <li>📋 연도별 특허 등록 현황</li>
            <li>🔄 기술 수명주기 단계별 분석</li>
            <li>🌏 해외 진출 현황 및 지역별 분포</li>
        </ul>
    </div>
    """, unsafe_allow_html=True)
    
    # 네비게이션 메뉴
    st.markdown('<h2 class="sub-header">📋 메뉴</h2>', unsafe_allow_html=True)
    
    col1, col2, col3 = st.columns(3)
    
    with col1:
        if st.button("🔬 기후기술 분류체계", key="nav1", help="기후기술의 분류체계를 파이차트로 시각화"):
            st.switch_page("pages/1_classification.py")
        
        if st.button("📋 기후기술 특허 현황", key="nav4", help="연도별 특허 등록 건수 분석"):
            st.switch_page("pages/3_patents.py")
    
    with col2:
        if st.button("🏢 기후기술 기관 현황", key="nav2", help="기관 규모별 매출액, 종사자 수 등 분석"):
            st.switch_page("pages/2_institutions.py")
        
        if st.button("🔄 기술 수명주기", key="nav5", help="기술 수명주기 단계별 현황"):
            st.switch_page("pages/4_lifecycle.py")
    
    with col3:
        if st.button("🌏 해외 진출 현황", key="nav6", help="지역별 기후기술 해외 진출 분석"):
            st.switch_page("pages/5_overseas.py")
    
    # 시스템 정보
    st.markdown("---")
    
    col1, col2, col3 = st.columns(3)
    
    with col1:
        st.markdown("""
        <div class="card">
            <h4>📊 데이터 현황</h4>
            <p>• 기후기술 분류: 45개 소분류</p>
            <p>• 기관 데이터: 2019-2020년</p>
            <p>• 특허 데이터: 누적 건수</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col2:
        st.markdown("""
        <div class="card">
            <h4>🔄 업데이트 정보</h4>
            <p>• 마지막 업데이트: 2024년</p>
            <p>• 데이터 소스: KOSIS, CTIS</p>
            <p>• 업데이트 주기: 연 1회</p>
        </div>
        """, unsafe_allow_html=True)
    
    with col3:
        st.markdown("""
        <div class="card">
            <h4>📞 문의사항</h4>
            <p>• 기술지원: Python + Streamlit</p>
            <p>• 데이터 문의: KOSIS 통계청</p>
            <p>• 버전: v1.0.0</p>
        </div>
        """, unsafe_allow_html=True)

    # 사이드바
    with st.sidebar:
        st.markdown("### 🌍 기후기술 대시보드")
        st.markdown("---")
        
        st.markdown("#### 📈 빠른 통계")
        
        # 샘플 통계 (실제 데이터로 교체 예정)
        st.metric("총 기후기술 분류", "45개", "3개 대분류")
        st.metric("분석 기간", "2019-2020", "2년간")
        st.metric("데이터 소스", "3개", "KOSIS, CTIS")
        
        st.markdown("---")
        st.markdown("#### 🔗 유용한 링크")
        st.markdown("- [KOSIS 통계청](https://kosis.kr)")
        st.markdown("- [기후기술정보시스템](https://www.ctis.re.kr)")
        st.markdown("- [Streamlit 문서](https://docs.streamlit.io)")

if __name__ == "__main__":
    main()