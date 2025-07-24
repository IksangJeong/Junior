import streamlit as st
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
from pathlib import Path
import json
from datetime import datetime
import sys
import os

# 상위 디렉토리 추가
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 페이지 설정
st.set_page_config(page_title="데이터 관리", page_icon="⚙️", layout="wide")

# CSS 스타일
st.markdown("""
<style>
    .data-card {
        background: linear-gradient(135deg, #6c5ce7 0%, #a29bfe 100%);
        color: white;
        padding: 1.5rem;
        border-radius: 15px;
        text-align: center;
        margin: 0.5rem 0;
    }
    .status-good {
        background-color: #00b894;
        color: white;
        padding: 0.5rem 1rem;
        border-radius: 20px;
        font-weight: bold;
    }
    .status-warning {
        background-color: #fdcb6e;
        color: #2d3436;
        padding: 0.5rem 1rem;
        border-radius: 20px;
        font-weight: bold;
    }
    .status-error {
        background-color: #e17055;
        color: white;
        padding: 0.5rem 1rem;
        border-radius: 20px;
        font-weight: bold;
    }
    .log-entry {
        background-color: #f8f9fa;
        padding: 0.8rem;
        border-radius: 8px;
        border-left: 4px solid #6c5ce7;
        margin: 0.5rem 0;
        font-family: monospace;
        font-size: 0.9rem;
    }
</style>
""", unsafe_allow_html=True)

def get_data_status():
    """데이터 상태 확인"""
    base_path = Path('assets/data')
    
    data_files = {
        'scraped': {
            'climate_tech_classification.csv': '기후기술 분류체계',
            'climate_tech_detailed.csv': '기후기술 상세정보'
        },
        'processed': {
            'institution_data.csv': '기관 현황',
            'patent_data.csv': '특허 현황', 
            'lifecycle_data.csv': '수명주기',
            'overseas_data.csv': '해외진출'
        }
    }
    
    status = {}
    
    for folder, files in data_files.items():
        status[folder] = {}
        folder_path = base_path / folder
        
        for filename, description in files.items():
            file_path = folder_path / filename
            
            if file_path.exists():
                # 파일 크기 및 수정 시간
                file_size = file_path.stat().st_size
                modified_time = datetime.fromtimestamp(file_path.stat().st_mtime)
                
                # 데이터 미리보기
                try:
                    df = pd.read_csv(file_path)
                    rows = len(df)
                    cols = len(df.columns)
                    
                    status[folder][filename] = {
                        'exists': True,
                        'description': description,
                        'size': file_size,
                        'modified': modified_time,
                        'rows': rows,
                        'cols': cols,
                        'preview': df.head(3)
                    }
                except Exception as e:
                    status[folder][filename] = {
                        'exists': True,
                        'description': description,
                        'size': file_size,
                        'modified': modified_time,
                        'error': str(e)
                    }
            else:
                status[folder][filename] = {
                    'exists': False,
                    'description': description
                }
    
    return status

def run_data_collection():
    """데이터 수집 실행"""
    try:
        # 데이터 수집 모듈 import
        from data.scraping import ClimateTechScraper
        
        with st.spinner("데이터 수집 중..."):
            scraper = ClimateTechScraper()
            
            # 진행 상황 표시
            progress_bar = st.progress(0)
            status_text = st.empty()
            
            # 기본 분류체계 크롤링
            status_text.text("기후기술 분류체계 수집 중...")
            progress_bar.progress(25)
            classification_df = scraper.scrape_classification_basic()
            
            # 상세정보 크롤링
            status_text.text("상세정보 수집 중...")
            progress_bar.progress(50)
            detailed_df = scraper.scrape_detailed_info()
            
            # 샘플 데이터 생성
            status_text.text("샘플 데이터 생성 중...")
            progress_bar.progress(75)
            
            if classification_df is None or detailed_df is None:
                scraper.create_sample_data()
            
            progress_bar.progress(100)
            status_text.text("데이터 수집 완료!")
            
            return True, "데이터 수집이 완료되었습니다."
            
    except Exception as e:
        return False, f"데이터 수집 실패: {str(e)}"

def create_data_overview_chart(status):
    """데이터 개요 차트 생성"""
    data_summary = []
    
    for folder, files in status.items():
        for filename, info in files.items():
            data_summary.append({
                'folder': folder,
                'file': filename,
                'description': info['description'],
                'exists': info['exists'],
                'rows': info.get('rows', 0) if info['exists'] else 0,
                'size_mb': info.get('size', 0) / (1024*1024) if info['exists'] else 0
            })
    
    df = pd.DataFrame(data_summary)
    
    # 존재하는 파일만 필터링
    existing_files = df[df['exists'] == True]
    
    if not existing_files.empty:
        # 파일별 데이터 행 수
        fig1 = px.bar(
            existing_files,
            x='description',
            y='rows',
            title="파일별 데이터 행 수",
            labels={'rows': '행 수', 'description': '파일'}
        )
        fig1.update_layout(xaxis_tickangle=-45)
        
        # 파일별 크기
        fig2 = px.pie(
            existing_files,
            values='size_mb',
            names='description',
            title="파일별 크기 분포 (MB)"
        )
        
        return fig1, fig2
    
    return None, None

def show_data_preview(status):
    """데이터 미리보기"""
    st.subheader("📋 데이터 미리보기")
    
    # 파일 선택
    available_files = []
    for folder, files in status.items():
        for filename, info in files.items():
            if info['exists'] and 'preview' in info:
                available_files.append(f"{folder}/{filename}")
    
    if available_files:
        selected_file = st.selectbox("미리보기할 파일 선택", available_files)
        
        if selected_file:
            folder, filename = selected_file.split('/')
            file_info = status[folder][filename]
            
            st.markdown(f"**파일:** {filename}")
            st.markdown(f"**설명:** {file_info['description']}")
            st.markdown(f"**크기:** {file_info['rows']} 행 × {file_info['cols']} 열")
            st.markdown(f"**수정일:** {file_info['modified'].strftime('%Y-%m-%d %H:%M:%S')}")
            
            # 데이터 미리보기
            st.dataframe(file_info['preview'], use_container_width=True)
            
            # 상세 정보
            if st.checkbox("상세 정보 보기"):
                file_path = Path(f'assets/data/{folder}/{filename}')
                full_df = pd.read_csv(file_path)
                
                col1, col2 = st.columns(2)
                
                with col1:
                    st.markdown("**데이터 타입:**")
                    st.dataframe(full_df.dtypes.reset_index().rename(columns={'index': '컬럼', 0: '타입'}))
                
                with col2:
                    st.markdown("**기본 통계:**")
                    if len(full_df.select_dtypes(include=['number']).columns) > 0:
                        st.dataframe(full_df.describe())
                    else:
                        st.info("수치형 데이터가 없습니다.")
    else:
        st.info("미리보기할 데이터가 없습니다.")

def show_system_info():
    """시스템 정보 표시"""
    st.subheader("🖥️ 시스템 정보")
    
    try:
        import platform
        import psutil
        
        col1, col2 = st.columns(2)
        
        with col1:
            st.markdown("**환경 정보:**")
            st.text(f"OS: {platform.system()} {platform.release()}")
            st.text(f"Python: {platform.python_version()}")
            st.text(f"CPU: {psutil.cpu_count()}코어")
            
        with col2:
            st.markdown("**메모리 정보:**")
            memory = psutil.virtual_memory()
            st.text(f"총 메모리: {memory.total // (1024**3):.1f} GB")
            st.text(f"사용 가능: {memory.available // (1024**3):.1f} GB")
            st.text(f"사용률: {memory.percent:.1f}%")
            
    except ImportError:
        st.warning("시스템 정보를 위해 psutil 패키지가 필요합니다.")

def main():
    st.title("⚙️ 데이터 관리")
    
    # 탭 구성
    tab1, tab2, tab3, tab4 = st.tabs(["📊 데이터 현황", "🔍 데이터 수집", "👀 데이터 미리보기", "⚙️ 시스템 정보"])
    
    with tab1:
        st.subheader("📊 데이터 현황")
        
        # 데이터 상태 확인
        with st.spinner("데이터 상태 확인 중..."):
            status = get_data_status()
        
        # 전체 요약
        total_files = sum(len(files) for files in status.values())
        existing_files = sum(
            sum(1 for info in files.values() if info['exists']) 
            for files in status.values()
        )
        
        col1, col2, col3, col4 = st.columns(4)
        
        with col1:
            st.markdown(f"""
            <div class="data-card">
                <h3>{total_files}</h3>
                <p>총 파일 수</p>
            </div>
            """, unsafe_allow_html=True)
        
        with col2:
            st.markdown(f"""
            <div class="data-card">
                <h3>{existing_files}</h3>
                <p>존재하는 파일</p>
            </div>
            """, unsafe_allow_html=True)
        
        with col3:
            total_rows = sum(
                sum(info.get('rows', 0) for info in files.values() if info['exists'])
                for files in status.values()
            )
            st.markdown(f"""
            <div class="data-card">
                <h3>{total_rows:,}</h3>
                <p>총 데이터 행 수</p>
            </div>
            """, unsafe_allow_html=True)
        
        with col4:
            completion_rate = (existing_files / total_files) * 100 if total_files > 0 else 0
            st.markdown(f"""
            <div class="data-card">
                <h3>{completion_rate:.1f}%</h3>
                <p>데이터 완성도</p>
            </div>
            """, unsafe_allow_html=True)
        
        # 상세 상태 표시
        st.subheader("📋 파일별 상태")
        
        for folder, files in status.items():
            st.markdown(f"### 📁 {folder.upper()}")
            
            for filename, info in files.items():
                col1, col2, col3 = st.columns([3, 1, 1])
                
                with col1:
                    st.markdown(f"**{filename}**")
                    st.markdown(f"*{info['description']}*")
                
                with col2:
                    if info['exists']:
                        st.markdown('<span class="status-good">존재함</span>', unsafe_allow_html=True)
                        if 'rows' in info:
                            st.text(f"{info['rows']:,} 행")
                    else:
                        st.markdown('<span class="status-error">없음</span>', unsafe_allow_html=True)
                
                with col3:
                    if info['exists']:
                        if 'modified' in info:
                            days_ago = (datetime.now() - info['modified']).days
                            if days_ago == 0:
                                st.text("오늘")
                            elif days_ago == 1:
                                st.text("1일 전")
                            else:
                                st.text(f"{days_ago}일 전")
                
                st.markdown("---")
        
        # 데이터 개요 차트
        fig1, fig2 = create_data_overview_chart(status)
        
        if fig1 and fig2:
            st.subheader("📈 데이터 개요")
            
            col1, col2 = st.columns(2)
            
            with col1:
                st.plotly_chart(fig1, use_container_width=True)
            
            with col2:
                st.plotly_chart(fig2, use_container_width=True)
    
    with tab2:
        st.subheader("🔍 데이터 수집")
        
        st.markdown("""
        ### 데이터 수집 옵션
        
        **자동 수집:** 웹 크롤링을 통해 실제 데이터를 수집합니다.
        - 기후기술 분류체계 (CTIS 웹사이트)
        - 기후기술 상세정보 (CTIS 웹사이트)
        
        **샘플 데이터:** 실제 데이터와 유사한 샘플 데이터를 생성합니다.
        - 기관 현황 데이터
        - 특허 현황 데이터
        - 수명주기 데이터
        - 해외진출 데이터
        """)
        
        col1, col2 = st.columns(2)
        
        with col1:
            if st.button("🕷️ 실제 데이터 수집", use_container_width=True):
                success, message = run_data_collection()
                
                if success:
                    st.success(message)
                    st.balloons()
                else:
                    st.error(message)
        
        with col2:
            if st.button("📋 샘플 데이터 생성", use_container_width=True):
                try:
                    from data.scraping import ClimateTechScraper
                    
                    with st.spinner("샘플 데이터 생성 중..."):
                        scraper = ClimateTechScraper()
                        scraper.create_sample_data()
                    
                    st.success("샘플 데이터가 생성되었습니다.")
                except Exception as e:
                    st.error(f"샘플 데이터 생성 실패: {str(e)}")
        
        # 수집 로그
        st.subheader("📝 수집 로그")
        
        if 'collection_logs' in st.session_state:
            for log in st.session_state.collection_logs[-10:]:  # 최근 10개
                st.markdown(f"""
                <div class="log-entry">
                    <strong>{log['timestamp']}</strong><br>
                    {log['message']}
                </div>
                """, unsafe_allow_html=True)
        else:
            st.info("수집 로그가 없습니다.")
    
    with tab3:
        show_data_preview(status)
    
    with tab4:
        show_system_info()
        
        # 추가 도구
        st.subheader("🔧 관리 도구")
        
        col1, col2, col3 = st.columns(3)
        
        with col1:
            if st.button("🗑️ 캐시 정리"):
                st.cache_data.clear()
                st.success("캐시가 정리되었습니다.")
        
        with col2:
            if st.button("📊 메모리 사용량 확인"):
                try:
                    import psutil
                    process = psutil.Process()
                    memory_info = process.memory_info()
                    st.info(f"현재 메모리 사용량: {memory_info.rss / (1024**2):.1f} MB")
                except ImportError:
                    st.warning("psutil 패키지가 필요합니다.")
        
        with col3:
            if st.button("🔄 앱 재시작 권장"):
                st.warning("변경사항 적용을 위해 앱을 재시작하세요.")
                st.markdown("```bash\nstreamlit run main.py\n```")
        
        # 설정 파일 관리
        st.subheader("⚙️ 설정 관리")
        
        config_path = Path('config/app_config.json')
        
        if config_path.exists():
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            
            st.json(config)
            
            if st.button("📥 설정 다운로드"):
                st.download_button(
                    label="설정 파일 다운로드",
                    data=json.dumps(config, indent=2, ensure_ascii=False),
                    file_name="app_config.json",
                    mime="application/json"
                )
        else:
            st.info("설정 파일이 없습니다.")
            
            if st.button("⚙️ 기본 설정 생성"):
                default_config = {
                    "app_name": "기후기술 대시보드",
                    "version": "1.0.0",
                    "data_refresh_interval": 24,
                    "chart_theme": "default",
                    "auto_refresh": False
                }
                
                config_path.parent.mkdir(exist_ok=True)
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                
                st.success("기본 설정이 생성되었습니다.")
                st.experimental_rerun()
        
        # 디버그 정보
        if st.checkbox("🐛 디버그 정보 표시"):
            st.subheader("디버그 정보")
            
            debug_info = {
                "Streamlit 버전": st.__version__,
                "현재 시간": datetime.now().isoformat(),
                "세션 상태 키": list(st.session_state.keys()),
                "환경 변수": dict(os.environ)
            }
            
            st.json(debug_info)
    
    # 하단 작업 버튼들
    st.markdown("---")
    
    col1, col2, col3 = st.columns([1, 1, 1])
    
    with col1:
        if st.button("🔄 데이터 새로고침", use_container_width=True):
            st.cache_data.clear()
            st.experimental_rerun()
    
    with col2:
        if st.button("📊 대시보드로 이동", use_container_width=True):
            st.switch_page("main.py")
    
    with col3:
        if st.button("📋 전체 데이터 내보내기", use_container_width=True):
            # 모든 데이터를 ZIP으로 압축하여 다운로드
            try:
                import zipfile
                import io
                
                zip_buffer = io.BytesIO()
                
                with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
                    for folder in ['scraped', 'processed']:
                        folder_path = Path(f'assets/data/{folder}')
                        if folder_path.exists():
                            for file_path in folder_path.glob('*.csv'):
                                zip_file.write(file_path, f'{folder}/{file_path.name}')
                
                zip_buffer.seek(0)
                
                st.download_button(
                    label="📥 ZIP 다운로드",
                    data=zip_buffer.getvalue(),
                    file_name=f"climate_tech_data_{datetime.now().strftime('%Y%m%d')}.zip",
                    mime="application/zip"
                )
                
            except Exception as e:
                st.error(f"데이터 내보내기 실패: {str(e)}")

if __name__ == "__main__":
    main()