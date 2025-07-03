#!/usr/bin/env python3
"""
기후기술 대시보드 실행 스크립트
"""

import subprocess
import sys
import os
from pathlib import Path

def check_requirements():
    """필요한 패키지 설치 확인"""
    print("📦 패키지 설치 확인 중...")
    
    try:
        import streamlit
        import pandas
        import plotly
        from bs4 import BeautifulSoup  # 수정된 부분
        print("✅ 모든 필수 패키지가 설치되어 있습니다.")
        return True
    except ImportError as e:
        print(f"❌ 필수 패키지가 누락되었습니다: {e}")
        print("📥 requirements.txt에서 패키지를 설치하세요:")
        print("pip install -r requirements.txt")
        return False

def setup_directories():
    """필요한 디렉토리 구조 확인 및 생성"""
    print("📁 디렉토리 구조 확인 중...")
    
    required_dirs = [
        'assets/data/raw',
        'assets/data/processed',
        'assets/data/scraped',
        'assets/images',
        '.streamlit'
    ]
    
    for dir_path in required_dirs:
        Path(dir_path).mkdir(parents=True, exist_ok=True)
    
    print("✅ 디렉토리 구조가 준비되었습니다.")

def run_data_collection():
    """데이터 수집 실행"""
    print("\n🔍 데이터 수집을 시작하시겠습니까?")
    choice = input("y/n: ").lower().strip()
    
    if choice == 'y':
        print("📊 데이터 크롤링 시작...")
        try:
            from data.scraping import main as scraping_main
            scraping_main()
            print("✅ 데이터 수집 완료!")
        except Exception as e:
            print(f"⚠️ 데이터 수집 실패: {e}")
            print("샘플 데이터로 진행합니다.")
    else:
        print("📋 샘플 데이터로 진행합니다.")

def run_streamlit_app():
    """Streamlit 앱 실행"""
    print("\n🚀 Streamlit 앱을 시작합니다...")
    print("=" * 50)
    print("🌍 기후기술 대시보드")
    print("📍 URL: http://localhost:8502")
    print("❌ 종료하려면 Ctrl+C를 누르세요")
    print("=" * 50)
    
    try:
        # 포트 8502로 실행 (8501이 사용 중일 수 있음)
        subprocess.run([
            sys.executable, 
            "-m", 
            "streamlit", 
            "run", 
            "main.py",
            "--server.port=8502",
            "--server.headless=false"
        ])
    except KeyboardInterrupt:
        print("\n👋 앱을 종료합니다.")
    except Exception as e:
        print(f"❌ 앱 실행 실패: {e}")
        print("💡 직접 실행 방법:")
        print("   streamlit run main.py --server.port=8502")

def main():
    """메인 실행 함수"""
    print("🌍 기후기술 대시보드 시작")
    print("=" * 30)
    
    # 1. 패키지 확인
    if not check_requirements():
        return
    
    # 2. 디렉토리 설정
    setup_directories()
    
    # 3. 데이터 수집 (선택사항)
    run_data_collection()
    
    # 4. 앱 실행
    run_streamlit_app()

if __name__ == "__main__":
    main()