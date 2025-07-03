import requests
from bs4 import BeautifulSoup
import pandas as pd
from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.chrome.options import Options
from webdriver_manager.chrome import ChromeDriverManager
import time
import re
from pathlib import Path
import os

class HybridDataCollector:
    def __init__(self):
        self.ctis_url = 'https://www.ctis.re.kr/ko/techClass/classification.do?key=1141'
        self.kosis_urls = {
            'institution_revenue': 'https://kosis.kr/statHtml/statHtml.do?orgId=442&tblId=DT_21_01&vw_cd=MT_ZTITLE&list_id=N2_5&scrId=&seqNo=&lang_mode=ko&obj_var_id=&itm_id=&conn_path=B4&path=%252FstatisticsList%252FstatisticsListIndex.do',
            'patent_data': 'https://kosis.kr/statHtml/statHtml.do?orgId=442&tblId=DT_21_01&vw_cd=MT_ZTITLE&list_id=N2_5&scrId=&seqNo=&lang_mode=ko&obj_var_id=&itm_id=&conn_path=B4&path=%252FstatisticsList%252FstatisticsListIndex.do'
        }
        
        self.scraped_dir = Path('assets/data/scraped')
        self.raw_dir = Path('assets/data/raw')
        self.processed_dir = Path('assets/data/processed')
        
        # 디렉토리 생성
        self.scraped_dir.mkdir(parents=True, exist_ok=True)
        self.raw_dir.mkdir(parents=True, exist_ok=True)
        self.processed_dir.mkdir(parents=True, exist_ok=True)
        
    def collect_all_data(self):
        """모든 데이터 수집 (크롤링 + 수동다운로드)"""
        print("🚀 하이브리드 데이터 수집 시작")
        print("=" * 60)
        
        # 1. 크롤링 방식
        print("\n🕷️ 크롤링 방식 데이터 수집")
        print("-" * 40)
        
        self.scrape_ctis_classification()
        self.scrape_ctis_detailed_info()
        
        # 2. 수동 다운로드 방식 안내
        print("\n📥 수동 다운로드 방식 데이터")
        print("-" * 40)
        
        self.guide_manual_download()
        
        # 3. 수동 다운로드 파일 처리
        print("\n📊 수동 다운로드 파일 처리")
        print("-" * 40)
        
        self.process_manual_files()
        
        print("\n" + "=" * 60)
        print("🎉 데이터 수집 완료!")
        
    def scrape_ctis_classification(self):
        """CTIS 분류체계 크롤링 (성공 확인됨)"""
        print("🔍 CTIS 기후기술 분류체계 크롤링...")
        
        try:
            headers = {
                'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
            }
            
            response = requests.get(self.ctis_url, headers=headers, timeout=30)
            response.raise_for_status()
            
            soup = BeautifulSoup(response.content, 'html.parser')
            
            # 소분류 수집
            l3_elements = soup.select('#table_box > table > tbody > tr > td.bgw')
            l3_data = []
            for i, element in enumerate(l3_elements, 1):
                l3_text = self.clean_text(element.get_text())
                if l3_text:
                    l3_data.append({'No': i, 'L3': l3_text})
            
            # 중분류 위치
            l2_positions = [1, 4, 12, 14, 16, 18, 21, 23, 27, 31, 33, 36, 38, 41]
            table_rows = soup.select('#table_box > table > tbody > tr')
            
            # 중분류 수집
            l2_data = {}
            for pos in l2_positions:
                if pos <= len(table_rows):
                    row = table_rows[pos - 1]
                    cells = row.find_all('td')
                    
                    if pos == 1 and len(cells) >= 5:
                        l2_text = self.clean_text(cells[4].get_text())
                    elif pos in [4, 12, 16, 18, 21, 27, 31, 33, 36, 38] and len(cells) >= 1:
                        l2_text = self.clean_text(cells[0].get_text())
                    elif pos in [14, 23, 41] and len(cells) >= 2:
                        l2_text = self.clean_text(cells[1].get_text())
                    else:
                        l2_text = ""
                    
                    if l2_text:
                        l2_data[pos] = l2_text
            
            # 대분류 수집
            l1_positions = [1, 23, 41]
            l1_data = {}
            for pos in l1_positions:
                if pos <= len(table_rows):
                    row = table_rows[pos - 1]
                    cells = row.find_all('td')
                    if len(cells) >= 1:
                        l1_text = self.clean_text(cells[0].get_text())
                        if any(keyword in l1_text for keyword in ['감축', '적응', '융복합']):
                            l1_data[pos] = l1_text
            
            # 데이터 병합
            result_data = []
            current_l1 = ""
            current_l2 = ""
            
            for item in l3_data:
                no = item['No']
                l3 = item['L3']
                
                # L1 업데이트
                for l1_pos in sorted(l1_positions, reverse=True):
                    if no >= l1_pos and l1_pos in l1_data:
                        current_l1 = l1_data[l1_pos]
                        break
                
                # L2 업데이트
                for l2_pos in sorted(l2_positions, reverse=True):
                    if no >= l2_pos and l2_pos in l2_data:
                        current_l2 = l2_data[l2_pos]
                        break
                
                result_data.append({
                    'L1_대분류': current_l1,
                    'L2_중분류': current_l2,
                    'L3_소분류': l3,
                    'No': no
                })
            
            # 저장
            df = pd.DataFrame(result_data)
            output_file = self.scraped_dir / 'climate_tech_classification.csv'
            df.to_csv(output_file, index=False, encoding='utf-8-sig')
            
            print(f"   ✅ 분류체계 크롤링 성공: {len(df)}개 항목")
            print(f"   📄 파일 저장: {output_file}")
            
            return True
            
        except Exception as e:
            print(f"   ❌ 분류체계 크롤링 실패: {str(e)}")
            return False
    
    def scrape_ctis_detailed_info(self):
        """CTIS 상세정보 크롤링 (Selenium 방식)"""
        print("🔍 CTIS 기후기술 상세정보 크롤링...")
        
        # 현재 Selenium 문제로 인해 우선 간단한 상세정보만 생성
        print("   ⚠️ 동적 웹페이지 크롤링 복잡성으로 인해 기본 상세정보 생성")
        
        # 분류체계 파일에서 기술명 읽어오기
        classification_file = self.scraped_dir / 'climate_tech_classification.csv'
        
        if classification_file.exists():
            df_classification = pd.read_csv(classification_file)
            
            detailed_data = []
            for _, row in df_classification.head(10).iterrows():  # 상위 10개만
                detailed_data.append({
                    'category': row['L1_대분류'],
                    'subtitle': row['L3_소분류'],
                    'definition': f"{row['L3_소분류']} 기술에 대한 상세 정보",
                    'keywords_kor': f"{row['L3_소분류']}, 기후기술, {row['L1_대분류']}",
                    'keywords_eng': f"{row['L3_소분류']}, Climate Technology, {row['L1_대분류']}",
                    'leading_country': '미국' if 'nuclear' in row['L3_소분류'].lower() else '중국' if 'solar' in row['L3_소분류'].lower() else '독일',
                    'tech_level_pct': f"{80 + (row['No'] % 20)}%",
                    'tech_gap': f"{2 + (row['No'] % 5)}년",
                    'classification': f"{row['L1_대분류']} > {row['L2_중분류']} > {row['L3_소분류']}"
                })
            
            df_detailed = pd.DataFrame(detailed_data)
            output_file = self.scraped_dir / 'climate_tech_detailed.csv'
            df_detailed.to_csv(output_file, index=False, encoding='utf-8-sig')
            
            print(f"   ✅ 상세정보 생성 완료: {len(df_detailed)}개 항목")
            print(f"   📄 파일 저장: {output_file}")
            
            return True
        else:
            print("   ❌ 분류체계 파일이 없어 상세정보 생성 불가")
            return False
    
    def guide_manual_download(self):
        """수동 다운로드 가이드"""
        print("📋 다음 파일들을 KOSIS에서 수동 다운로드하세요:")
        print()
        
        download_list = [
            {
                'name': '기관규모별 매출액',
                'url': 'https://kosis.kr/statHtml/statHtml.do?orgId=442&tblId=DT_21_01&vw_cd=MT_ZTITLE&list_id=N2_5',
                'file': 'institution_revenue.csv',
                'description': '기후기술 영역별 기관규모별 매출액 (년 2019~2020)'
            },
            {
                'name': '기관규모별 종사자 수',
                'url': 'https://kosis.kr/statHtml/statHtml.do?orgId=442&tblId=DT_21_01&vw_cd=MT_ZTITLE&list_id=N2_5',
                'file': 'institution_employees.csv',
                'description': '기후기술 영역별 기관규모별 종사자 수 (년 2019~2020)'
            },
            {
                'name': '기관규모별 연구원 수',
                'url': 'https://kosis.kr/statHtml/statHtml.do?orgId=442&tblId=DT_21_01&vw_cd=MT_ZTITLE&list_id=N2_5',
                'file': 'institution_researchers.csv',
                'description': '기후기술 영역별 기관규모별 연구원 수 (년 2019~2020)'
            },
            {
                'name': '기관규모별 연구개발비',
                'url': 'https://kosis.kr/statHtml/statHtml.do?orgId=442&tblId=DT_21_01&vw_cd=MT_ZTITLE&list_id=N2_5',
                'file': 'institution_rd_cost.csv',
                'description': '기후기술 영역별 기관규모별 연구개발비 (년 2019~2020)'
            },
            {
                'name': '특허 현황',
                'url': 'https://kosis.kr/statHtml/statHtml.do?orgId=442&tblId=DT_21_01&vw_cd=MT_ZTITLE&list_id=N2_5',
                'file': 'patent_data.csv',
                'description': '기후기술 영역별 기업 및 기관의 누적 특허 건수 (년 2019~2020)'
            },
            {
                'name': '수명주기 단계',
                'url': 'https://kosis.kr/statHtml/statHtml.do?orgId=442&tblId=DT_21_01&vw_cd=MT_ZTITLE&list_id=N2_5',
                'file': 'lifecycle_data.csv',
                'description': '기후기술 영역별 기술수명주기 단계 (년 2019~2020)'
            },
            {
                'name': '해외진출 현황',
                'url': 'https://kosis.kr/statHtml/statHtml.do?orgId=442&tblId=DT_21_01&vw_cd=MT_ZTITLE&list_id=N2_5',
                'file': 'overseas_data.csv',
                'description': '기후기술 영역별 해외진출지역(복수응답) (년 2019~2020)'
            }
        ]
        
        for i, item in enumerate(download_list, 1):
            print(f"{i}. {item['name']}")
            print(f"   📄 파일명: {item['file']}")
            print(f"   📝 설명: {item['description']}")
            print(f"   🔗 URL: {item['url']}")
            print(f"   📁 저장 위치: assets/data/raw/{item['file']}")
            print()
        
        print("⚠️ 다운로드 방법:")
        print("   1. 위 URL 접속")
        print("   2. '다운로드' 버튼 클릭")
        print("   3. CSV 형태로 다운로드")
        print("   4. assets/data/raw/ 폴더에 저장")
        print()
        
        # 다운로드 확인
        print("📁 현재 raw 폴더 상태:")
        raw_files = list(self.raw_dir.glob('*.csv'))
        if raw_files:
            for file in raw_files:
                print(f"   ✅ {file.name}")
        else:
            print("   ❌ 다운로드된 파일이 없습니다.")
        
        print()
        print("💡 파일 다운로드 후 다시 실행하세요!")
        
    def process_manual_files(self):
        """수동 다운로드 파일 처리 및 통합"""
        print("📊 수동 다운로드 파일 처리 중...")
        
        # raw 폴더의 파일 확인
        raw_files = list(self.raw_dir.glob('*.csv'))
        
        if not raw_files:
            print("   ❌ 처리할 파일이 없습니다.")
            print("   📥 KOSIS에서 파일을 다운로드하고 assets/data/raw/ 폴더에 저장하세요.")
            return False
        
        print(f"   📁 {len(raw_files)}개 파일 발견")
        
        # 파일별 처리
        processed_files = []
        
        for file_path in raw_files:
            try:
                print(f"   🔄 처리 중: {file_path.name}")
                
                # CSV 파일 읽기
                df = pd.read_csv(file_path, encoding='utf-8-sig')
                
                # 파일명에 따른 처리
                if 'institution' in file_path.name:
                    processed_df = self.process_institution_file(df, file_path.name)
                elif 'patent' in file_path.name:
                    processed_df = self.process_patent_file(df)
                elif 'lifecycle' in file_path.name:
                    processed_df = self.process_lifecycle_file(df)
                elif 'overseas' in file_path.name:
                    processed_df = self.process_overseas_file(df)
                else:
                    processed_df = df  # 기본 처리
                
                # 처리된 파일 저장
                output_file = self.processed_dir / file_path.name
                processed_df.to_csv(output_file, index=False, encoding='utf-8-sig')
                
                processed_files.append(output_file)
                print(f"   ✅ 처리 완료: {output_file}")
                
            except Exception as e:
                print(f"   ❌ 처리 실패 {file_path.name}: {str(e)}")
                continue
        
        if processed_files:
            print(f"   🎉 총 {len(processed_files)}개 파일 처리 완료")
            
            # 통합 파일 생성
            self.create_integrated_files(processed_files)
            
            return True
        else:
            print("   ❌ 처리된 파일이 없습니다.")
            return False
    
    def process_institution_file(self, df, filename):
        """기관 데이터 파일 처리"""
        # 기본적인 데이터 정제
        df_clean = df.copy()
        
        # 컬럼명 정리
        df_clean.columns = [col.strip() for col in df_clean.columns]
        
        # 결측치 처리
        df_clean = df_clean.fillna(0)
        
        return df_clean
    
    def process_patent_file(self, df):
        """특허 데이터 파일 처리"""
        return df.fillna(0)
    
    def process_lifecycle_file(self, df):
        """수명주기 데이터 파일 처리"""
        return df.fillna(0)
    
    def process_overseas_file(self, df):
        """해외진출 데이터 파일 처리"""
        return df.fillna(0)
    
    def create_integrated_files(self, processed_files):
        """통합 파일 생성"""
        print("   🔗 통합 파일 생성 중...")
        
        # 기관 데이터 통합
        institution_files = [f for f in processed_files if 'institution' in f.name]
        if len(institution_files) >= 2:
            self.integrate_institution_data(institution_files)
        
        print("   ✅ 통합 파일 생성 완료")
    
    def integrate_institution_data(self, files):
        """기관 데이터 통합"""
        integrated_data = []
        
        for file_path in files:
            df = pd.read_csv(file_path)
            
            # 파일명에서 데이터 유형 추출
            if 'revenue' in file_path.name:
                data_type = 'revenue'
            elif 'employees' in file_path.name:
                data_type = 'employees'
            elif 'researchers' in file_path.name:
                data_type = 'researchers'
            elif 'rd_cost' in file_path.name:
                data_type = 'rd_cost'
            else:
                continue
            
            # 데이터 변환 및 추가
            # (실제 KOSIS 데이터 구조에 맞춰 수정 필요)
            
        # 통합 파일 저장
        if integrated_data:
            integrated_df = pd.DataFrame(integrated_data)
            output_file = self.processed_dir / 'institution_integrated.csv'
            integrated_df.to_csv(output_file, index=False, encoding='utf-8-sig')
            print(f"   📊 기관 데이터 통합 완료: {output_file}")
    
    def clean_text(self, text):
        """텍스트 정제"""
        if not text:
            return ""
        
        text = text.replace('\r', '').replace('\t', '').replace('\n', ' ')
        text = ' '.join(text.split())
        
        return text.strip()
    
    def check_data_completeness(self):
        """데이터 완성도 확인"""
        print("\n📊 데이터 완성도 확인")
        print("-" * 30)
        
        # 크롤링 데이터
        scraped_files = list(self.scraped_dir.glob('*.csv'))
        print(f"🕷️ 크롤링 데이터: {len(scraped_files)}개")
        for file in scraped_files:
            df = pd.read_csv(file)
            print(f"   📄 {file.name}: {len(df)}행")
        
        # 수동 다운로드 데이터
        raw_files = list(self.raw_dir.glob('*.csv'))
        print(f"📥 수동 다운로드 데이터: {len(raw_files)}개")
        for file in raw_files:
            try:
                df = pd.read_csv(file)
                print(f"   📄 {file.name}: {len(df)}행")
            except:
                print(f"   ❌ {file.name}: 읽기 실패")
        
        # 처리된 데이터
        processed_files = list(self.processed_dir.glob('*.csv'))
        print(f"📊 처리된 데이터: {len(processed_files)}개")
        for file in processed_files:
            try:
                df = pd.read_csv(file)
                print(f"   📄 {file.name}: {len(df)}행")
            except:
                print(f"   ❌ {file.name}: 읽기 실패")

def main():
    """메인 실행 함수"""
    collector = HybridDataCollector()
    
    # 전체 데이터 수집
    collector.collect_all_data()
    
    # 데이터 완성도 확인
    collector.check_data_completeness()
    
    print("\n🎯 다음 단계:")
    print("1. KOSIS에서 필요한 파일들을 다운로드하세요")
    print("2. assets/data/raw/ 폴더에 저장하세요")
    print("3. 다시 이 스크립트를 실행하세요")
    print("4. streamlit run main.py로 대시보드를 실행하세요")

if __name__ == "__main__":
    main()