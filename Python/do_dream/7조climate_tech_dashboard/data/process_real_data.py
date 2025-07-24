import pandas as pd
from pathlib import Path
import numpy as np

class RealDataProcessor:
    def __init__(self):
        self.raw_dir = Path('assets/data/raw')
        self.processed_dir = Path('assets/data/processed')
        self.processed_dir.mkdir(parents=True, exist_ok=True)
        
    def process_all_data(self):
        """모든 실제 데이터 처리"""
        print("🚀 실제 KOSIS 데이터 처리 시작")
        print("=" * 50)
        
        # 1. 기관 데이터 통합
        self.process_institution_data()
        
        # 2. 특허 데이터 처리
        self.process_patent_data()
        
        # 3. 수명주기 데이터 처리
        self.process_lifecycle_data()
        
        # 4. 해외진출 데이터 처리
        self.process_overseas_data()
        
        print("\n🎉 모든 데이터 처리 완료!")
        self.verify_processed_data()
    
    def read_csv_safely(self, file_path):
        """안전한 CSV 읽기"""
        encodings = ['utf-8', 'utf-8-sig', 'cp949', 'euc-kr']
        
        for encoding in encodings:
            try:
                df = pd.read_csv(file_path, encoding=encoding)
                print(f"   ✅ {encoding} 인코딩으로 읽기 성공: {df.shape}")
                return df
            except Exception as e:
                continue
        
        print(f"   ❌ 모든 인코딩 실패: {file_path.name}")
        return None
    
    def process_institution_data(self):
        """기관 데이터 통합 처리"""
        print("\n📊 기관 데이터 처리 중...")
        
        # 기관 관련 파일들
        institution_files = {
            'revenue': 'institution_revenue.csv',
            'employees': 'institution_employees.csv',
            'researchers': 'institution_researchers.csv',
            'rd_cost': 'institution_rd_cost.csv'
        }
        
        # 각 파일 읽기
        data_dict = {}
        for metric, filename in institution_files.items():
            file_path = self.raw_dir / filename
            if file_path.exists():
                df = self.read_csv_safely(file_path)
                if df is not None:
                    data_dict[metric] = df
                    print(f"   📄 {filename}: {df.shape}")
            else:
                print(f"   ❌ {filename} 파일 없음")
        
        if data_dict:
            # 데이터 통합
            integrated_data = self.integrate_institution_data(data_dict)
            
            # 저장
            output_file = self.processed_dir / 'institution_data.csv'
            integrated_data.to_csv(output_file, index=False, encoding='utf-8-sig')
            print(f"   ✅ 통합 파일 저장: {output_file} ({len(integrated_data)}행)")
        else:
            print("   ❌ 처리할 기관 데이터가 없습니다.")
    
    def integrate_institution_data(self, data_dict):
        """기관 데이터 통합"""
        print("   🔗 데이터 통합 중...")
        
        integrated_rows = []
        
        # 첫 번째 데이터를 기준으로 구조 파악
        first_key = list(data_dict.keys())[0]
        base_df = data_dict[first_key]
        
        print(f"   📋 기준 데이터: {first_key}")
        print(f"   📊 컬럼: {base_df.columns.tolist()}")
        
        # 각 행을 처리
        for idx, row in base_df.iterrows():
            if idx == 0:  # 헤더 행은 건너뛰기
                continue
                
            try:
                # 기본 정보 추출 (첫 번째, 두 번째 컬럼에서)
                if len(row) >= 2:
                    field_info = str(row.iloc[0]) if pd.notna(row.iloc[0]) else ""
                    scale_info = str(row.iloc[1]) if pd.notna(row.iloc[1]) else ""
                    
                    # 연도별 데이터 추출 (나머지 컬럼들)
                    for col_idx, value in enumerate(row.iloc[2:], 2):
                        if pd.notna(value) and str(value).replace('.', '').replace(',', '').isdigit():
                            # 가능한 연도 추정
                            year = 2019 + (col_idx - 2) % 4  # 2019, 2020, 2021, 2022 순환
                            
                            # 통합 행 생성
                            integrated_row = {
                                'year': year,
                                'field': self.clean_field_name(field_info),
                                'scale': self.clean_scale_name(scale_info),
                                'tech_type': '전체',  # 기본값
                                first_key: float(str(value).replace(',', '')) if str(value).replace(',', '').replace('.', '').isdigit() else 0
                            }
                            
                            # 다른 메트릭 데이터도 추가
                            for metric, df in data_dict.items():
                                if metric != first_key and idx < len(df):
                                    try:
                                        other_value = df.iloc[idx, col_idx] if col_idx < len(df.columns) else 0
                                        if pd.notna(other_value):
                                            integrated_row[metric] = float(str(other_value).replace(',', '')) if str(other_value).replace(',', '').replace('.', '').isdigit() else 0
                                        else:
                                            integrated_row[metric] = 0
                                    except:
                                        integrated_row[metric] = 0
                            
                            integrated_rows.append(integrated_row)
                            
            except Exception as e:
                continue
        
        # DataFrame 생성
        if integrated_rows:
            df = pd.DataFrame(integrated_rows)
            
            # 데이터 정제
            df = df[df['year'].between(2019, 2022)]  # 유효한 연도만
            df = df.dropna(subset=[first_key])  # 주요 메트릭이 있는 행만
            
            # 누락된 컬럼 기본값 설정
            required_cols = ['revenue', 'employees', 'researchers', 'rd_cost']
            for col in required_cols:
                if col not in df.columns:
                    df[col] = 0
            
            print(f"   ✅ 통합 완료: {len(df)}행")
            return df
        else:
            print("   ❌ 통합 데이터 생성 실패")
            return pd.DataFrame()
    
    def clean_field_name(self, field_str):
        """분야명 정제"""
        field_str = str(field_str).strip()
        
        if '감축' in field_str:
            return '감축'
        elif '적응' in field_str:
            return '적응'
        elif '융복합' in field_str or '융합' in field_str:
            return '융복합'
        else:
            return '기타'
    
    def clean_scale_name(self, scale_str):
        """규모명 정제"""
        scale_str = str(scale_str).strip()
        
        if '대기업' in scale_str or '대규모' in scale_str:
            return '대기업'
        elif '중기업' in scale_str or '중규모' in scale_str:
            return '중기업'
        elif '소기업' in scale_str or '소규모' in scale_str:
            return '소기업'
        elif '연구' in scale_str:
            return '연구기관'
        else:
            return '기타'
    
    def process_patent_data(self):
        """특허 데이터 처리"""
        print("\n📋 특허 데이터 처리 중...")
        
        file_path = self.raw_dir / 'patent_data.csv'
        if not file_path.exists():
            print("   ❌ patent_data.csv 파일 없음")
            return
        
        df = self.read_csv_safely(file_path)
        if df is None:
            return
        
        # 특허 데이터 구조 변환
        processed_data = []
        
        for idx, row in df.iterrows():
            if idx == 0:  # 헤더 건너뛰기
                continue
                
            try:
                field_info = str(row.iloc[0]) if len(row) > 0 and pd.notna(row.iloc[0]) else ""
                tech_info = str(row.iloc[1]) if len(row) > 1 and pd.notna(row.iloc[1]) else ""
                
                # 연도별 데이터 처리
                for col_idx in range(2, min(len(row), 6)):  # 최대 4년치 데이터
                    if pd.notna(row.iloc[col_idx]):
                        value = str(row.iloc[col_idx]).replace(',', '')
                        if value.replace('.', '').isdigit():
                            processed_data.append({
                                'year': 2019 + (col_idx - 2),
                                'field': self.clean_field_name(field_info),
                                'tech_name': tech_info,
                                'patent_count': int(float(value)),
                                'category': self.clean_field_name(field_info)
                            })
            except:
                continue
        
        if processed_data:
            result_df = pd.DataFrame(processed_data)
            output_file = self.processed_dir / 'patent_data.csv'
            result_df.to_csv(output_file, index=False, encoding='utf-8-sig')
            print(f"   ✅ 특허 데이터 저장: {output_file} ({len(result_df)}행)")
        else:
            print("   ❌ 특허 데이터 처리 실패")
    
    def process_lifecycle_data(self):
        """수명주기 데이터 처리"""
        print("\n🔄 수명주기 데이터 처리 중...")
        
        file_path = self.raw_dir / 'lifecycle_data.csv'
        if not file_path.exists():
            print("   ❌ lifecycle_data.csv 파일 없음")
            return
        
        df = self.read_csv_safely(file_path)
        if df is None:
            return
        
        # 수명주기 단계
        lifecycle_stages = ['기초연구', '응용연구', '개발연구', '시제품제작', 
                           '사업화준비', '시장진입', '시장확산', '성숙기']
        
        processed_data = []
        
        for idx, row in df.iterrows():
            if idx == 0:
                continue
                
            try:
                field_info = str(row.iloc[0]) if len(row) > 0 and pd.notna(row.iloc[0]) else ""
                tech_info = str(row.iloc[1]) if len(row) > 1 and pd.notna(row.iloc[1]) else ""
                
                # 각 수명주기 단계별 데이터
                for stage_idx, stage in enumerate(lifecycle_stages):
                    if stage_idx + 2 < len(row) and pd.notna(row.iloc[stage_idx + 2]):
                        value = str(row.iloc[stage_idx + 2]).replace(',', '')
                        if value.replace('.', '').isdigit():
                            processed_data.append({
                                'year': 2020,  # 기본 연도
                                'field': self.clean_field_name(field_info),
                                'tech_name': tech_info,
                                'lifecycle_stage': stage,
                                'project_count': int(float(value)),
                                'stage_order': stage_idx + 1
                            })
            except:
                continue
        
        if processed_data:
            result_df = pd.DataFrame(processed_data)
            output_file = self.processed_dir / 'lifecycle_data.csv'
            result_df.to_csv(output_file, index=False, encoding='utf-8-sig')
            print(f"   ✅ 수명주기 데이터 저장: {output_file} ({len(result_df)}행)")
        else:
            print("   ❌ 수명주기 데이터 처리 실패")
    
    def process_overseas_data(self):
        """해외진출 데이터 처리"""
        print("\n🌏 해외진출 데이터 처리 중...")
        
        file_path = self.raw_dir / 'overseas_data.csv'
        if not file_path.exists():
            print("   ❌ overseas_data.csv 파일 없음")
            return
        
        df = self.read_csv_safely(file_path)
        if df is None:
            return
        
        # 지역 좌표 매핑
        region_coords = {
            '동남아시아': (10.0, 110.0, '베트남, 태국, 인도네시아'),
            '중국': (35.0, 104.0, '중국'),
            '일본': (36.0, 138.0, '일본'),
            '유럽': (54.0, 15.0, '독일, 프랑스, 영국'),
            '북미': (45.0, -100.0, '미국, 캐나다'),
            '중동': (25.0, 45.0, 'UAE, 사우디아라비아'),
            '남미': (-15.0, -60.0, '브라질, 아르헨티나'),
            '아프리카': (0.0, 20.0, '남아프리카공화국, 이집트')
        }
        
        processed_data = []
        
        for idx, row in df.iterrows():
            if idx == 0:
                continue
                
            try:
                field_info = str(row.iloc[0]) if len(row) > 0 and pd.notna(row.iloc[0]) else ""
                tech_info = str(row.iloc[1]) if len(row) > 1 and pd.notna(row.iloc[1]) else ""
                
                # 지역별 데이터 처리
                for col_idx in range(2, len(row)):
                    if pd.notna(row.iloc[col_idx]):
                        value = str(row.iloc[col_idx]).replace(',', '')
                        if value.replace('.', '').isdigit():
                            # 지역명 추정 (컬럼 인덱스 기반)
                            region_names = list(region_coords.keys())
                            region = region_names[(col_idx - 2) % len(region_names)]
                            
                            coords = region_coords[region]
                            
                            processed_data.append({
                                'year': 2020,
                                'region': region,
                                'field': self.clean_field_name(field_info),
                                'tech_name': tech_info,
                                'export_count': int(float(value)),
                                'latitude': coords[0],
                                'longitude': coords[1],
                                'countries': coords[2]
                            })
            except:
                continue
        
        if processed_data:
            result_df = pd.DataFrame(processed_data)
            output_file = self.processed_dir / 'overseas_data.csv'
            result_df.to_csv(output_file, index=False, encoding='utf-8-sig')
            print(f"   ✅ 해외진출 데이터 저장: {output_file} ({len(result_df)}행)")
        else:
            print("   ❌ 해외진출 데이터 처리 실패")
    
    def verify_processed_data(self):
        """처리된 데이터 검증"""
        print("\n📊 처리된 데이터 검증")
        print("-" * 30)
        
        processed_files = list(self.processed_dir.glob('*.csv'))
        
        for file_path in processed_files:
            try:
                df = pd.read_csv(file_path)
                print(f"✅ {file_path.name}: {len(df)}행 x {len(df.columns)}열")
                
                # 샘플 데이터 표시
                if len(df) > 0:
                    print(f"   📋 컬럼: {list(df.columns)}")
                    print(f"   📊 샘플: {df.iloc[0].to_dict()}")
                print()
                
            except Exception as e:
                print(f"❌ {file_path.name}: 읽기 실패 - {str(e)}")

def main():
    """메인 실행 함수"""
    processor = RealDataProcessor()
    processor.process_all_data()
    
    print("\n🎯 다음 단계:")
    print("streamlit run main.py --server.port=8502")

if __name__ == "__main__":
    main()