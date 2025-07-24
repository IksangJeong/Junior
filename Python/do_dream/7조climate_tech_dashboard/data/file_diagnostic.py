import pandas as pd
from pathlib import Path
import chardet
import os

def diagnose_files():
    """파일 진단 도구"""
    print("🔍 파일 진단 시작")
    print("=" * 50)
    
    raw_dir = Path('assets/data/raw')
    
    if not raw_dir.exists():
        print("❌ assets/data/raw 폴더가 없습니다.")
        return
    
    csv_files = list(raw_dir.glob('*.csv'))
    
    if not csv_files:
        print("❌ CSV 파일이 없습니다.")
        return
    
    print(f"📁 {len(csv_files)}개 파일 발견")
    print()
    
    for file_path in csv_files:
        print(f"🔍 진단 중: {file_path.name}")
        print("-" * 30)
        
        # 1. 파일 크기 확인
        file_size = file_path.stat().st_size
        print(f"📊 파일 크기: {file_size:,} bytes")
        
        # 2. 인코딩 감지
        try:
            with open(file_path, 'rb') as f:
                raw_data = f.read(10000)  # 처음 10KB만 읽기
                detected = chardet.detect(raw_data)
                print(f"🔤 감지된 인코딩: {detected}")
        except Exception as e:
            print(f"❌ 인코딩 감지 실패: {str(e)}")
        
        # 3. 파일 내용 미리보기 (다양한 인코딩으로 시도)
        encodings = ['cp949', 'euc-kr', 'utf-8-sig', 'utf-8', 'latin-1']
        
        for encoding in encodings:
            try:
                print(f"🔄 {encoding}로 시도 중...")
                
                # 텍스트로 첫 3줄 읽기
                with open(file_path, 'r', encoding=encoding) as f:
                    lines = []
                    for i, line in enumerate(f):
                        if i < 3:
                            lines.append(line.strip())
                        else:
                            break
                
                print(f"✅ {encoding} 성공!")
                print("📋 첫 3줄 미리보기:")
                for i, line in enumerate(lines, 1):
                    print(f"   {i}: {line[:100]}...")
                
                # pandas로 읽기 시도
                df = pd.read_csv(file_path, encoding=encoding, nrows=2)
                print(f"📊 pandas 읽기 성공 - 모양: {df.shape}")
                print(f"📋 컬럼명: {list(df.columns)}")
                
                break
                
            except UnicodeDecodeError:
                print(f"❌ {encoding}: 인코딩 오류")
                continue
            except Exception as e:
                print(f"❌ {encoding}: {str(e)}")
                continue
        else:
            print("❌ 모든 인코딩 시도 실패")
        
        print()

def fix_files():
    """파일 수정 도구"""
    print("🔧 파일 수정 시작")
    print("=" * 50)
    
    raw_dir = Path('assets/data/raw')
    csv_files = list(raw_dir.glob('*.csv'))
    
    for file_path in csv_files:
        print(f"🔧 수정 중: {file_path.name}")
        
        # 성공한 인코딩으로 읽고 UTF-8로 다시 저장
        encodings = ['cp949', 'euc-kr', 'utf-8-sig', 'utf-8', 'latin-1']
        
        for encoding in encodings:
            try:
                # 파일 읽기
                df = pd.read_csv(file_path, encoding=encoding)
                
                # 백업 파일 생성
                backup_path = file_path.with_suffix('.backup.csv')
                file_path.rename(backup_path)
                
                # UTF-8로 다시 저장
                df.to_csv(file_path, index=False, encoding='utf-8-sig')
                
                print(f"✅ {file_path.name} 수정 완료 ({encoding} → utf-8-sig)")
                break
                
            except Exception as e:
                continue
        else:
            print(f"❌ {file_path.name} 수정 실패")

def test_pandas_read():
    """pandas 읽기 테스트"""
    print("🧪 pandas 읽기 테스트")
    print("=" * 50)
    
    raw_dir = Path('assets/data/raw')
    csv_files = list(raw_dir.glob('*.csv'))
    
    for file_path in csv_files:
        print(f"🧪 테스트 중: {file_path.name}")
        
        # 다양한 옵션으로 시도
        options = [
            {'encoding': 'cp949'},
            {'encoding': 'euc-kr'},
            {'encoding': 'utf-8-sig'},
            {'encoding': 'utf-8'},
            {'encoding': 'cp949', 'sep': ','},
            {'encoding': 'cp949', 'sep': '\t'},
            {'encoding': 'cp949', 'sep': ';'},
            {'encoding': 'cp949', 'delimiter': ','},
            {'encoding': 'cp949', 'skipinitialspace': True},
            {'encoding': 'cp949', 'header': 0},
            {'encoding': 'cp949', 'header': None},
        ]
        
        for i, option in enumerate(options):
            try:
                df = pd.read_csv(file_path, **option)
                print(f"   ✅ 옵션 {i+1} 성공: {option}")
                print(f"      📊 모양: {df.shape}")
                print(f"      📋 컬럼: {list(df.columns)[:3]}...")
                if len(df) > 0:
                    print(f"      📋 첫 행: {df.iloc[0, 0] if len(df.columns) > 0 else 'N/A'}")
                break
            except Exception as e:
                continue
        else:
            print(f"   ❌ 모든 옵션 실패")
        
        print()

if __name__ == "__main__":
    print("🔧 파일 진단 도구")
    print("1. 파일 진단")
    print("2. 파일 수정")
    print("3. pandas 읽기 테스트")
    print()
    
    choice = input("선택하세요 (1/2/3): ").strip()
    
    if choice == '1':
        diagnose_files()
    elif choice == '2':
        fix_files()
    elif choice == '3':
        test_pandas_read()
    else:
        print("모든 진단 실행:")
        diagnose_files()
        print("\n" + "=" * 50 + "\n")
        test_pandas_read()