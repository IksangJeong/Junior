import pandas as pd
from pathlib import Path

def quick_fix():
    """빠른 파일 수정"""
    raw_dir = Path('assets/data/raw')
    csv_files = list(raw_dir.glob('*.csv'))
    
    print("🔧 파일 빠른 수정 시작")
    print("=" * 40)
    
    for file_path in csv_files:
        print(f"\n🔧 수정 중: {file_path.name}")
        
        # UTF-8로 읽기 시도
        try:
            df = pd.read_csv(file_path, encoding='utf-8')
            print(f"   ✅ UTF-8로 읽기 성공!")
            print(f"   📊 데이터 형태: {df.shape}")
            print(f"   📋 컬럼명: {list(df.columns)[:3]}...")
            
            # CP949로 다시 저장
            df.to_csv(file_path, index=False, encoding='cp949')
            print(f"   ✅ CP949로 저장 완료")
            
        except Exception as e:
            print(f"   ❌ UTF-8 실패: {str(e)}")
            
            # UTF-8-sig로 시도
            try:
                df = pd.read_csv(file_path, encoding='utf-8-sig')
                print(f"   ✅ UTF-8-sig로 읽기 성공!")
                print(f"   📊 데이터 형태: {df.shape}")
                
                # CP949로 다시 저장
                df.to_csv(file_path, index=False, encoding='cp949')
                print(f"   ✅ CP949로 저장 완료")
                
            except Exception as e2:
                print(f"   ❌ UTF-8-sig도 실패: {str(e2)}")
    
    print("\n🎉 수정 완료!")

if __name__ == "__main__":
    quick_fix()