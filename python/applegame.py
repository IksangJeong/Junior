import tkinter as tk
from tkinter import ttk
import keyboard  # 🎯 전역 핫키 감지를 위한 모듈 추가
import pandas as pd
from datetime import datetime
import pyautogui
import time
import os

NUM_ROWS = 10
NUM_COLS = 17
SCALE = 1
SIZE = 66 * SCALE

#
# 1) 숫자 인식
#
def detect_numbers(region):
    grid = [[0]*NUM_COLS for _ in range(NUM_ROWS)]
    total_sum = 0

    for digit in range(1, 10):
        for loc in pyautogui.locateAllOnScreen(
            f'images/apple{digit}.png', region=region, confidence=0.99
        ):
            r = (loc.top  - region[1]) // SIZE
            c = (loc.left - region[0]) // SIZE
            if 0 <= r < NUM_ROWS and 0 <= c < NUM_COLS:
                if grid[r][c] == 0:  # 중복 방지
                    grid[r][c] = digit
                    total_sum += digit
    return grid, total_sum

class BoxPy:
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

class StrategyPy:
    def __init__(self):
        self.boxes = []
        self.score = 0.0

def hash_grid(grid):
    h = 0
    for row in grid:
        for val in row:
            h = h*11 + val
            h &= 0xFFFFFFFFFFFFFFFF
    return h

def build_prefix_sum(grid):
    prefix = [[0]*(NUM_COLS+1) for _ in range(NUM_ROWS+1)]
    for r in range(NUM_ROWS):
        for c in range(NUM_COLS):
            prefix[r+1][c+1] = prefix[r+1][c] + prefix[r][c+1] - prefix[r][c] + grid[r][c]
    return prefix

def rect_sum(prefix, x, y, w, h):
    r1, c1 = y, x
    r2, c2 = y+h, x+w
    return prefix[r2][c2] - prefix[r2][c1] - prefix[r1][c2] + prefix[r1][c1]

def find_strategy_recursive(grid):
    visited = set()
    best = StrategyPy()

    max_moves = (NUM_ROWS*NUM_COLS)//2 + 1
    best_intermediate_scores = [float('-inf')] * max_moves

    def recurse(g, current, num_moves):
        nonlocal best
        if current.score > best.score:
            best.boxes = current.boxes[:]
            best.score = current.score

        if current.score > best_intermediate_scores[num_moves]:
            best_intermediate_scores[num_moves] = current.score

        if current.score + 5 < best_intermediate_scores[num_moves]:
            return

        hval = hash_grid(g)
        if hval in visited:
            return
        visited.add(hval)

        if len(visited) > 100_000:
            return

        D = 4
        candidates = []
        prefix = build_prefix_sum(g)

        for y in range(NUM_ROWS):
            for x in range(NUM_COLS):
                for h_ in range(1, NUM_ROWS - y + 1):
                    sub_sum = 0
                    for w_ in range(1, NUM_COLS - x + 1):
                        sub_sum = rect_sum(prefix, x, y, w_, h_)
                        if sub_sum == 10:
                            # 간단히 "sum_of_squares - area" 점수 예시
                            sum_of_squares = 0
                            for rr in range(y, y + h_):
                                for cc in range(x, x + w_):
                                    val = g[rr][cc]
                                    sum_of_squares += val*val
                            area = w_ * h_
                            rect_score = sum_of_squares - area
                            candidates.append((rect_score, x, y, w_, h_))
                        elif sub_sum > 10:
                            break
        candidates.sort(key=lambda x: x[0], reverse=True)
        moves = candidates[:D]

        for (rect_score, sx, sy, w_, h_) in moves:
            newg = [row[:] for row in g]
            for rr in range(sy, sy+h_):
                for cc in range(sx, sx+w_):
                    newg[rr][cc] = 0

            box_obj = BoxPy(sx, sy, w_, h_)
            current.boxes.append(box_obj)
            current.score += rect_score

            recurse(newg, current, num_moves+1)

            current.boxes.pop()
            current.score -= rect_score

    current = StrategyPy()
    recurse(grid, current, 0)
    return best

def find_strategy(grid):
    """외부에서 호출 => (x1,y1,x2,y2) 튜플 리스트 반환 + 정확한 점수"""

    best_obj = find_strategy_recursive(grid)
    moves = []

    # 🎯 (1) grid 복사본
    temp_grid = [row[:] for row in grid]

    total_removed_cells = 0

    for b in best_obj.boxes:
        x1 = b.x
        y1 = b.y
        x2 = b.x + b.w - 1
        y2 = b.y + b.h - 1

        moves.append((x1, y1, x2, y2))

        # (2) "0이 아닌 칸" 개수 세기 (중복 없는 최종 제거)
        removed_cells = sum(
            1 for rr in range(y1, y2 + 1)
              for cc in range(x1, x2 + 1)
              if temp_grid[rr][cc] > 0
        )
        total_removed_cells += removed_cells

        # (3) temp_grid에서도 실제로 0 처리
        for rr in range(y1, y2 + 1):
            for cc in range(x1, x2 + 1):
                temp_grid[rr][cc] = 0

    return moves, total_removed_cells




def debug_subgrid(grid, x1, y1, x2, y2):
    print("사각형 내부 숫자:")
    for rr in range(y1, y2+1):
        rowvals = grid[rr][x1:x2+1]
        print(" ".join(str(v) for v in rowvals))

def remove_subgrid(grid, x1, y1, x2, y2):
    for rr in range(y1, y2+1):
        for cc in range(x1, x2+1):
            grid[rr][cc] = 0


def log_to_excel(counts, total_sum, best_score, reason):
    file_path = "game_log.xlsx"  # 저장할 엑셀 파일 이름

    # 현재 시간 기록
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # 새로운 데이터 생성 (1~9 숫자별 개수, 사과합, 예상 점수, 이유 포함)
    new_data = [[now] + counts[1:] + [total_sum, best_score, reason]]

    # 컬럼명 설정
    column_names = ["Time"] + [str(i) for i in range(1, 10)] + ["Total Apples", "Expected Score", "Reason"]

    # 기존 파일이 있으면 불러오고, 없으면 새로 생성
    if os.path.exists(file_path):
        existing_df = pd.read_excel(file_path, engine="openpyxl")  # 기존 데이터 불러오기
        new_df = pd.DataFrame(new_data, columns=column_names)  # 새로운 데이터 생성
        combined_df = pd.concat([existing_df, new_df], ignore_index=True)  # 기존 데이터 + 새로운 데이터 합치기
    else:
        combined_df = pd.DataFrame(new_data, columns=column_names)  # 새로운 데이터 + 컬럼 추가

    # 엑셀 파일에 저장 (덮어쓰기 없이 기존 데이터 유지)
    with pd.ExcelWriter(file_path, mode="w", engine="openpyxl") as writer:
        combined_df.to_excel(writer, index=False)  # 인덱스 없이 저장


# ------------------- GUI 클래스 ------------------- #

class FruitBoxGUI:
    def __init__(self, root):
        self.root = root
        root.title("Fruit Box Bot")

        self.running = False  # 🎯 실행 여부 확인 플래그
        # 🎯 GUI 클래스 초기화 부분에 추가 (어느 창에서든 F9 동작)
        keyboard.add_hotkey("F9", lambda: self.on_f9())

    
        self.frm_main = ttk.Frame(root, padding=10)
        self.frm_main.pack(fill="both", expand=True)

        # 실행 버튼
        self.btn_run = ttk.Button(self.frm_main, text="실행", command=self.play_game)
        self.btn_run.grid(row=1, column=0, sticky="sw", padx=5, pady=5)


        # 중앙: 그리드 표시
        self.txt_grid = tk.Text(self.frm_main, width=40, height=15)
        self.txt_grid.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=5, pady=5)

        # 오른쪽: 숫자별 개수 / 예상 점수
        self.frm_info = ttk.Frame(self.frm_main)
        self.frm_info.grid(row=0, column=2, sticky="ns", padx=5, pady=5)

        self.lbl_counts_title = ttk.Label(self.frm_info, text="숫자 개수", font=("", 12, "bold"))
        self.lbl_counts_title.pack(anchor="nw")

        self.lbl_counts = ttk.Label(self.frm_info, text="(대기 중)", justify="left")
        self.lbl_counts.pack(anchor="nw", pady=5)

        self.lbl_score_title = ttk.Label(self.frm_info, text="예상 점수", font=("", 12, "bold"))
        self.lbl_score_title.pack(anchor="nw", pady=(10,0))

        self.lbl_score_val = ttk.Label(self.frm_info, text="0", foreground="blue")
        self.lbl_score_val.pack(anchor="nw")

        # 🎯 인식된 사과 합을 표시하는 Label 추가
        self.lbl_sum_title = ttk.Label(self.frm_info, text="사과 숫자 합", font=("", 12, "bold"))
        self.lbl_sum_title.pack(anchor="nw", pady=(10,0))

        self.lbl_sum_val = ttk.Label(self.frm_info, text="0")
        self.lbl_sum_val.pack(anchor="nw")


        # 레이아웃 확장
        self.frm_main.rowconfigure(0, weight=1)
        self.frm_main.columnconfigure(1, weight=1)

    def update_gui_info(self, grid, total_sum, expected_score):
        self.txt_grid.delete("1.0", tk.END)
        self.txt_grid.insert(tk.END, "[현재 GRID]\n")
        for row in grid:
            self.txt_grid.insert(tk.END, " ".join(str(x) for x in row) + "\n")

        # 🎯 인식된 사과 총합 표시
        self.lbl_sum_val.config(text=str(total_sum))

        # 🎯 예상 점수 표시
        self.lbl_score_val.config(text=str(expected_score))

    def on_f9(self):
        """ F9 키를 누르면 현재 실행 중인 작업을 취소하고 다시 시작 """
        print("[INFO] F9 입력 감지: 실행 중단 후 재시작")
        self.running = False  # 🎯 현재 실행 중단
        self.root.after(100, self.play_game)  # 🎯 100ms 후 다시 실행


    def play_game(self):
        if self.running:
            print("[INFO] 기존 실행 중단 후 재시작...")
            self.running = False  # 🎯 실행 중단
            return  # 🔄 중단하고 종료

        self.running = True  # 🎯 새로운 실행 시작
        
        
        """ 실행 버튼 눌렀을 때: 실제 게임 플레이 로직 + GUI 업데이트 """
        print("[INFO] 게임 화면 분석 중...")
        try:
            # 1) reset 버튼 찾아서 region
            left, top, w_, h_ = pyautogui.locateOnScreen('images/reset.png', confidence=0.99)
        except TypeError:
            # 못 찾으면 예외처리
            self.txt_grid.delete("1.0", tk.END)
            self.txt_grid.insert(tk.END, "[ERROR] reset.png 인식 실패\n")
            return

        left += 8*SCALE
        top  -= 740*SCALE
        region = (left, top, SIZE*NUM_COLS, SIZE*NUM_ROWS)

        # 2) reset / play (자동 클릭)
        pyautogui.leftClick(left/SCALE, top/SCALE)
        pyautogui.leftClick((left-3)/SCALE, (top+760)/SCALE)
        pyautogui.leftClick((left+300)/SCALE, (top+400)/SCALE)

        # 3) grid 인식
        grid, total_ = detect_numbers(region)
        
        # 숫자별 개수
        counts = [0]*10
        for r in range(NUM_ROWS):
            for c in range(NUM_COLS):
                val = grid[r][c]
                if 1 <= val <= 9:
                    counts[val] += 1
        
        # 🎯 150점 미만이면 즉시 F9 재실행 (Alt+F10 입력 X)
        if total_ > 800:
            print("[INFO] 사과가 " + str(total_) + "점. 즉시 재실행 (F9)")
            log_to_excel(counts, total_, -1, "Apples > 800")  # 🎯 기록 추가

            self.root.after(100, self.on_f9)  # 🎯 1초 후 F9 실행
            return  # 🚀 즉시 종료하여 드래그 작업 스킵

        # 중앙 Text 위젯에 표시
        self.txt_grid.delete("1.0", tk.END)
        self.txt_grid.insert(tk.END, "[인식된 GRID]\n")
        for row in grid:
            self.txt_grid.insert(tk.END, " ".join(str(x) for x in row) + "\n")



        count_str_list = [f"{digit}: {counts[digit]}개" for digit in range(1,10)]
        self.lbl_counts.config(text="\n".join(count_str_list))



        # 🎯 먼저 GUI 업데이트 실행 후, 드래그 시작
        self.update_gui_info(grid, total_, "계산중")
        self.root.update_idletasks()  # GUI 즉시 반영


        # 4) 최적 사각형 목록 구하기 + 점수
        moves, best_score = find_strategy(grid)
        print(f"[INFO] 최적 사각형 개수= {len(moves)}")

        # 🎯 150점 미만이면 즉시 F9 재실행 (Alt+F10 입력 X)
        if best_score < 160:
            print("[INFO] 예상 점수가 " + str(best_score) + "점이므로 즉시 재실행 (F9)")
            log_to_excel(counts, total_, best_score, "Score < 160")  # 🎯 기록 추가

            self.root.after(100, self.on_f9)  # 🎯 1초 후 F9 실행
            return  # 🚀 즉시 종료하여 드래그 작업 스킵
        
        # 점수 라벨에 표시
        self.lbl_score_val.config(text=str(round(best_score, 2)))

        # 🎯 먼저 GUI 업데이트 실행 후, 드래그 시작
        self.update_gui_info(grid, total_, best_score)
        self.root.update_idletasks()  # GUI 즉시 반영


        # 5) 실제로 드래그 동작
        for i, (x1, y1, x2, y2) in enumerate(moves, start=1):
            if not self.running:  # 🎯 F9가 눌리면 즉시 중단
                print("[INFO] 실행 중단됨.")
                return   
            print(f"\n[{i}/{len(moves)}] 사각형: (행 {y1}~{y2}, 열 {x1}~{x2})")
            # 디버그 출력
            print("사각형 내부 숫자:")
            for rr in range(y1, y2+1):
                print(" ".join(str(grid[rr][cc]) for cc in range(x1, x2+1)))

            # 픽셀 좌표
            start_x = left + x1*SIZE
            start_y = top  + y1*SIZE
            end_x   = left + (x2+1)*SIZE - 1
            end_y   = top  + (y2+1)*SIZE - 1
            print(f"🖱 드래그 시작=({start_x},{start_y}), 끝=({end_x},{end_y})")

            pyautogui.moveTo(start_x/SCALE, start_y/SCALE)
            pyautogui.mouseDown()
            time.sleep(0.05)
            pyautogui.moveTo(end_x/SCALE, end_y/SCALE, duration=0.05)
            pyautogui.moveTo(end_x/SCALE+1, end_y/SCALE+1, duration=0.02)
            pyautogui.mouseUp()

            # 실제 grid에서도 해당 범위 0 처리
            for rr in range(y1, y2+1):
                for cc in range(x1, x2+1):
                    grid[rr][cc] = 0


        time.sleep(120)
        if best_score >= 160:
            print("[INFO] 점수가 160점 이상이므로 Alt + F10 입력")
            pyautogui.hotkey('alt', 'f10')  # 🎯 Alt + F10 자동 입력
            log_to_excel(counts, total_, best_score, "Clear")  # 🎯 기록 추가


        # 7) 2분(120초) 대기 후 재실행 (F9 누른 것처럼)
        print("[INFO] 2분 후 자동 재실행")
        self.root.after(100, self.on_f9)  # 🎯 120초 후 F9 입력과 동일한 동작 실행


def main():
    root = tk.Tk()
    app = FruitBoxGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()