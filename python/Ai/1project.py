# #필기체 숫자 분류
# import matplotlib.pyplot as plt
# from sklearn import datasets, metrics
# from sklearn.model_selection import train_test_split

# digits = datasets.load_digits()
# plt.imshow(digits.images[0], cmap=plt.cm.gray_r, interpolation='nearest')

# #이미지 평탄화
# n_samples = len(digits.images)
# data = digits.images.reshape((n_samples, -1))

# #학습 데이터와 테스트 데이터 분리
# X_train, X_test, y_train, y_test = train_test_split(data, digits.target, test_size=0.2)

# #모델 학습
# from sklearn.neighbors import KNeighborsClassifier
# knn = KNeighborsClassifier(n_neighbors=6)

# knn.fit(X_train, y_train)

# #모델 예측
# y_pred = knn.predict(X_test) 

# scores = metrics.accuracy_score(y_test, y_pred)
# print(scores)

# plt.imshow(X_test[10].reshape(8,8), cmap=plt.cm.gray_r, interpolation='nearest')

# y_pred = knn.predict([X_test[10]])
# print(y_pred)
import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from sklearn import datasets, metrics
from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.svm import SVC
import seaborn as sns
import pandas as pd
from PIL import Image, ImageDraw, ImageOps

class DigitRecognitionApp:
    def __init__(self, root):
        self.root = root
        self.root.title("손글씨 숫자 인식 애플리케이션")
        self.root.geometry("1200x800")
        
        # 데이터 로드 및 모델 학습
        self.load_data()
        self.train_model()
        
        # 메인 프레임 생성
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 왼쪽 프레임 (그리기 영역)
        self.left_frame = ttk.Frame(self.main_frame)
        self.left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 오른쪽 프레임 (결과 표시 영역)
        self.right_frame = ttk.Frame(self.main_frame)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # 그리기 영역 설정
        self.setup_drawing_area()
        
        # 결과 표시 영역 설정
        self.setup_result_area()
        
        # 컨트롤 패널 설정
        self.setup_control_panel()
    
    def load_data(self):
        # MNIST 데이터셋 로드
        self.digits = datasets.load_digits()
        n_samples = len(self.digits.images)
        self.data = self.digits.images.reshape((n_samples, -1))
        
        # 학습 데이터와 테스트 데이터 분리
        self.X_train, self.X_test, self.y_train, self.y_test = train_test_split(
            self.data, self.digits.target, test_size=0.2, random_state=42
        )
    
    def train_model(self):
        # 모델 선택에 따라 학습 (기본값은 KNN)
        self.model_type = tk.StringVar(value="KNN")
        self.n_neighbors = tk.IntVar(value=5)
        self.hidden_layer_sizes = tk.StringVar(value="100,100")
        
        # 진행 상황 라벨
        self.progress_var = tk.StringVar(value="")
        
        # 모델 학습 시작
        self.progress_var.set("KNN 모델 학습 중...")
        
        # KNN 모델 학습
        self.knn = KNeighborsClassifier(n_neighbors=self.n_neighbors.get())
        self.knn.fit(self.X_train, self.y_train)
        
        self.progress_var.set("MLP 모델 학습 중...")
        # MLP 모델 학습
        hidden_layers = tuple(map(int, self.hidden_layer_sizes.get().split(',')))
        self.mlp = MLPClassifier(hidden_layer_sizes=hidden_layers, max_iter=1000, random_state=42)
        self.mlp.fit(self.X_train, self.y_train)
        
        self.progress_var.set("SVM 모델 학습 중...")
        # SVM 모델 학습
        self.svm = SVC(gamma='scale', probability=True, random_state=42)
        self.svm.fit(self.X_train, self.y_train)
        
        self.progress_var.set("모델 학습 완료!")
        
        # 현재 선택된 모델 설정
        self.current_model = self.knn
    
    def setup_drawing_area(self):
        # 그리기 영역 라벨
        ttk.Label(self.left_frame, text="여기에 숫자를 꽉 차게 그려주세요:", font=("Arial", 14)).pack(pady=10)
        
        # 캔버스 설정
        self.canvas_frame = ttk.Frame(self.left_frame, borderwidth=2, relief="groove")
        self.canvas_frame.pack(pady=10)
        
        self.canvas_size = 280
        self.canvas = tk.Canvas(self.canvas_frame, width=self.canvas_size, height=self.canvas_size, bg="black")
        self.canvas.pack()
        
        # 그리기 이벤트 바인딩
        self.drawing = False
        self.last_x = None
        self.last_y = None
        
        # 이미지 객체 생성 (캔버스에 그린 내용을 저장)
        self.image = Image.new("RGB", (self.canvas_size, self.canvas_size), color="black")
        self.draw = ImageDraw.Draw(self.image)
        
        # 펜 두께 설정
        self.pen_width = tk.IntVar(value=15)
        self.canvas.bind("<Button-1>", self.start_drawing)
        self.canvas.bind("<B1-Motion>", self.draw_on_canvas)
        self.canvas.bind("<ButtonRelease-1>", self.stop_drawing)
        
        # 그리드 라인 토글 변수
        self.show_grid = tk.BooleanVar(value=True)
        
        # 버튼 프레임
        self.canvas_button_frame = ttk.Frame(self.left_frame)
        self.canvas_button_frame.pack(pady=10)
        
        # 지우기 버튼
        ttk.Button(self.canvas_button_frame, text="지우기", command=self.clear_canvas).pack(side=tk.LEFT, padx=5)
        
        # 인식 버튼
        ttk.Button(self.canvas_button_frame, text="인식하기", command=self.recognize_digit).pack(side=tk.LEFT, padx=5)
        
        # 펜 두께 슬라이더
        ttk.Label(self.canvas_button_frame, text="펜 두께:").pack(side=tk.LEFT, padx=(20, 5))
        ttk.Scale(
            self.canvas_button_frame,
            from_=1, to=30,
            variable=self.pen_width,
            orient=tk.HORIZONTAL,
            length=100
        ).pack(side=tk.LEFT, padx=5)
        
        # 그리드 표시 체크박스
        ttk.Checkbutton(
            self.canvas_button_frame,
            text="그리드 표시",
            variable=self.show_grid,
            command=self.toggle_grid
        ).pack(side=tk.LEFT, padx=20)
        
        # 8x8 그리드 라인 그리기
        self.grid_lines = []
        self.draw_grid_lines()
    
    def draw_grid_lines(self):
        """캔버스에 8x8 그리드 라인 그리기"""
        # 이전 그리드 라인 삭제
        for line in self.grid_lines:
            self.canvas.delete(line)
        
        self.grid_lines = []
        
        # 그리드 표시가 활성화된 경우에만 그리드 그리기
        if self.show_grid.get():
            # 셀 크기 계산
            cell_size = self.canvas_size / 8
            
            # 수직선 그리기
            for i in range(1, 8):
                x = i * cell_size
                line = self.canvas.create_line(
                    x, 0, x, self.canvas_size, 
                    fill="gray", dash=(2, 4)
                )
                self.grid_lines.append(line)
            
            # 수평선 그리기
            for i in range(1, 8):
                y = i * cell_size
                line = self.canvas.create_line(
                    0, y, self.canvas_size, y, 
                    fill="gray", dash=(2, 4)
                )
                self.grid_lines.append(line)
    
    def toggle_grid(self):
        """그리드 표시 토글"""
        if self.show_grid.get():
            self.draw_grid_lines()
        else:
            for line in self.grid_lines:
                self.canvas.delete(line)
            self.grid_lines = []
    
    def start_drawing(self, event):
        self.drawing = True
        self.last_x = event.x
        self.last_y = event.y
    
    def draw_on_canvas(self, event):
        if self.drawing:
            x, y = event.x, event.y
            if self.last_x and self.last_y:
                # 캔버스에 선 그리기
                self.canvas.create_line(
                    self.last_x, self.last_y, x, y,
                    fill="white", width=self.pen_width.get(), 
                    capstyle=tk.ROUND, smooth=True
                )
                
                # 이미지 객체에도 동일한 선 그리기
                self.draw.line(
                    [self.last_x, self.last_y, x, y],
                    fill="white", width=self.pen_width.get()
                )
            
            self.last_x = x
            self.last_y = y
    
    def stop_drawing(self, event):
        self.drawing = False
        self.last_x = None
        self.last_y = None
    
    def clear_canvas(self):
        # 캔버스 초기화
        self.canvas.delete("all")
        
        # 이미지 객체 초기화
        self.image = Image.new("RGB", (self.canvas_size, self.canvas_size), color="black")
        self.draw = ImageDraw.Draw(self.image)
        
        # 그리드 라인 다시 그리기
        self.draw_grid_lines()
        
        # 결과 표시 초기화
        self.prediction_label.config(text="?")
        self.ax_digit.clear()
        self.ax_digit.set_xticks([])
        self.ax_digit.set_yticks([])
        self.digit_canvas.draw()
        
        self.ax_proba.clear()
        self.ax_proba.set_xticks([])
        self.ax_proba.set_yticks([])
        self.proba_canvas.draw()
        
        # 8x8 그리드 표시 초기화
        self.grid_ax.clear()
        self.grid_ax.set_xticks([])
        self.grid_ax.set_yticks([])
        self.grid_canvas.draw()
        
        self.progress_var.set("캔버스가 초기화되었습니다.")
    
    def recognize_digit(self):
        """그린 숫자를 인식합니다."""
        self.progress_var.set("숫자 인식 중...")
        
        # 이미지 전처리
        img = self.image.copy()
        
        # 그레이스케일로 변환
        img = img.convert("L")
        
        # 이진화 (흰색=글씨, 검은색=배경)
        img = img.point(lambda p: p > 10 and 255)
        
        # 이미지 자르기 (주요 콘텐츠 영역만)
        bbox = ImageOps.invert(img).getbbox()
        if bbox:
            # 여백 추가 (20% 정도)
            width, height = bbox[2] - bbox[0], bbox[3] - bbox[1]
            padx, pady = int(width * 0.2), int(height * 0.2)
            
            # 여백이 있는 새 바운딩 박스
            new_bbox = (max(0, bbox[0] - padx),
                      max(0, bbox[1] - pady),
                      min(img.width, bbox[2] + padx),
                      min(img.height, bbox[3] + pady))
            
            # 숫자 부분만 크롭
            img = img.crop(new_bbox)
            
            # 정사각형으로 만들기 위한 패딩 추가
            width, height = img.size
            if width > height:
                padding = (width - height) // 2
                img = ImageOps.expand(img, (0, padding, 0, padding), fill=0)
            else:
                padding = (height - width) // 2
                img = ImageOps.expand(img, (padding, 0, padding, 0), fill=0)
        
        # 원본 크기 저장 (시각화용)
        self.processed_img = img
        
        # 8x8로 리사이즈 (MNIST 입력 형식)
        img_8x8 = img.resize((8, 8), Image.LANCZOS)
        
        # 값 범위 변환 (0-255 -> 0-16)
        img_array = np.array(img_8x8) / 255.0 * 16
        
        # 8x8 이미지 시각화
        self.grid_ax.clear()
        self.grid_ax.imshow(img_array, cmap='gray_r')
        self.grid_ax.set_title('8x8 변환 이미지')
        self.grid_ax.set_xticks(np.arange(8))
        self.grid_ax.set_yticks(np.arange(8))
        self.grid_ax.grid(True, color='gray', linestyle='-', linewidth=0.5)
        self.grid_canvas.draw()
        
        # 원본 이미지 시각화
        self.ax_digit.clear()
        self.ax_digit.imshow(np.array(self.processed_img), cmap='gray_r')
        self.ax_digit.set_title('입력 숫자')
        self.ax_digit.set_xticks([])
        self.ax_digit.set_yticks([])
        self.digit_canvas.draw()
        
        # 모델 입력 형식으로 변환
        img_vector = img_array.reshape(1, -1)
        
        # 예측
        prediction = self.current_model.predict(img_vector)[0]
        
        # 확률 계산
        if hasattr(self.current_model, "predict_proba"):
            probabilities = self.current_model.predict_proba(img_vector)[0]
        else:
            # SVM의 경우 decision_function을 사용
            decision_values = self.current_model.decision_function(img_vector)[0]
            probabilities = np.exp(decision_values) / np.sum(np.exp(decision_values))
        
        # 결과 표시
        self.prediction_label.config(text=str(prediction))
        
        # 확률 그래프 표시
        self.ax_proba.clear()
        bars = self.ax_proba.bar(range(10), probabilities)
        
        # 최대값 강조 표시
        max_idx = np.argmax(probabilities)
        bars[max_idx].set_color('red')
        
        self.ax_proba.set_xticks(range(10))
        self.ax_proba.set_xlabel('숫자')
        self.ax_proba.set_ylabel('확률')
        self.ax_proba.set_title('예측 확률')
        
        # 확률값 표시
        for i, p in enumerate(probabilities):
            if p > 0.05:  # 확률이 5% 이상인 경우만 레이블 표시
                self.ax_proba.text(i, p + 0.02, f'{p:.2f}', ha='center')
                
        self.proba_canvas.draw()
        
        self.progress_var.set(f"인식 완료! 예측: {prediction} (확률: {probabilities[prediction]:.2f})")
    
    def setup_result_area(self):
        # 결과 표시 영역 라벨
        ttk.Label(self.right_frame, text="인식 결과:", font=("Arial", 14)).pack(pady=10)
        
        # 결과 표시 프레임
        self.result_frame = ttk.Frame(self.right_frame)
        self.result_frame.pack(pady=10)
        
        # 숫자 이미지 표시
        self.fig_digit = Figure(figsize=(4, 4))
        self.ax_digit = self.fig_digit.add_subplot(111)
        self.digit_canvas = FigureCanvasTkAgg(self.fig_digit, self.result_frame)
        self.digit_canvas.get_tk_widget().pack(side=tk.LEFT, padx=10)
        
        # 예측 결과 표시
        self.prediction_label = ttk.Label(self.result_frame, text="?", font=("Arial", 48))
        self.prediction_label.pack(side=tk.LEFT, padx=20)
        
        # 8x8 그리드 표시 프레임
        self.grid_frame = ttk.Frame(self.right_frame)
        self.grid_frame.pack(pady=10)
        
        ttk.Label(self.grid_frame, text="8x8 변환 결과:").pack(anchor=tk.W)
        
        # 8x8 그리드 시각화
        self.fig_grid = Figure(figsize=(4, 4))
        self.grid_ax = self.fig_grid.add_subplot(111)
        self.grid_canvas = FigureCanvasTkAgg(self.fig_grid, self.grid_frame)
        self.grid_canvas.get_tk_widget().pack(pady=5)
        
        # 그리드 초기화
        self.grid_ax.set_xticks([])
        self.grid_ax.set_yticks([])
        
        # 확률 그래프 표시
        self.fig_proba = Figure(figsize=(6, 3))
        self.ax_proba = self.fig_proba.add_subplot(111)
        self.proba_canvas = FigureCanvasTkAgg(self.fig_proba, self.right_frame)
        self.proba_canvas.get_tk_widget().pack(pady=10)
        
        # 혼동 행렬 표시
        ttk.Label(self.right_frame, text="모델 성능 (혼동 행렬):", font=("Arial", 12)).pack(pady=5)
        self.fig_cm = Figure(figsize=(5, 4))
        self.ax_cm = self.fig_cm.add_subplot(111)
        self.cm_canvas = FigureCanvasTkAgg(self.fig_cm, self.right_frame)
        self.cm_canvas.get_tk_widget().pack(pady=10)
        
        # 초기 혼동 행렬 표시
        self.plot_confusion_matrix()
        
        # 상태 메시지 라벨
        self.status_label = ttk.Label(self.right_frame, textvariable=self.progress_var)
        self.status_label.pack(pady=5)
    
    def setup_control_panel(self):
        # 컨트롤 패널 프레임
        self.control_frame = ttk.LabelFrame(self.left_frame, text="모델 설정")
        self.control_frame.pack(fill=tk.X, pady=10)
        
        # 모델 선택
        ttk.Label(self.control_frame, text="모델 선택:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        model_combo = ttk.Combobox(self.control_frame, textvariable=self.model_type, 
                                  values=["KNN", "MLP", "SVM"], state="readonly")
        model_combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        model_combo.bind("<<ComboboxSelected>>", self.update_model)
        
        # KNN 설정
        ttk.Label(self.control_frame, text="KNN 이웃 수:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        knn_spinbox = ttk.Spinbox(self.control_frame, from_=1, to=20, textvariable=self.n_neighbors, width=5)
        knn_spinbox.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        
        # MLP 설정
        ttk.Label(self.control_frame, text="MLP 은닉층 (예: 100,100):").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        ttk.Entry(self.control_frame, textvariable=self.hidden_layer_sizes, width=15).grid(row=2, column=1, padx=5, pady=5, sticky="w")
        
        # 모델 재학습 버튼
        ttk.Button(self.control_frame, text="모델 재학습", command=self.retrain_model).grid(row=3, column=0, columnspan=2, padx=5, pady=5)
        
        # 정확도 표시
        self.accuracy_var = tk.StringVar(value="정확도: ")
        ttk.Label(self.control_frame, textvariable=self.accuracy_var).grid(row=4, column=0, columnspan=2, padx=5, pady=5)
        
        # 초기 정확도 계산
        self.calculate_accuracy()
        
        # 샘플 숫자 표시 버튼
        ttk.Button(self.control_frame, text="샘플 숫자 보기", command=self.show_sample_digits).grid(row=5, column=0, columnspan=2, padx=5, pady=5)
    
    def update_model(self, event=None):
        model_type = self.model_type.get()
        if model_type == "KNN":
            self.current_model = self.knn
        elif model_type == "MLP":
            self.current_model = self.mlp
        else:  # SVM
            self.current_model = self.svm
        
        self.calculate_accuracy()
        self.plot_confusion_matrix()
    
    def retrain_model(self):
        model_type = self.model_type.get()
        self.progress_var.set(f"{model_type} 모델 재학습 중...")
        
        if model_type == "KNN":
            self.knn = KNeighborsClassifier(n_neighbors=self.n_neighbors.get())
            self.knn.fit(self.X_train, self.y_train)
            self.current_model = self.knn
        
        elif model_type == "MLP":
            try:
                hidden_layers = tuple(map(int, self.hidden_layer_sizes.get().split(',')))
                self.mlp = MLPClassifier(hidden_layer_sizes=hidden_layers, max_iter=1000, random_state=42)
                self.mlp.fit(self.X_train, self.y_train)
                self.current_model = self.mlp
            except ValueError:
                tk.messagebox.showerror("오류", "은닉층 형식이 잘못되었습니다. 예: 100,100")
                return
        
        else:  # SVM
            self.svm = SVC(gamma='scale', probability=True, random_state=42)
            self.svm.fit(self.X_train, self.y_train)
            self.current_model = self.svm
        
        self.calculate_accuracy()
        self.plot_confusion_matrix()
        self.progress_var.set(f"{model_type} 모델 재학습 완료!")
    
    def calculate_accuracy(self):
        y_pred = self.current_model.predict(self.X_test)
        accuracy = metrics.accuracy_score(self.y_test, y_pred)
        self.accuracy_var.set(f"정확도: {accuracy:.4f}")
    
    def plot_confusion_matrix(self):
        y_pred = self.current_model.predict(self.X_test)
        cm = metrics.confusion_matrix(self.y_test, y_pred)
        
        self.ax_cm.clear()
        sns.heatmap(cm, annot=True, fmt="d", cmap="Blues", ax=self.ax_cm)
        self.ax_cm.set_xlabel('예측 값')
        self.ax_cm.set_ylabel('실제 값')
        self.ax_cm.set_title('혼동 행렬')
        self.cm_canvas.draw()
    
    def show_sample_digits(self):
        """샘플 숫자를 팝업 창에 표시합니다."""
        # 새 창 생성
        sample_window = tk.Toplevel(self.root)
        sample_window.title("MNIST 샘플 숫자")
        sample_window.geometry("800x600")
        
        # Matplotlib Figure 생성
        fig = Figure(figsize=(10, 8))
        
        # 5x10 그리드에 샘플 이미지와 실제 값 표시
        for i in range(50):
            ax = fig.add_subplot(5, 10, i+1)
            ax.imshow(self.digits.images[i], cmap='gray_r')
            ax.set_title(f"{self.digits.target[i]}")
            ax.set_xticks([])
            ax.set_yticks([])
        
        fig.tight_layout()
        
        # 캔버스에 그림 표시
        canvas = FigureCanvasTkAgg(fig, sample_window)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 샘플 가져오기 버튼 추가
        sample_frame = ttk.Frame(sample_window, padding=10)
        sample_frame.pack(fill=tk.X)
        
        # 선택된 인덱스 변수
        self.selected_sample_idx = tk.IntVar(value=0)
        
        # 샘플 선택 슬라이더
        ttk.Label(sample_frame, text="샘플 인덱스:").pack(side=tk.LEFT)
        sample_slider = ttk.Scale(
            sample_frame, 
            from_=0, to=len(self.digits.images)-1, 
            variable=self.selected_sample_idx,
            orient=tk.HORIZONTAL,
            length=300
        )
        sample_slider.pack(side=tk.LEFT, padx=5)
        
        # 선택된 인덱스 표시 라벨
        sample_idx_label = ttk.Label(sample_frame, textvariable=self.selected_sample_idx)
        sample_idx_label.pack(side=tk.LEFT, padx=5)
        
        # 샘플 가져오기 버튼
        ttk.Button(
            sample_frame, 
            text="이 샘플로 테스트", 
            command=lambda: self.load_sample_digit(self.selected_sample_idx.get())
        ).pack(side=tk.LEFT, padx=20)
    
    def load_sample_digit(self, idx):
        """샘플 숫자를 테스트합니다."""
        # 샘플 이미지 가져오기
        sample_img = self.digits.images[idx].copy()
        
        # 8x8 이미지 시각화
        self.grid_ax.clear()
        self.grid_ax.imshow(sample_img, cmap='gray_r')
        self.grid_ax.set_title('8x8 샘플 이미지')
        self.grid_ax.set_xticks(np.arange(8))
        self.grid_ax.set_yticks(np.arange(8))
        self.grid_ax.grid(True, color='gray', linestyle='-', linewidth=0.5)
        self.grid_canvas.draw()
        
        # 원본 이미지 시각화
        self.ax_digit.clear()
        self.ax_digit.imshow(sample_img, cmap='gray_r')
        self.ax_digit.set_title(f'샘플 숫자 {self.digits.target[idx]}')
        self.ax_digit.set_xticks([])
        self.ax_digit.set_yticks([])
        self.digit_canvas.draw()
        
        # 모델 입력 형식으로 변환
        img_vector = sample_img.reshape(1, -1)
        
        # 예측
        prediction = self.current_model.predict(img_vector)[0]
        
        # 확률 계산
        if hasattr(self.current_model, "predict_proba"):
            probabilities = self.current_model.predict_proba(img_vector)[0]
        else:
            # SVM의 경우 decision_function을 사용
            decision_values = self.current_model.decision_function(img_vector)[0]
            probabilities = np.exp(decision_values) / np.sum(np.exp(decision_values))
        
        # 결과 표시
        self.prediction_label.config(text=str(prediction))
        
        # 확률 그래프 표시
        self.ax_proba.clear()
        bars = self.ax_proba.bar(range(10), probabilities)
        
        # 최대값 강조 표시
        max_idx = np.argmax(probabilities)
        bars[max_idx].set_color('red')
        
        self.ax_proba.set_xticks(range(10))
        self.ax_proba.set_xlabel('숫자')
        self.ax_proba.set_ylabel('확률')
        self.ax_proba.set_title('예측 확률')
        
        # 확률값 표시
        for i, p in enumerate(probabilities):
            if p > 0.05:  # 확률이 5% 이상인 경우만 레이블 표시
                self.ax_proba.text(i, p + 0.02, f'{p:.2f}', ha='center')
                
        self.proba_canvas.draw()
        
        self.progress_var.set(f"샘플 숫자 {self.digits.target[idx]} 테스트 완료! 예측: {prediction} (확률: {probabilities[prediction]:.2f})")

# 애플리케이션 실행
if __name__ == "__main__":
    # GUI 스타일 설정
    try:
        from ttkthemes import ThemedTk
        root = ThemedTk(theme="arc")  # 더 현대적인 테마 적용
    except ImportError:
        root = tk.Tk()
        print("참고: ttkthemes 라이브러리를 설치하면 더 좋은 UI를 사용할 수 있습니다.")
    
    app = DigitRecognitionApp(root)
    
    # 시작 메시지
    app.progress_var.set("준비 완료! 8x8 그리드에 숫자를 그리고 인식 버튼을 눌러주세요.")
    
    root.mainloop()