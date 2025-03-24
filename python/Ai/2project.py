# 라이브러리들 가져오기
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.font_manager as fm

# 한글 폰트 설정 - 깨짐 방지
# matplotlib 한글 폰트 설정
plt.rcParams['font.family'] = 'Apple SD Gothic Neo'  # iOS 기본 한글 폰트
plt.rcParams['axes.unicode_minus'] = False  # 마이너스 기호 깨짐 방지
from sklearn.datasets import load_iris
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler  # 결국 안썼네 씨발
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
from sklearn.linear_model import LogisticRegression
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC
from sklearn.tree import DecisionTreeClassifier
from sklearn.ensemble import RandomForestClassifier

# 데이터 가져오기... 개좆같이 항상 똑같은 붓꽃 데이터
iris = load_iris()
X = iris.data  # 특성들
y = iris.target  # 레이블(꽃 종류)
feature_names = iris.feature_names
target_names = iris.target_names

# 판다스로 변환 (나중에 시각화할 때 편함)
df = pd.DataFrame(X, columns=feature_names)
df['species'] = pd.Categorical.from_codes(y, target_names)

# 데이터 기본정보 확인
print("데이터 정보:")
print(f"샘플 수: {X.shape[0]}")
print(f"특성 수: {X.shape[1]}")
print(f"클래스: {target_names}")
print("\n처음 5개:")
print(df.head())
print("\n통계:")
print(df.describe())

# 시각화 - 이거 안하면 교수님이 빡치심
plt.figure(figsize=(15, 10))

# 산점도 행렬 - 특성간 관계 볼 수 있음
plt.subplot(2, 2, 1)  # 이 위치 잡는거 존나 헷갈리네
sns.pairplot(df, hue='species')
plt.title('특성 산점도 행렬')

# 히스토그램 - 각 특성별로 분포 확인
plt.figure(figsize=(12, 8))
for i in range(4):  # 4개 특성
    plt.subplot(2, 2, i+1)  # 2x2 그리드
    for species in range(3):  # 3개 종
        plt.hist(X[y == species, i], bins=10, alpha=0.5, label=target_names[species])
    plt.xlabel(feature_names[i])
    plt.ylabel('빈도')
    plt.legend()
plt.tight_layout()  # 겹치지 말라고
plt.suptitle('특성별 히스토그램', y=1.02, fontsize=15)  # 제목 씨발 왜 y=1.02지?

# 상관관계 히트맵 - 특성간 상관계수
plt.figure(figsize=(10, 8))
correlation = df.iloc[:, :-1].corr()  # 마지막 species 열 빼고
sns.heatmap(correlation, annot=True, cmap='coolwarm')  # annot=True는 숫자표시
plt.title('특성 상관관계')

# 데이터 나누기 (학습용/테스트용)
# stratify=y: 클래스 비율 유지, 안하면 classes 불균형 생길수도
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42, stratify=y)
print(f"\n학습 데이터: {X_train.shape}")
print(f"테스트 데이터: {X_test.shape}")

# 모델들 정의 - 여러개 해서 제일 좋은거 쓸거임
# 교수님 말로는 SVM이 제일 좋다는데 확인해보자
models = {
    '로지스틱 회귀': LogisticRegression(max_iter=200, random_state=42),  # 기본 100인데 수렴 안돼서 씨발 200으로
    'KNN': KNeighborsClassifier(n_neighbors=5),  # 이웃 5개 기본값임
    'SVM': SVC(kernel='rbf', random_state=42),  # rbf 커널이 대체로 좋음
    '의사결정트리': DecisionTreeClassifier(random_state=42),  # 과적합 잘되는데 일단 해봄
    '랜덤포레스트': RandomForestClassifier(n_estimators=100, random_state=42)  # 나무 100개 세워서 앙상블
}

# 모델 학습 & 평가
results = {}
for name, model in models.items():
    # 학습
    model.fit(X_train, y_train)
    
    # 예측
    y_pred = model.predict(X_test)
    
    # 평가지표
    accuracy = accuracy_score(y_test, y_pred)
    results[name] = accuracy
    print(f"\n{name} 결과:")
    print(f"정확도: {accuracy:.4f}")
    print(f"상세 지표:")
    print(classification_report(y_test, y_pred, target_names=target_names))
    
    # 혼동행렬 - 어떤 클래스를 어떤 클래스로 잘못 예측했는지
    plt.figure(figsize=(8, 6))
    cm = confusion_matrix(y_test, y_pred)
    sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', 
                xticklabels=target_names, yticklabels=target_names)
    plt.title(f'{name} 혼동행렬')
    plt.xlabel('예측')
    plt.ylabel('실제')

# 모델 비교 그래프
plt.figure(figsize=(10, 6))
plt.bar(results.keys(), results.values())
plt.title('모델 정확도 비교')
plt.xlabel('모델')
plt.ylabel('정확도')
plt.ylim(0.8, 1.0)  # 정확도가 다 높아서 차이 보려고 0.8부터 시작
for i, (name, accuracy) in enumerate(results.items()):
    plt.text(i, accuracy + 0.01, f'{accuracy:.4f}', ha='center')

# 최고 모델 찾기
best_model_name = max(results, key=results.get)  # 가장 높은 정확도 모델 찾기
best_model = models[best_model_name]
print(f"\n최고 모델: {best_model_name} (정확도: {results[best_model_name]:.4f})")

# 새 데이터 예측해보기
# 테스트셋에서 몇개 뽑아서 예측해봄
sample_indices = np.random.choice(len(X_test), 5, replace=False)  # 5개 샘플 랜덤 선택
sample_X = X_test[sample_indices]
sample_y = y_test[sample_indices]

# 예측
sample_pred = best_model.predict(sample_X)

# 결과 출력
print("\n새 데이터 예측 결과:")
for i, (features, true_label, pred_label) in enumerate(zip(sample_X, sample_y, sample_pred)):
    print(f"샘플 {i+1}:")
    for j, feature in enumerate(features):
        print(f"  {feature_names[j]}: {feature:.2f}")
    print(f"  실제: {target_names[true_label]}")
    print(f"  예측: {target_names[pred_label]}")
    print(f"  결과: {'정확' if true_label == pred_label else '틀림'}")
    print()

# 의사결정 경계 시각화 - 이건 교수님한테 잘 보이려고 추가
plt.figure(figsize=(12, 8))

# 2개 특성만 선택 (전부 다 하면 4차원인데 못그림)
features = [0, 1]  # sepal length, sepal width
feature_names_selected = [feature_names[i] for i in features]

# 격자 만들기
x_min, x_max = X[:, features[0]].min() - 0.5, X[:, features[0]].max() + 0.5
y_min, y_max = X[:, features[1]].min() - 0.5, X[:, features[1]].max() + 0.5
xx, yy = np.meshgrid(np.arange(x_min, x_max, 0.01), np.arange(y_min, y_max, 0.01))

# 각 모델별로 의사결정 경계 그리기
for i, (name, model) in enumerate(models.items()):
    plt.subplot(2, 3, i+1)  # 2행 3열 (마지막 하나는 빈칸)
    
    # 의사결정 경계용 데이터 - 다른 특성은 0으로 채움 (존나 꼼수임)
    Z = model.predict(np.c_[xx.ravel(), yy.ravel(), 
                          np.zeros(xx.ravel().shape[0]), 
                          np.zeros(xx.ravel().shape[0])])
    Z = Z.reshape(xx.shape)
    
    # 의사결정 경계 그리기
    plt.contourf(xx, yy, Z, alpha=0.3, cmap='rainbow')
    
    # 실제 데이터 포인트 그리기
    for target in range(3):
        plt.scatter(X[y == target][:, features[0]], X[y == target][:, features[1]], 
                   label=target_names[target], edgecolors='k', alpha=0.6)
    
    plt.title(f'{name} 의사결정 경계')
    plt.xlabel(feature_names_selected[0])
    plt.ylabel(feature_names_selected[1])
    if i == 0:  # 범례는 첫번째 그래프에만
        plt.legend()

plt.tight_layout()
plt.suptitle('의사결정 경계 비교', y=1.02, fontsize=15)
plt.show()  # 이거 없어도 그려지는데 그냥 씀