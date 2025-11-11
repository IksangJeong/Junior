"""
ML 실험 #444
모델 학습 및 평가
"""
import numpy as np

class SimpleModel_444:
    """간단한 선형 모델 #444"""
    def __init__(self, n_features):
        self.weights = np.random.randn(n_features) * 0.01
        self.bias = 0.0
        self.lr = 0.005

    def predict(self, X):
        return X @ self.weights + self.bias

    def fit(self, X, y, epochs=94):
        n = len(X)
        losses = []
        for epoch in range(epochs):
            pred = self.predict(X)
            error = pred - y
            loss = np.mean(error ** 2)
            losses.append(loss)

            # 경사 하강법
            self.weights -= self.lr * (2/n) * (X.T @ error)
            self.bias -= self.lr * (2/n) * np.sum(error)

        return losses

    def score(self, X, y):
        pred = self.predict(X)
        ss_res = np.sum((y - pred) ** 2)
        ss_tot = np.sum((y - np.mean(y)) ** 2)
        return 1 - ss_res / (ss_tot + 1e-10)

if __name__ == "__main__":
    np.random.seed(444)
    n_samples = 4540
    n_features = 6

    X = np.random.randn(n_samples, n_features)
    true_weights = np.array([2.62, -2.16, 1.47, -1.57, 2.89, -1.99])
    y = X @ true_weights + 0.77 + np.random.randn(n_samples) * 0.5

    model = SimpleModel_444(n_features)
    losses = model.fit(X, y)

    print(f"실험 #444")
    print(f"샘플 수: {n_samples}, 특성 수: {n_features}")
    print(f"초기 손실: {losses[0]:.4f}")
    print(f"최종 손실: {losses[-1]:.4f}")
    print(f"R² 점수: {model.score(X, y):.4f}")
