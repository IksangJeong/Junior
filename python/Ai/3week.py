import numpy as np

a  = np.array([1,2,3,4,5,6])
print (a)
print (a.shape)

a1 = a[np.newaxis, :]
print (a1)
print (a1.shape)

miles = np.array([1,2,3])
result = miles * 1.6
print (result)

arr1 = np.array([[1,2],[3,4],[5,6]])
arr2 = np.array ([[2,2],[2,2],[2,2]])
result = arr1 * arr2
print (result)

scores = np.array([[99,93,60],[98,82,93],[93,65,81],[78,82,81]])
scores.mean(axis=0)
print (scores.mean(axis=0))

#균일 분포에서 난수 생성하고 선 그래프 그리기
import matplotlib.pyplot as plt
# np.random.seed(100)
# np.random.rand(5)

# np.random.seed(5,3)


np.random.randn(5)
print(np.random.randn(5))
plt.plot(np.random.rand(5))
plt.show()

#정규 분포에서 난수 생성하고 히스토그램 그리기
data = np.random.randn(1000)
plt.hist(data, bins=30)
plt.show()

#이항 분포에서 난수 생성하고 히스토그램 그리기
data = np.random.binomial(100,0.5,1000)
plt.hist(data, bins=30)
plt.show()
plt.figure(figsize=(15, 5))

plt.subplot(1, 3, 1)
plt.plot(np.random.rand(5))
plt.title('Random Uniform Distribution')

plt.subplot(1, 3, 2)
data = np.random.randn(1000)
plt.hist(data, bins=30)
plt.title('Normal Distribution')

plt.subplot(1, 3, 3)
data = np.random.binomial(100, 0.5, 1000)
plt.hist(data, bins=30)
plt.title('Binomial Distribution')

plt.tight_layout()
plt.show()

