# 수열의 길이 입력
n = int(input())  # 수열의 길이
arr = list(map(int, input().split()))  # 수열 입력

# DP 배열 초기화
dp = [1] * n  # 각 위치의 LIS 길이를 저장, 최소 길이는 1

# LIS 계산
for i in range(1, n):
    for j in range(i):
        if arr[j] < arr[i]:  # 증가 조건을 만족하는 경우
            dp[i] = max(dp[i], dp[j] + 1)  # 현재 dp[i]와 dp[j] + 1 중 큰 값 선택

# 최장 증가 부분 수열의 길이 출력
print(max(dp))
