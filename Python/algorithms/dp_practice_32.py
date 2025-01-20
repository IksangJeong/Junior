"""
동적 프로그래밍 연습 #32
"""

def solve_32(n):
    """DP 문제 #32: 계단 오르기 변형"""
    if n <= 0:
        return 0
    if n <= 2:
        return n
    dp = [0] * (n + 1)
    dp[1] = 1
    dp[2] = 2
    for i in range(3, n + 1):
        dp[i] = dp[i-1] + dp[i-2]
        if i >= 5:
            dp[i] += dp[i-5]
    return dp[n]

if __name__ == "__main__":
    for n in [5, 10, 15, 20]:
        print(f"n={n}: {solve_32(n)}")
    print(f"\n연습 #32 완료")
