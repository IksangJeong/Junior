"""
동적 프로그래밍 연습 #26
"""

def solve_26(n):
    """DP 문제 #26: 계단 오르기 변형"""
    if n <= 0:
        return 0
    if n <= 2:
        return n
    dp = [0] * (n + 1)
    dp[1] = 1
    dp[2] = 2
    for i in range(3, n + 1):
        dp[i] = dp[i-1] + dp[i-2]
        if i >= 4:
            dp[i] += dp[i-4]
    return dp[n]

if __name__ == "__main__":
    for n in [5, 10, 15, 20]:
        print(f"n={n}: {solve_26(n)}")
    print(f"\n연습 #26 완료")
