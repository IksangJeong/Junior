"""
수학 알고리즘 연습 #41
"""
import math

def compute_41(n):
    """수학 계산 #41"""
    # 소인수분해
    factors = []
    d = 2
    temp = n
    while d * d <= temp:
        while temp % d == 0:
            factors.append(d)
            temp //= d
        d += 1
    if temp > 1:
        factors.append(temp)
    return factors

def is_prime_41(n):
    if n < 2:
        return False
    for i in range(2, int(math.sqrt(n)) + 1):
        if n % i == 0:
            return False
    return True

if __name__ == "__main__":
    test_num = 797
    print(f"연습 #41")
    print(f"{test_num}의 소인수분해: {compute_41(test_num)}")
    primes = [x for x in range(2, 255) if is_prime_41(x)]
    print(f"소수 목록: {primes[:15]}...")
