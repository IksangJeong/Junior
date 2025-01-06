"""
수학 알고리즘 연습 #17
"""
import math

def compute_17(n):
    """수학 계산 #17"""
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

def is_prime_17(n):
    if n < 2:
        return False
    for i in range(2, int(math.sqrt(n)) + 1):
        if n % i == 0:
            return False
    return True

if __name__ == "__main__":
    test_num = 389
    print(f"연습 #17")
    print(f"{test_num}의 소인수분해: {compute_17(test_num)}")
    primes = [x for x in range(2, 135) if is_prime_17(x)]
    print(f"소수 목록: {primes[:15]}...")
