/* 계산 연습 #125 */
#include <stdio.h>
#include <math.h>

/* 팩토리얼 */
long long factorial_125(int n) {
    if (n <= 1) return 1;
    return n * factorial_125(n - 1);
}

/* 피보나치 */
int fib_125(int n) {
    if (n <= 1) return n;
    int a = 0, b = 1;
    for (int i = 2; i <= n; i++) {
        int temp = a + b;
        a = b;
        b = temp;
    }
    return b;
}

/* 거듭제곱 */
double power_125(double base, int exp) {
    double result = 1.0;
    for (int i = 0; i < exp; i++) {
        result *= base;
    }
    return result;
}

int main() {
    printf("연습 #125\n");
    printf("8! = %lld\n", factorial_125(8));
    printf("fib(10) = %d\n", fib_125(10));
    printf("2^4 = %.0f\n", power_125(2, 4));
    return 0;
}
