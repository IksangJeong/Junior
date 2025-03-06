/* 계산 연습 #95 */
#include <stdio.h>
#include <math.h>

/* 팩토리얼 */
long long factorial_95(int n) {
    if (n <= 1) return 1;
    return n * factorial_95(n - 1);
}

/* 피보나치 */
int fib_95(int n) {
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
double power_95(double base, int exp) {
    double result = 1.0;
    for (int i = 0; i < exp; i++) {
        result *= base;
    }
    return result;
}

int main() {
    printf("연습 #95\n");
    printf("14! = %lld\n", factorial_95(14));
    printf("fib(10) = %d\n", fib_95(10));
    printf("2^6 = %.0f\n", power_95(2, 6));
    return 0;
}
