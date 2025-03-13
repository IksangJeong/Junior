/* 계산 연습 #110 */
#include <stdio.h>
#include <math.h>

/* 팩토리얼 */
long long factorial_110(int n) {
    if (n <= 1) return 1;
    return n * factorial_110(n - 1);
}

/* 피보나치 */
int fib_110(int n) {
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
double power_110(double base, int exp) {
    double result = 1.0;
    for (int i = 0; i < exp; i++) {
        result *= base;
    }
    return result;
}

int main() {
    printf("연습 #110\n");
    printf("5! = %lld\n", factorial_110(5));
    printf("fib(10) = %d\n", fib_110(10));
    printf("2^5 = %.0f\n", power_110(2, 5));
    return 0;
}
