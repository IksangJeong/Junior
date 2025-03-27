/* 계산 연습 #128 */
#include <stdio.h>
#include <math.h>

/* 팩토리얼 */
long long factorial_128(int n) {
    if (n <= 1) return 1;
    return n * factorial_128(n - 1);
}

/* 피보나치 */
int fib_128(int n) {
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
double power_128(double base, int exp) {
    double result = 1.0;
    for (int i = 0; i < exp; i++) {
        result *= base;
    }
    return result;
}

int main() {
    printf("연습 #128\n");
    printf("11! = %lld\n", factorial_128(11));
    printf("fib(13) = %d\n", fib_128(13));
    printf("5^3 = %.0f\n", power_128(5, 3));
    return 0;
}
