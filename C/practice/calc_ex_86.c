/* 계산 연습 #86 */
#include <stdio.h>
#include <math.h>

/* 팩토리얼 */
long long factorial_86(int n) {
    if (n <= 1) return 1;
    return n * factorial_86(n - 1);
}

/* 피보나치 */
int fib_86(int n) {
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
double power_86(double base, int exp) {
    double result = 1.0;
    for (int i = 0; i < exp; i++) {
        result *= base;
    }
    return result;
}

int main() {
    printf("연습 #86\n");
    printf("5! = %lld\n", factorial_86(5));
    printf("fib(16) = %d\n", fib_86(16));
    printf("3^5 = %.0f\n", power_86(3, 5));
    return 0;
}
