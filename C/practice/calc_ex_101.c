/* 계산 연습 #101 */
#include <stdio.h>
#include <math.h>

/* 팩토리얼 */
long long factorial_101(int n) {
    if (n <= 1) return 1;
    return n * factorial_101(n - 1);
}

/* 피보나치 */
int fib_101(int n) {
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
double power_101(double base, int exp) {
    double result = 1.0;
    for (int i = 0; i < exp; i++) {
        result *= base;
    }
    return result;
}

int main() {
    printf("연습 #101\n");
    printf("8! = %lld\n", factorial_101(8));
    printf("fib(16) = %d\n", fib_101(16));
    printf("3^4 = %.0f\n", power_101(3, 4));
    return 0;
}
