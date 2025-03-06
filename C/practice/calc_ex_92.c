/* 계산 연습 #92 */
#include <stdio.h>
#include <math.h>

/* 팩토리얼 */
long long factorial_92(int n) {
    if (n <= 1) return 1;
    return n * factorial_92(n - 1);
}

/* 피보나치 */
int fib_92(int n) {
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
double power_92(double base, int exp) {
    double result = 1.0;
    for (int i = 0; i < exp; i++) {
        result *= base;
    }
    return result;
}

int main() {
    printf("연습 #92\n");
    printf("11! = %lld\n", factorial_92(11));
    printf("fib(7) = %d\n", fib_92(7));
    printf("4^3 = %.0f\n", power_92(4, 3));
    return 0;
}
