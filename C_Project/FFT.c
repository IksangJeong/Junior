#include <stdio.h>
#include <complex.h>
#include <math.h>

#define N 8

void fft(complex double* a, int n) {
    if (n <= 1) return;
    complex double even[n/2], odd[n/2];

    for (int i = 0; i < n/2; i++) {
        even[i] = a[i*2];
        odd[i] = a[i*2+1];
    }

    fft(even, n/2);
    fft(odd, n/2);

    for (int k = 0; k < n/2; k++) {
        complex double t = cexp(-2.0 * I * M_PI * k / n) * odd[k];
        a[k] = even[k] + t;
        a[k + n/2] = even[k] - t;
    }
}

int main() {
    complex double a[N] = {1, 1, 1, 1, 0, 0, 0, 0};

    fft(a, N);

    for (int i = 0; i < N; i++)
        printf("(%lf, %lf)\n", creal(a[i]), cimag(a[i]));

    return 0;
}