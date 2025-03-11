/* C 프로그래밍 연습 #108 */
#include <stdio.h>

/* 배열 관련 함수 #108 */
void print_array_108(int arr[], int n) {
    for (int i = 0; i < n; i++) {
        printf("%d ", arr[i]);
    }
    printf("\n");
}

int sum_array_108(int arr[], int n) {
    int sum = 0;
    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }
    return sum;
}

int max_element_108(int arr[], int n) {
    int max = arr[0];
    for (int i = 1; i < n; i++) {
        if (arr[i] > max) max = arr[i];
    }
    return max;
}

int main() {
    int arr[] = {324, 325, 326, 327, 328, 329, 330};
    int n = sizeof(arr) / sizeof(arr[0]);
    printf("연습 #108\n");
    printf("배열: ");
    print_array_108(arr, n);
    printf("합계: %d\n", sum_array_108(arr, n));
    printf("최대: %d\n", max_element_108(arr, n));
    return 0;
}
