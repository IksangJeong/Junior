/* C 프로그래밍 연습 #123 */
#include <stdio.h>

/* 배열 관련 함수 #123 */
void print_array_123(int arr[], int n) {
    for (int i = 0; i < n; i++) {
        printf("%d ", arr[i]);
    }
    printf("\n");
}

int sum_array_123(int arr[], int n) {
    int sum = 0;
    for (int i = 0; i < n; i++) {
        sum += arr[i];
    }
    return sum;
}

int max_element_123(int arr[], int n) {
    int max = arr[0];
    for (int i = 1; i < n; i++) {
        if (arr[i] > max) max = arr[i];
    }
    return max;
}

int main() {
    int arr[] = {369, 370, 371, 372, 373, 374, 375};
    int n = sizeof(arr) / sizeof(arr[0]);
    printf("연습 #123\n");
    printf("배열: ");
    print_array_123(arr, n);
    printf("합계: %d\n", sum_array_123(arr, n));
    printf("최대: %d\n", max_element_123(arr, n));
    return 0;
}
