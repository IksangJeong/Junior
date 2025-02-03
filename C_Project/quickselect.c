#include <stdio.h>

int partition(int arr[], int l, int r) {
    int pivot = arr[r], i = l;
    for (int j = l; j < r; j++)
        if (arr[j] <= pivot)
            { int temp = arr[i]; arr[i] = arr[j]; arr[j] = temp; i++; }
    int temp = arr[i]; arr[i] = arr[r]; arr[r] = temp;
    return i;
}

int quickSelect(int arr[], int l, int r, int k) {
    if (l <= r) {
        int pi = partition(arr, l, r);
        if (pi == k) return arr[pi];
        if (pi > k) return quickSelect(arr, l, pi - 1, k);
        return quickSelect(arr, pi + 1, r, k);
    }
    return -1;
}

int main() {
    int arr[] = {7, 10, 4, 3, 20, 15};
    int n = sizeof(arr)/sizeof(arr[0]);
    int k = 2;
    printf("%d\n", quickSelect(arr, 0, n - 1, k - 1));
    return 0;
}