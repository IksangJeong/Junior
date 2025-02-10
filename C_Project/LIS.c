#include <stdio.h>

int lis(int arr[], int n) {
    int dp[n], max = 1;
    for (int i = 0; i < n; i++) dp[i] = 1;

    for (int i = 1; i < n; i++)
        for (int j = 0; j < i; j++)
            if (arr[i] > arr[j] && dp[i] < dp[j] + 1)
                dp[i] = dp[j] + 1, max = (dp[i] > max) ? dp[i] : max;

    return max;
}

int main() {
    int arr[] = {10, 22, 9, 33, 21, 50, 41, 60, 80};
    int n = sizeof(arr) / sizeof(arr[0]);
    printf("%d\n", lis(arr, n));
    return 0;
}