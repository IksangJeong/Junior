#include <stdio.h>

int main() {
    int height;

   
    printf("피라미드를 몇 층으로 쌓을건가요? ");
    scanf("%d", &height);

    for (int i = 1; i <= height; i++) {
       
        for (int j = 1; j <= height - i; j++) {
            printf(" ");
        }
       
        for (int k = 1; k <= (2 * i - 1); k++) {
            printf("*");
        }
        printf("\n");
    }

    return 0;
}