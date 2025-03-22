//내가 입력한 수 만큼 구구단을 출력하는 프로그램
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

int main() {
    int num;
    printf("구구단을 출력할 숫자를 입력하세요: ");
    scanf("%d", &num);

    for (int i = 1; i <= 9; i++) {
        printf("%d * %d = %d\n", num, i, num * i);
    }

    return 0;
}