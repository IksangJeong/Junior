// 문이 5개가 있고, 각 문마다 점점 어려운 수식 퀴즈가 출제(랜덤)
// 맞히면 통과, 틀리면 실패
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int getRandomeNumber(int level);
void showQuestion(int num1, int num2);

int main(void)
{
    srand(time(NULL));
    int count = 0;
    for (int i = 1; i < 5; i++)
    {
        // x * y = ?
        int num1 = getRandomeNumber(i);
        int num2 = getRandomeNumber(i);
        //printf("%d * %d = ?\n", num1, num2);
        showQuestion(num1, num2);
    }
    return 0;
}

int getRandomeNumber(int level)
{
    return rand() % (level * 7) + 1;
}

void showQuestion(int num1, int num2)
{
    printf("\n\n\n########## %d x %d = ? ##########\n\n\n", num1, num2);
}
