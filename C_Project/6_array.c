#include <stdio.h>

int main(void)
{
    srand(time(NULL));
    printf("\n\n=== 아빠는 대머리 게임 === \n\n" );
    int answer; // 사용자 입력값
    int treatment = rand() % 4; // 발모제 선택

    int cntShowBottle = 0; // 이번 게임에 보여줄 병 갯수
    int prevContainBottle[3] = {0, 0, 0}; // 앞선 게임에 나온 병
    // 서로 보여주는 병 갯수를 다르게 하여 정답률 향상(3병, 5병, 7병)

    // 3병 중에 1개는 발모제
    for (int i = 1; i <= 3; i++)
    {
        int randBottle = rand() % 5; // 0~4

        // 앞선 게임에 나온 병이면 다시 뽑기
        if (prevContainBottle[0] == randBottle || prevContainBottle[1] == randBottle)
        {
            i--;
            continue;
        }

        prevContainBottle[i - 1] = randBottle;
        if (treatment == randBottle)
        {
            continue;
        }
        cntShowBottle++;
    }
}