// up and down

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main() {
    int answer;
    int guess;
    int tries = 0;
    int maxTries = 5;
    int min = 1;
    int max = 100;

    srand(time(NULL));
    answer = rand() % 100 + 1;

    printf("1부터 100 사이의 숫자를 맞춰보세요.\n");

    do {
        printf("숫자를 입력하세요: ");
        scanf("%d", &guess);
        tries++;

        if (guess > answer) {
            printf("DOWN!\n");
        } else if (guess < answer) {
            printf("UP!\n");
        } else {
            printf("정답입니다! %d번 만에 맞추셨습니다.\n", tries);
            break;
        }

        if (tries == maxTries) {
            printf("기회를 모두 소진하셨습니다. 정답은 %d입니다.\n", answer);
            break;
        }
    } while (1);

    return 0;
}