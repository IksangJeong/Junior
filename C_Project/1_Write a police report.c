#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

// 상수 정의
#define MAX_NAME_LEN 256
#define MAX_CRIME_LEN 256

// 입력 함수
void getInputString(const char *prompt, char *output, int size) {
    printf("%s", prompt);
    scanf("%s", output);
}

void getInputInt(const char *prompt, int *output) {
    printf("%s", prompt);
    while (scanf("%d", output) != 1) {
        printf("유효한 정수를 입력하세요.\n");
        printf("%s", prompt);
        while (getchar() != '\n'); // 입력 버퍼 비우기
    }
}

void getInputFloat(const char *prompt, float *output) {
    printf("%s", prompt);
    while (scanf("%f", output) != 1) {
        printf("유효한 실수를 입력하세요.\n");
        printf("%s", prompt);
        while (getchar() != '\n'); // 입력 버퍼 비우기
    }
}

void getInputDouble(const char *prompt, double *output) {
    printf("%s", prompt);
    while (scanf("%lf", output) != 1) {
        printf("유효한 실수를 입력하세요.\n");
        printf("%s", prompt);
        while (getchar() != '\n'); // 입력 버퍼 비우기
    }
}

// 출력 함수
void printCriminalInfo(const char *name, int age, float weight, double height, const char *crime) {
    printf("\n\n--- 범죄자 정보 ---\n\n");
    printf("이름: %s\n", name);
    printf("나이: %d\n", age);
    printf("몸무게: %.2f kg\n", weight);
    printf("키: %.2lf cm\n", height);
    printf("범죄명: %s\n", crime);
}

int main(void) {
    char name[MAX_NAME_LEN];
    int age;
    float weight;
    double height;
    char crime[MAX_CRIME_LEN];

    // 입력 받기
    getInputString("이름이 뭐에요? ", name, MAX_NAME_LEN);
    getInputInt("몇살이에요? ", &age);
    getInputFloat("몸무게는 몇 kg이에요? ", &weight);
    getInputDouble("키는 몇 cm이에요? ", &height);
    getInputString("무슨 범죄를 저질렀나요? ", crime, MAX_CRIME_LEN);

    // 출력
    printCriminalInfo(name, age, weight, height, crime);

    return 0;
}
/*
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

// 프로젝트
// 경찰관이 범죄자의 정보를 입수 (조서 작성)
// 이름? 나이? 몸무게? 키? 범죄명?

int main (void)
{
    char name [256];
    printf("이름이 뭐에요? ");
    scanf("%s", name , sizeof(name));

    int age;
    printf("몇살이에요? ");
    scanf("%d", &age);

    float weight;
    printf("몸무게는 몇 kg이에요? ");
    scanf("%f", &weight);

    double height;
    printf("키는 몇 cm이에요? ");
    scanf("%lf", &height);

    char what [256];
    printf("무슨 범죄를 저질렀나요? ");
    scanf("%s", what , sizeof(what));

    // 조서 내용 출력
    printf("\n\n--- 범죄자 정보 ---\n\n");
    printf("이름: %s\n", name);
    printf("나이: %d\n", age);
    printf("몸무게: %.2f\n", weight);
    printf("키: %.2lf\n", height);
    printf("범죄명: %s\n", what);

    return 0;   
}
*/


