/* 문자열 연습 #100 */
#include <stdio.h>
#include <string.h>

/* 문자열 길이 계산 (직접 구현) */
int my_strlen_100(const char *s) {
    int len = 0;
    while (s[len] != '\0') len++;
    return len;
}

/* 문자열 복사 */
void my_strcpy_100(char *dest, const char *src) {
    int i = 0;
    while (src[i] != '\0') {
        dest[i] = src[i];
        i++;
    }
    dest[i] = '\0';
}

/* 문자열 비교 */
int my_strcmp_100(const char *s1, const char *s2) {
    while (*s1 && *s1 == *s2) {
        s1++;
        s2++;
    }
    return *s1 - *s2;
}

int main() {
    char str1[] = "Hello C Programming 100";
    char str2[100];
    my_strcpy_100(str2, str1);
    printf("연습 #100\n");
    printf("원본: %s (길이: %d)\n", str1, my_strlen_100(str1));
    printf("복사: %s\n", str2);
    printf("비교: %d\n", my_strcmp_100(str1, str2));
    return 0;
}
