#include <stdio.h>
#include <string.h>
#define MAX 100  



// 주소록 구조체 정의
typedef struct {
    char name[20];
    char phone[15];
    char studentID[10];
    char birth[11];
} student;

student contacts[MAX];  // 주소록 배열
int count = 0;  // 현재 저장된 개수

// 목록 출력 함수
void listContacts() {
    printf("\n===== 주소록 목록 =====\n");
    for (int i = 0; i < count; i++) {
        printf("%d\t%s\t%s\t%s\t%s\n", i + 1, contacts[i].name, contacts[i].phone, contacts[i].studentID, contacts[i].birth);
    }
}

// 새로운 연락처 추가
void addContact() {
    if (count >= MAX) {
        printf("주소록이 가득 찼습니다!\n");
        return;
    }
    
    printf("\n===== 연락처 추가 =====\n");
    printf("이름: "); scanf_s("%s", contacts[count].name, sizeof(contacts[count].name));
    printf("전화번호: "); scanf_s("%s", contacts[count].phone, sizeof(contacts[count].phone));
    printf("학번: "); scanf_s("%s", contacts[count].studentID, sizeof(contacts[count].studentID));
    printf("생일(YYYY.MM.DD): "); scanf_s("%s", contacts[count].birth, sizeof(contacts[count].birth));
    
    count++;
    printf("추가 완료!\n");
}

// 이름으로 검색
void searchContact() {
    char searchName[20];
    int found = 0;
    
    printf("\n===== 연락처 검색 =====\n");
    printf("검색할 이름: "); scanf_s("%s", searchName, sizeof(searchName));
    
    for (int i = 0; i < count; i++) {
        if (strcmp(contacts[i].name, searchName) == 0) {
            printf("%d\t%s\t%s\t%s\t%s\n", i + 1, contacts[i].name, contacts[i].phone, contacts[i].studentID, contacts[i].birth);
            found = 1;
        }
    }
    
    if (!found) {
        printf("해당 이름의 연락처가 없습니다.\n");
    }
}

// 연락처 삭제
void deleteContact() {
    char deleteName[20];
    printf("\n===== 연락처 삭제 =====\n");
    printf("삭제할 이름: "); scanf_s("%s", deleteName, sizeof(deleteName));
    
    int found = 0;
    for (int i = 0; i < count; i++) {
        if (strcmp(contacts[i].name, deleteName) == 0) {
            found = 1;
            // 삭제할 항목 이후의 모든 항목을 한 칸씩 앞으로 이동
            for (int j = i; j < count + 1 ; j++) {
                contacts[j] = contacts[j + 1];  // 올바르게 수정: j+1의 데이터를 j로 복사
            }
            count--;
            printf("삭제 완료!\n");
            return;
        }
    }
    
    if (!found) {
        printf("해당 이름의 연락처가 없습니다.\n");
    }
}


#include <stdio.h>
#include <string.h>

#define _CRT_SECURE_NO_WARNINGS

#define MAX 100  // 최대 저장 개수

// 주소록 구조체 정의
typedef struct {
    char name[20];
    char phone[15];
    char studentID[10];
    char birth[11];
} student;

student contacts[MAX];  // 주소록 배열
int count = 0;  // 현재 저장된 개수

// 목록 출력 함수
void listContacts() {
    printf("\n===== 주소록 목록 =====\n");
    for (int i = 0; i < count; i++) {
        printf("%d\t%s\t%s\t%s\t%s\n", i + 1, contacts[i].name, contacts[i].phone, contacts[i].studentID, contacts[i].birth);
    }
}

// 새로운 연락처 추가
void addContact() {
    if (count >= MAX) {
        printf("주소록이 가득 찼습니다!\n");
        return;
    }
    printf("\n===== 연락처 추가 =====\n");
    printf("이름: "); scanf_s("%s", contacts[count].name, 30);
    printf("전화번호: "); scanf_s("%s", contacts[count].phone, 30);
    printf("학번: "); scanf_s("%s", contacts[count].studentID, 30);
    printf("생일(YYYY.MM.DD): "); scanf_s("%s", contacts[count].birth, 30);

    count++;
    printf("추가 완료!\n");
}

// 이름으로 검색
void searchContact() {
    char searchName[20];
    printf("\n===== 연락처 검색 =====\n");
    printf("검색할 이름: "); scanf_s("%s", searchName, 30);

    for (int i = 0; i < count; i++) {
        if (strcmp(contacts[i].name, searchName) == 0) {
            printf("%d\t%s\t%s\t%s\t%s\n", i + 1, contacts[i].name, contacts[i].phone, contacts[i].studentID, contacts[i].birth);
            return;
        }
    }
    printf("해당 이름의 연락처가 없습니다.\n");
}

// 연락처 삭제
void deleteContact() {
    char deleteName[20];
    printf("\n===== 연락처 삭제 =====\n");
    printf("삭제할 이름: "); scanf_s("%s", deleteName, 30);

    int found = 0;
    for (int i = 0; i < count; i++){
        if (strcmp(contacts[i].name, deleteName) == 0) {
            found = 1;
            for (int j = i; j < count - 1; j++) {
                contacts[j] = contacts[j + 1];  // 한 칸씩 당김
            }
            count--;
            printf("삭제 완료!\n");
            return; 
        }
    }
    // char* dnm = &deleteName;
//     int found = 0;
//     for (int i = 0; i < count; i++) {
//         if (strcmp(contacts[i].name, dnm) == 0) {
//             found = 1;
//             for (int j = i; j < count - 1; j++) {
//                 contacts[j] = contacts[j - 1];  // 한 칸씩 당김
//             }
//             count--;
//             printf("삭제 완료!\n");
//             return;
//         }
//     }
//     if (!found) {
//         printf("해당 이름의 연락처가 없습니다.\n");
//     }
// }

// 메인 함수
// int main() {
//     int choice;

//     while (1) {
//         printf("\n===== 주소록 관리 프로그램 =====\n");
//         printf("1. 목록 보기\n");
//         printf("2. 추가하기\n");
//         printf("3. 검색하기(이름)\n");
//         printf("4. 삭제하기\n");
//         printf("5. 종료\n");
//         printf("선택: ");
//         scanf_s("%d", &choice);

//         switch (choice) {
//         case 1: listContacts(); break;
//         case 2: addContact(); break;
//         case 3: searchContact(); break;
//         case 4: deleteContact(); break;
//         case 5: printf("프로그램 종료\n"); return 0;
//         default: printf("잘못된 입력입니다. 다시 시도하세요.\n");
//         }
//     }
// }




// // 메인 함수
// int main() {
//     int choice;
    
//     while (1) {
//         printf("\n===== 주소록 관리 프로그램 =====\n");
//         printf("1. 목록 보기\n");
//         printf("2. 추가하기\n");
//         printf("3. 검색하기(이름)\n");
//         printf("4. 삭제하기\n");
//         printf("5. 종료\n");
//         printf("선택: ");
//         scanf_s("%d", &choice);
        
//         switch (choice) {
//             case 1: listContacts(); break;
//             case 2: addContact(); break;
//             case 3: searchContact(); break;
//             case 4: deleteContact(); break;
//             case 5: printf("프로그램 종료\n"); return 0;
//             default: printf("잘못된 입력입니다. 다시 시도하세요.\n");
//         }
//     }
// }
// #include <stdio.h>
// #include <string.h>
// #define MAX 100  // 최대 저장 개수

// // 주소록 구조체 정의
// typedef struct {
//     char name[20];
//     char phone[15];
//     char studentID[10];
//     char birth[11];
// } student;

// student contacts[MAX];  // 주소록 배열
// int count = 0;  // 현재 저장된 개수

// // 목록 출력 함수
// void listContacts() {
//     printf("\n===== 주소록 목록 =====\n");
//     for (int i = 0; i < count; i++) {
//         printf("%d\t%s\t%s\t%s\t%s\n", i + 1, contacts[i].name, contacts[i].phone, contacts[i].studentID, contacts[i].birth);
//     }
// }

// // 새로운 연락처 추가
// void addContact() {
//     if (count >= MAX) {
//         printf("주소록이 가득 찼습니다!\n");
//         return;
//     }
    
//     printf("\n===== 연락처 추가 =====\n");
//     printf("이름: "); scanf_s("%s", contacts[count].name, sizeof(contacts[count].name));
//     printf("전화번호: "); scanf_s("%s", contacts[count].phone, sizeof(contacts[count].phone));
//     printf("학번: "); scanf_s("%s", contacts[count].studentID, sizeof(contacts[count].studentID));
//     printf("생일(YYYY.MM.DD): "); scanf_s("%s", contacts[count].birth, sizeof(contacts[count].birth));
    
//     count++;
//     printf("추가 완료!\n");
// }

// // 이름으로 검색
// void searchContact() {
//     char searchName[20];
//     int found = 0;
    
//     printf("\n===== 연락처 검색 =====\n");
//     printf("검색할 이름: "); scanf_s("%s", searchName, sizeof(searchName));
    
//     for (int i = 0; i < count; i++) {
//         if (strcmp(contacts[i].name, searchName) == 0) {
//             printf("%d\t%s\t%s\t%s\t%s\n", i + 1, contacts[i].name, contacts[i].phone, contacts[i].studentID, contacts[i].birth);
//             found = 1;
//         }
//     }
    
//     if (!found) {
//         printf("해당 이름의 연락처가 없습니다.\n");
//     }
// }

// // 연락처 삭제
// void deleteContact() {
//     char deleteName[20];
//     printf("\n===== 연락처 삭제 =====\n");
//     printf("삭제할 이름: "); scanf_s("%s", deleteName, sizeof(deleteName));
    
//     int found = 0;
//     for (int i = 0; i < count; i++) {
//         if (strcmp(contacts[i].name, deleteName) == 0) {
//             found = 1;
//             // 삭제할 항목 이후의 모든 항목을 한 칸씩 앞으로 이동
//             for (int j = i; j < count - 1; j++) {
//                 contacts[j] = contacts[j + 1];  // 올바르게 수정: j+1의 데이터를 j로 복사
//             }
//             count--;
//             printf("삭제 완료!\n");
//             return;
//         }
//     }
    
//     if (!found) {
//         printf("해당 이름의 연락처가 없습니다.\n");
//     }
// }

// // 메인 함수
// int main() {
//     int choice;
    
//     while (1) {
//         printf("\n===== 주소록 관리 프로그램 =====\n");
//         printf("1. 목록 보기\n");
//         printf("2. 추가하기\n");
//         printf("3. 검색하기(이름)\n");
//         printf("4. 삭제하기\n");
//         printf("5. 종료\n");
//         printf("선택: ");
//         scanf_s("%d", &choice);
        
//         switch (choice) {
//             case 1: listContacts(); break;
//             case 2: addContact(); break;
//             case 3: searchContact(); break;
//             case 4: deleteContact(); break;
//             case 5: printf("프로그램 종료\n"); return 0;
//             default: printf("잘못된 입력입니다. 다시 시도하세요.\n");
//         }
//     }
// }