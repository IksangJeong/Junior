#include <stdio.h>
#include <string.h>

void computeLPSArray(char* pat, int M, int* lps) {
    int len = 0, i = 1;
    lps[0] = 0;

    while (i < M) {
        if (pat[i] == pat[len]) lps[i++] = ++len;
        else if (len) len = lps[len - 1];
        else lps[i++] = 0;
    }
}

void KMPSearch(char* pat, char* txt) {
    int M = strlen(pat), N = strlen(txt), lps[M], i = 0, j = 0;
    computeLPSArray(pat, M, lps);

    while (i < N) {
        if (pat[j] == txt[i]) { i++; j++; }
        if (j == M) { printf("Found at index %d\n", i - j); j = lps[j - 1]; }
        else if (i < N && pat[j] != txt[i]) j ? (j = lps[j - 1]) : i++;
    }
}

int main() {
    char txt[] = "ababcabcabababd";
    char pat[] = "ababd";
    KMPSearch(pat, txt);
    return 0;
}