"""
문자열 알고리즘 연습 #22
"""

def string_process(s):
    """문자열 처리"""
    freq = {}
    for c in s:
        freq[c] = freq.get(c, 0) + 1
    if freq:
        max_char = max(freq, key=freq.get)
        return max_char, freq[max_char]
    return None, 0

def reverse_words(s):
    """단어 뒤집기"""
    return " ".join(s.split()[::-1])

if __name__ == "__main__":
    test_strings = [
        "hello world python",
        "algorithm practice number 22",
        "abcdefghijklmnop" * 2,
    ]
    for s in test_strings:
        char, count = string_process(s)
        print(f"문자열: {s[:30]}... 최빈: {char}({count}번)")
    print("뒤집기:", reverse_words(test_strings[0]))
