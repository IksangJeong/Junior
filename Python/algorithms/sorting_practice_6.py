"""
정렬 알고리즘 연습 #6
다양한 정렬 알고리즘 비교
"""

def sort_variant_6(arr):
    """정렬 변형 #6"""
    n = len(arr)
    for i in range(n):
        for j in range(i+1, n):
            if arr[j] < arr[i]:
                arr[i], arr[j] = arr[j], arr[i]
    return arr

def is_sorted(arr):
    return all(arr[i] <= arr[i+1] for i in range(len(arr)-1))

if __name__ == "__main__":
    import random
    test = random.sample(range(100), 20)
    print(f"연습 #6")
    print("정렬 전:", test[:5], "...")
    result = sort_variant_6(test.copy())
    print("정렬 후:", result[:5], "...")
    print("정렬 확인:", is_sorted(result))
