"""
탐색 알고리즘 연습 #31
"""

def search_31(arr, target):
    """탐색 함수 #31"""
    left, right = 0, len(arr) - 1
    steps = 0
    while left <= right:
        mid = (left + right) // 2
        steps += 1
        if arr[mid] == target:
            return mid, steps
        elif arr[mid] < target:
            left = mid + 1
        else:
            right = mid - 1
    return -1, steps

if __name__ == "__main__":
    data = list(range(0, 1000, 4))
    target = data[len(data)//3] if data else 0
    idx, steps = search_31(data, target)
    print(f"연습 #31: 검색 대상 {target}")
    print(f"결과 인덱스: {idx}, 탐색 횟수: {steps}")
