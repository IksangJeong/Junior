"""
리스트 자료구조 연습 #72
"""

class CustomList_72:
    """커스텀 리스트 #72"""
    def __init__(self):
        self.data = []
        self.size = 0

    def append(self, value):
        self.data.append(value)
        self.size += 1

    def insert_at(self, index, value):
        if 0 <= index <= self.size:
            self.data.insert(index, value)
            self.size += 1

    def remove_at(self, index):
        if 0 <= index < self.size:
            val = self.data.pop(index)
            self.size -= 1
            return val
        return None

    def find(self, value):
        for i, v in enumerate(self.data):
            if v == value:
                return i
        return -1

    def __repr__(self):
        return f"CustomList({self.data})"

if __name__ == "__main__":
    lst = CustomList_72()
    for i in range(7):
        lst.append(i * 4)
    print(f"연습 #72: {lst}")
    print(f"크기: {lst.size}")
    lst.insert_at(2, 999)
    print(f"삽입 후: {lst}")
