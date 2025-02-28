"""
해시 자료구조 연습 #80
"""

class HashMap_80:
    """해시맵 #80"""
    def __init__(self, capacity=10):
        self.capacity = capacity
        self.buckets = [[] for _ in range(capacity)]
        self.count = 0

    def _hash(self, key):
        return hash(key) % self.capacity

    def put(self, key, value):
        idx = self._hash(key)
        for i, (k, v) in enumerate(self.buckets[idx]):
            if k == key:
                self.buckets[idx][i] = (key, value)
                return
        self.buckets[idx].append((key, value))
        self.count += 1

    def get(self, key):
        idx = self._hash(key)
        for k, v in self.buckets[idx]:
            if k == key:
                return v
        return None

    def keys(self):
        result = []
        for bucket in self.buckets:
            for k, v in bucket:
                result.append(k)
        return result

if __name__ == "__main__":
    hm = HashMap_80()
    data = {"key_" + str(i): i * 1 for i in range(3)}
    for k, v in data.items():
        hm.put(k, v)
    print(f"연습 #80: 해시맵 크기 {hm.count}")
    for k in hm.keys():
        print(f"  {k}: {hm.get(k)}")
