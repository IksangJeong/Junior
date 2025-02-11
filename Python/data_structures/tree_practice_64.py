"""
트리 자료구조 연습 #64
"""

class TreeNode:
    def __init__(self, val):
        self.val = val
        self.left = None
        self.right = None

class BST_64:
    """이진 탐색 트리 #64"""
    def __init__(self):
        self.root = None

    def insert(self, val):
        if not self.root:
            self.root = TreeNode(val)
        else:
            self._insert(self.root, val)

    def _insert(self, node, val):
        if val < node.val:
            if node.left is None:
                node.left = TreeNode(val)
            else:
                self._insert(node.left, val)
        else:
            if node.right is None:
                node.right = TreeNode(val)
            else:
                self._insert(node.right, val)

    def inorder(self):
        result = []
        self._inorder(self.root, result)
        return result

    def _inorder(self, node, result):
        if node:
            self._inorder(node.left, result)
            result.append(node.val)
            self._inorder(node.right, result)

    def height(self):
        return self._height(self.root)

    def _height(self, node):
        if not node:
            return 0
        return 1 + max(self._height(node.left), self._height(node.right))

if __name__ == "__main__":
    bst = BST_64()
    values = [76, 55, 40, 73, 80, 8, 79, 95, 13]
    for v in values:
        bst.insert(v)
    print(f"연습 #64")
    print(f"중위 순회: {bst.inorder()}")
    print(f"트리 높이: {bst.height()}")
