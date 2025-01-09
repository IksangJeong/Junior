"""
그래프 알고리즘 연습 #21
"""
from collections import deque

def graph_traverse_21(adj, start):
    """그래프 순회 #21"""
    visited = set()
    queue = deque([start])
    visited.add(start)
    order = []
    while queue:
        node = queue.popleft()
        order.append(node)
        for neighbor in sorted(adj.get(node, [])):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
    return order

if __name__ == "__main__":
    graph = {}
    for i in range(4):
        graph[i] = [j for j in range(i+1, min(i+3, 4))]
    result = graph_traverse_21(graph, 0)
    print(f"연습 #21 - 그래프 순회: {result}")
