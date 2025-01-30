

#include <stdio.h>
#define V 5

int isSafe(int v, int graph[V][V], int path[], int pos) {
    if (!graph[path[pos - 1]][v]) return 0;
    for (int i = 0; i < pos; i++)
        if (path[i] == v) return 0;
    return 1;
}

int hamCycleUtil(int graph[V][V], int path[], int pos) {
    if (pos == V) return graph[path[pos - 1]][path[0]];
    for (int v = 1; v < V; v++) {
        if (isSafe(v, graph, path, pos)) {
            path[pos] = v;
            if (hamCycleUtil(graph, path, pos + 1)) return 1;
            path[pos] = -1;
        }
    }
    return 0;
}

void hamiltonianCycle(int graph[V][V]) {
    int path[V];
    for (int i = 0; i < V; i++) path[i] = -1;
    path[0] = 0;
    if (!hamCycleUtil(graph, path, 1)) printf("No Hamiltonian Cycle\n");
    else {
        for (int i = 0; i < V; i++) printf("%d ", path[i]);
        printf("%d\n", path[0]);
    }
}

int main() {
    int graph[V][V] = {{0, 1, 0, 1, 0}, {1, 0, 1, 1, 1}, {0, 1, 0, 0, 1}, {1, 1, 0, 0, 1}, {0, 1, 1, 1, 0}};
    hamiltonianCycle(graph);
    return 0;
}