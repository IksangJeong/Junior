#include <stdio.h>
#include <limits.h>

#define V 5
#define E 8

struct Edge {
    int src, dest, weight;
};

void bellmanFord(struct Edge edges[], int V, int E, int src) {
    int dist[V];
    for (int i = 0; i < V; i++)
        dist[i] = INT_MAX;
    dist[src] = 0;

    for (int i = 0; i < V - 1; i++)
        for (int j = 0; j < E; j++)
            if (dist[edges[j].src] != INT_MAX && dist[edges[j].src] + edges[j].weight < dist[edges[j].dest])
                dist[edges[j].dest] = dist[edges[j].src] + edges[j].weight;

    for (int i = 0; i < E; i++)
        if (dist[edges[i].src] != INT_MAX && dist[edges[i].src] + edges[i].weight < dist[edges[i].dest]) {
            printf("Negative weight cycle detected\n");
            return;
        }

    for (int i = 0; i < V; i++)
        printf("Distance to %d: %d\n", i, dist[i]);
}

int main() {
    struct Edge edges[E] = {{0, 1, -1}, {0, 2, 4}, {1, 2, 3}, {1, 3, 2}, {1, 4, 2}, {3, 2, 5}, {3, 1, 1}, {4, 3, -3}};
    bellmanFord(edges, V, E, 0);
    return 0;
}