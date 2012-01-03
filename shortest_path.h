#ifndef SHORTEST_PATH_H
#define SHORTEST_PATH_H

void djikstra_all(int s);
void djikstra_one(int s, int e);
void bellman_ford(int s);
void bellman_ford_dist(int s);
void bellman_ford_dist_to_go(int s);
void bellman_ford_constrained(int s, int e, double b);

#endif
