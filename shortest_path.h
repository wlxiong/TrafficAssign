#ifndef SHORTEST_PATH_H
#define SHORTEST_PATH_H

void djikstra_all(int s);
void djikstra_one(int s, int e);
void bellman_ford(int s);
void bellman_ford_leng(int s);
void lagrangian_relax(int s, int e);

#endif
