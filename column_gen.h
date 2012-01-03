#ifndef COLUMN_GEN_H
#define COLUMN_GEN_H

bool generate_path(int p);
double search_path_direction(int p);
double column_subproblem(int pair);
void init_path_set();
void init_path_flow();
void column_gen();

#endif
