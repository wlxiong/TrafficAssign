#ifndef COLUMN_GEN_H
#define COLUMN_GEN_H

bool column_gen_path();
void init_path_set();
void logit_path_direction();
void logit_path_load();
double master_problem_path(double criterion);
void col_gen_logit(double criterion);

#endif
