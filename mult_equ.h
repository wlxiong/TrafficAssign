#ifndef MULT_EQU_H
#define MULT_EQU_H

void load_part_trip(double percent);
void logit_mult_direction();
void init_mult_set();
void logit_mult_load();
double master_problem_mult(double criterion);
bool column_gen_mult();
void mult_logit(double criterion);

#endif
