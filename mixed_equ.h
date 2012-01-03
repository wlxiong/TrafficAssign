#ifndef MIXED_EQU_H
#define MIXED_EQU_H

void load_part_trip(double percent);
void init_mult_set();
void init_mult_flow();
bool column_gen_mult();
void search_mult_direction();
double master_problem_mixed(double criterion);
void mixed_equilibrium(double criterion);

#endif
