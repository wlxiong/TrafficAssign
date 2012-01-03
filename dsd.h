#ifndef DSD_H
#define DSD_H

bool cmp_route(ROUTE& a, ROUTE& b);
bool generate_route();
void init_route_set();
void logit_route_direction();
void logit_route_load();
double master_problem(double criterion);
void dsd_logit(double criterion);

#endif
