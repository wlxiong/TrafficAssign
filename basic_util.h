#ifndef BASIC_UTIL_H
#define BASIC_UTIL_H

void set_flow(double f);
void set_direction(double d);
void set_route_flow(double f);
void set_path_flow(double f);
void set_route_direction(double d);
void set_path_direction(double d);
void init_link_length();
double travel_time(int i);
void update_travel_time();
void update_marginal_cost();
void update_general_cost();
void update_route_cost();
void update_path_cost();
void route_to_link_direction();
void route_to_link_flow();
void path_to_link_direction();
void path_to_link_flow();
double update_link_flow(double step);
double update_route_flow(double step);
double update_path_flow(double step);
void create_adj_list();
void create_rev_list();
double bisection(double eps, double a, double b, double (*diff)(double));
double golden_section(double eps, double a, double b, double (*func)(double));

#endif

