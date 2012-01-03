#ifndef BASIC_UTIL_H
#define BASIC_UTIL_H

#include <iostream>
#include <cmath>
#include "data_struct.h"
#include "global_var.h"
#include "func.h"
using namespace std;

void set_flow(double f);
void set_single_flow(double f);
void set_direction(double d);
void set_route_flow(double f);
void set_path_flow(double f);
void set_route_direction(double d);
void set_path_direction(double d);
void update_travel_time();
void update_marginal_cost();
void update_general_cost();
void update_route_cost();
void update_path_cost();
void route_to_link();
void path_to_link();
void path_to_link(int p, int r, double d);
double update_link_flow(double step);
void remove_single_flow(int p);
double update_single_flow(double step);
double update_route_flow(double step);
double update_path_flow(double step);
void create_adj_list();
void create_rev_list();
double bisection(double eps, double a, double b, double (*diff)(double));
double golden_section(double eps, double a, double b, double (*func)(double));

#endif

