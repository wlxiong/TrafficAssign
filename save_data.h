#ifndef SAVE_DATA_H
#define SAVE_DATA_H

#include <string>
using namespace std;

void print_flow(const char* ps_file);
void save_flow(const char* flow_file);
void save_route(const char* route_file);
void save_data();
void verify_assign();

#endif
