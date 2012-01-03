#ifndef LOAD_DATA_H
#define LOAD_DATA_H

#include <cstdio>
#include <string>
#include "data_struct.h"
using namespace std;

void print_link(FILE* fout, const LINK& link);
void print_trip(FILE* fout, const PAIR& pair);
void verify_data(const char* verify_file);

void* getln(FILE* fin, char* ln);
void load_net(const char* net_file);
void load_trip(const char* trip_file);
bool load_node(const char* node_file);
void load_case();

#endif
