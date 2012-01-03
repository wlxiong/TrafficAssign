#ifndef GLOBAL_VAR_H
#define GLOBAL_VAR_H

#include "data_struct.h"

#define MAX_NODE 2000
#define MAX_PAIR 10000
#define MAX_LINK 5000

//nodes are counted from 1, other objects are counted from 0. 
extern LINK links[MAX_LINK]; 
extern PAIR pairs[MAX_PAIR]; 
extern NODE nodes[MAX_NODE];
extern META metadata; 
extern const double INFINITE;
extern const double golden_rate;

#endif
