#include <cmath>
#include <limits>
#include <iostream>
#include "data_struct.h"
#include "global_var.h"

//nodes are counted from 1, other objects are counted from 0. 
LINK links[MAX_LINK]; 
PAIR pairs[MAX_PAIR]; 
NODE nodes[MAX_NODE];
META metadata;
const double INFINITE = std::numeric_limits<double>::max();
const double golden_rate = (sqrt(5.0) - 1)/2.0;
