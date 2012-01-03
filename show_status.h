#ifndef SHOW_STATUS_H
#define SHOW_STATUS_H

#include "data_struct.h"

void disp_link(const LINK& link);
void disp_trip(const PAIR& pair);
void rep_error(const char* msg, const char* obj);
void show_msg(const char* msg, const char* obj);

#endif
