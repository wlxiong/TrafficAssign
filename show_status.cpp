#include <cstdio>
#include <iostream>
#include "data_struct.h"

void disp_link(const LINK& link){
	printf("%5d  %5d  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %d\n", 
		link.init_node, link.term_node, link.capacity, 
		link.length, link.free_time, link.b, link.power, 
		link.speed_limit, link.toll, link.type);
}

void disp_trip(const PAIR& pair){
	printf("O %-4d  D %-4d  %lf\n", 
		pair.origin, pair.destination, pair.trip);
}

void rep_error(const char* msg, const char* obj){
	printf("[error] %s \"%s\"\n", msg, obj);
	exit (EXIT_FAILURE);
}

void show_msg(const char* msg, const char* obj){
	printf("[status] %s \"%s\"\n", msg, obj);
}
