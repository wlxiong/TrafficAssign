#include <cmath>
#include <cstdio>
#include "global_var.h"
#include "basic_util.h"

double SO_link_obj(double step){
	double INT = 0.0, flow;
	int i;

	for(i=0; i<metadata.n_link; i++){
		flow = links[i].flow + step*links[i].direction;
		INT += flow * links[i].free_time
			*(1 + links[i].b * pow(flow/links[i].capacity, links[i].power));
	}
	return INT;
}

double UE_link_obj(double step){
	double INT = 0.0, flow;
	int i;

	for(i=0; i<metadata.n_link; i++){
		flow = links[i].flow + step*links[i].direction;
		INT += links[i].free_time*flow;
		INT += links[i].free_time*links[i].b*flow/(links[i].power + 1)
			*pow(flow/links[i].capacity, links[i].power);
	}

	return INT;
}

double UE_link_diff(double step){
	double DIFF = 0.0, flow;
	int i;

	for(i=0; i<metadata.n_link; i++){
		flow = links[i].flow + step*links[i].direction;
		DIFF += links[i].direction*links[i].free_time 
			*(1.0 + links[i].b*pow(flow/links[i].capacity, links[i].power));
	}

	return DIFF;
}

double SUE_route_logit(double step){
	double INT = 0.0, flow;
	int p, r, l;

	for(p=0; p<metadata.n_pair; p++){
		for(r=0; r<pairs[p].n_route; r++){
			flow = pairs[p].routes[r].flow + step*pairs[p].routes[r].direction;
			INT += flow*log(flow);
		}
	}
	INT /= metadata.theta;
	
	set_direction(0.0);
	route_to_link_direction();
	for(l=0; l<metadata.n_link; l++){
		flow = links[l].flow + step*links[l].direction;
		INT += links[l].free_time*flow;
		INT += links[l].free_time*links[l].b*flow/(links[l].power + 1)
			* pow(flow/links[l].capacity, links[l].power);
	}

	return INT;
}

double SUE_SO_mixed(double step){
	double INT = 0.0, flow;
	int p, r, l, i, j, k;

// the first term of objective
	for(p=0; p<metadata.n_pair; p++){
		for(r=0; r<pairs[p].n_route; r++){
			flow = pairs[p].routes[r].flow + step*pairs[p].routes[r].direction;
			INT += flow*log(flow);
		}
	}
	INT /= metadata.theta;
//	printf("SUE_SO_mixed() step %lf\n", step);
//	printf("SUE_SO_mixed() INT1 %lf\n", INT);

// the second term of objective
	set_direction(0.0);
	route_to_link_direction();
	path_to_link_direction();
	for(l=0; l<metadata.n_link; l++){
		flow = links[l].flow + step*links[l].direction;
		INT += links[l].free_time*links[l].b*flow/(links[l].power + 1.0)
			* pow(flow/links[l].capacity, links[l].power);
	}
//	printf("SUE_SO_mixed() INT2 %lf\n", INT);

// the third term of objective
	set_flow(0.0);
	set_direction(0.0);
	route_to_link_flow();
	route_to_link_direction();
	for(l=0; l<metadata.n_link; l++){
		flow = links[l].flow + step*links[l].direction;
		INT += links[l].free_time*flow;
	}
//	printf("SUE_SO_mixed() INT3 %lf\n", INT);

// the fouth term of objective
	set_flow(0.0);
	set_direction(0.0);
	path_to_link_flow();
	path_to_link_direction();
	for(l=0; l<metadata.n_link; l++){
		flow = links[l].flow + step*links[l].direction;
		INT += links[l].free_time*flow/(links[l].power + 1.0);
	}
//	printf("SUE_SO_mixed() INT4 %lf\n", INT);

	route_to_link_flow();
	route_to_link_direction();

	return INT;
}
