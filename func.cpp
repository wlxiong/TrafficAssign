#include <cmath>
#include <cstdio>
#include "global_var.h"
#include "basic_util.h"

double SO_link_obj(double step){
	double INT = 0.0, flow;
	int i;
	for(i=0; i<metadata.n_link; i++){
		flow = links[i].flow + step*links[i].direction;
		INT += links[i].cost*flow;
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
	route_to_link();
	for(l=0; l<metadata.n_link; l++){
		flow = links[l].flow + step*links[l].direction;
		INT += links[l].free_time*flow;
		INT += links[l].free_time*links[l].b*flow/(links[l].power + 1)
			* pow(flow/links[l].capacity, links[l].power);
	}

	return INT;
}

double SUE_path_logit(double step){
	double INT = 0.0, flow;
	int p, r, l;

	for(p=0; p<metadata.n_pair; p++){
		for(r=0; r<pairs[p].n_path; r++){
			flow = pairs[p].paths[r].flow + step*pairs[p].paths[r].direction;
			INT += flow*log(flow);
		}
	}
	INT /= metadata.lambda;

	set_direction(0.0);
	path_to_link();
	for(l=0; l<metadata.n_link; l++){
		flow = links[l].flow + step*links[l].direction;
		INT += links[l].free_time*flow;
		INT += links[l].free_time*links[l].b*flow/(links[l].power + 1)
			* pow(flow/links[l].capacity, links[l].power);
	}

	return INT;
}

double SUE_mult_logit(double step){
	double INT1 = 0.0, INT2 = 0.0, INT3 = 0.0, flow;
	int p, r, l;

	for(p=0; p<metadata.n_pair; p++){
		for(r=0; r<pairs[p].n_route; r++){
			flow = pairs[p].routes[r].flow + step*pairs[p].routes[r].direction;
			INT1 += flow*log(flow);
		}
	}
	INT1 /= metadata.theta;

	for(p=0; p<metadata.n_pair; p++){
		for(r=0; r<pairs[p].n_path; r++){
			flow = pairs[p].paths[r].flow + step*pairs[p].paths[r].direction;
			INT2 += flow*log(flow);
		}
	}
	INT2 /= metadata.lambda;
	
	for(l=0; l<metadata.n_link; l++){
		flow = links[l].flow + step*links[l].direction;
		INT3 += links[l].free_time*flow;
		INT3 += links[l].free_time*links[l].b*flow/(links[l].power + 1)
			* pow(flow/links[l].capacity, links[l].power);
	}
//	printf("int1 %lf int2 %lf int3 %lf\n", INT1, INT2, INT3);
	return INT1 + INT2 + INT3;
}
