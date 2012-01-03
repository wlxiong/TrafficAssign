#include <iostream>
#include <string>
#include <cstdio>
#include <cmath>
#include "func.h"
#include "data_struct.h"
#include "global_var.h"
#include "basic_util.h"
#include "shortest_path.h"
#include "dsd.h"
#include "column_gen.h"
#include "save_data.h"
#include "load_data.h"
#include "show_status.h"
using namespace std;

double trips[MAX_PAIR];

void load_part_trip(double percent){
	int i;
	for(i=0; i<metadata.n_pair; i++)
		pairs[i].trip = percent*trips[i];
}

void logit_mult_direction(){
	load_part_trip(metadata.stoch_part);
	logit_route_direction();
	load_part_trip(metadata.determ_part);
	logit_path_direction();
}

void init_mult_set(){
	printf("init_mult_set()\n");
	load_part_trip(metadata.stoch_part);
	init_route_set();
	load_part_trip(metadata.determ_part);
	init_path_set();
}

void logit_mult_load(){
	printf("logit_mult_load()\n");
	set_flow(0.0);
	set_route_flow(0.0);
	set_path_flow(0.0);
	update_route_cost();
	update_path_cost();
	logit_mult_direction();
//	printf("Logit Load\n");
//	for(int i=0; i<metadata.n_link; i++)
//		printf(" link  f %lf  d %lf\n", links[i].flow, links[i].direction);
//	getchar();
//	for(int i=0; i<metadata.n_pair; i++)
//		for(int j=0; j<pairs[i].n_route; j++)
//			printf(" P %d, R %d: f = %lf, d = %lf, c = %lf\n ", 
//				i+1, j+1, pairs[i].routes[j].flow, 
//				pairs[i].routes[j].direction, pairs[i].routes[j].cost);
//	for(int i=0; i<metadata.n_link; i++)
//		printf(" link  f %lf  d %lf\n", links[i].flow, links[i].direction);
//	cout<<endl;
	set_direction(0.0);
	route_to_link();
	path_to_link();
	update_route_flow(1.0);
	update_path_flow(1.0);
	update_link_flow(1.0);
//	for(int i=0; i<metadata.n_link; i++)
//		printf(" link  f %lf  d %lf\n", links[i].flow, links[i].direction);
//	getchar();
}

double master_problem_mult(double criterion){
	double eps = INFINITE, e1, e2, _INT = 0.0, INT, step;
	
//	printf("master_porblem_mult()\n");
	while(eps > criterion){
		update_route_cost();
		update_path_cost();
		logit_mult_direction();
		set_direction(0.0);
		route_to_link();
		path_to_link();
//		for(int i=1; i<100; i++)
//			printf(" obj  %lf\n", SUE_route_logit(i/100.0));
//		getchar();
//		for(int i=0; i<metadata.n_pair; i++)
//			for(int j=0; j<pairs[i].n_route; j++)
//				printf(" P %d, R %d: f = %lf, d = %lf, c = %lf\n ", 
//					i+1, j+1, pairs[i].routes[j].flow, 
//					pairs[i].routes[j].direction, pairs[i].routes[j].cost);
//		for(int i=0; i<metadata.n_link; i++)
//			printf(" link  f %lf  c %lf  d %lf\n", links[i].flow, links[i].cost, links[i].direction);
//		getchar();
		step = golden_section(metadata.line_search_eps, 0.0, 1.0, metadata.objective);
		e1 = update_route_flow(step);
		e2 = update_path_flow(step);
		eps = e1<e2? e2:e1;
//		printf("e1 %lf  e2 %lf  eps %lf  step %lf\n", e1, e2, eps, step);
//		getchar();
		update_link_flow(step);
//		INT = objective(step);
//		cout<<" step "<<step;
//		cout<<" eps "<<eps;
//		cout<<" D_INT "<<INT - _INT;
//		cout<<" INT "<<INT<<endl;
//		_INT = INT;
	}

	return metadata.objective(step);
}

bool column_gen_mult(){
	bool new_gen = false;	
//	printf("column_gen_mult()\n");
	load_part_trip(metadata.stoch_part);
	new_gen = column_gen();
	load_part_trip(metadata.determ_part);
	new_gen = column_gen_path() || new_gen;
	
	return new_gen;
}

void mult_logit(double criterion){
	int i;
	bool new_route = true;
	double step, eps = INFINITE, _INT = INFINITE, INT;

	for(i=0; i<metadata.n_pair; i++)
		trips[i] = pairs[i].trip;

	init_mult_set();
	logit_mult_load();
	printf("\nmult_logit()\n");
	while(new_route || eps > criterion){
//		for(int i=0; i<metadata.n_pair; i++)
//			printf("\t %d: %d", i+1, pairs[i].n_route);
//		getchar();
		INT = master_problem_mult(metadata.flow_converg_eps);
		new_route = column_gen_mult();
		eps = _INT - INT;
//		cout<<"\n-----\n";
		cout<<" eps "<<eps;
		cout<<" INT "<<INT<<endl;
		_INT = INT;
		printf("new route: %d\n\n", new_route);
//		for(int i=0; i<metadata.n_pair; i++)
//		for(int j=0; j<pairs[i].n_route; j++)
//			printf(" P %d, R %d: f = %lf, d = %lf, c = %lf\n ", 
//				i+1, j+1, pairs[i].routes[j].flow, 
//				pairs[i].routes[j].direction, pairs[i].routes[j].cost);
	}
//	for(int i=0; i<metadata.n_pair; i++)
//		printf("  %4d: r(%d) p(%d)  ", i+1, pairs[i].n_route, pairs[i].n_path);

	for(i=0; i<metadata.n_pair; i++)
		pairs[i].trip = trips[i];

}

/*
int main(int argc, char *argv[]){
	string case_name(argv[1]);
//	freopen("debug\\out.txt","w",stdout);

//	load_case("TestNet");
//	load_case("SiouxFalls");
//	load_case("Braess");
	load_case("Barcelona");
	create_adj_list();

	cout<<"start MLT()\n";
	objective = SUE_mult_logit;
	theta = 1.0;
	lambda = 1.0;
	determ_part = .0;
	stoch_part = 1.0 - determ_part;
	mult_logit(obj_converg_eps);
	cout<<"end MLT()\n";

	save_data("Barcelona");
//	save_data("TestNet");
//	save_data("Braess");
//	save_data("SiouxFalls");
	
	return 0;
}
*/
