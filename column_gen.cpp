#include <iostream>
#include <cstdio>
#include <cmath>
#include "func.h"
#include "data_struct.h"
#include "global_var.h"
#include "basic_util.h"
#include "shortest_path.h"
#include "dsd.h"
#include "save_data.h"
#include "load_data.h"
#include "show_status.h"
using namespace std;

bool column_gen_path(){
	int O, D, t, l, p, r, n;
	ROUTE tmp;
	bool new_path = false;

//	printf("start column_gen()\n");
//	printf("n_pair %d\n", metadata.n_pair);
	for(p=0; p<metadata.n_pair; p++){
		if(pairs[p].trip==0.0)
			continue;
//		printf(" %d/%d t(%lf)", p+1, metadata.n_pair, pairs[p].trip);
		if(O != pairs[p].origin){
			O = pairs[p].origin;
			bellman_ford(O);
		}
		D = pairs[p].destination;
//		printf("O %d  D %d(%d)\n", O, D, nodes[D].pre);
		if(nodes[D].pre == -1)
			rep_error("Network is disconnected", "column_gen_path()");
		tmp.cost = 0.0;
		tmp.flow = 0.0;
		tmp.direction = 0;
		tmp.leng = 0;
		t = D;
		while(t!=O){
			if(tmp.leng == MAX_ROUTE_LENG)
				rep_error("Exceed max route length", "column_gen_path()");
			l = nodes[t].pre;
			tmp.links[ tmp.leng++ ] = l;
			tmp.cost += links[l].cost;
			t = links[l].init_node;
		}
//		printf(" route length %d\n", tmp.leng++);
		for(r=0; r<pairs[p].n_path; r++){
			if(cmp_route(tmp, pairs[p].paths[r]))
				break;
		}
		if(r == pairs[p].n_path){
			n = pairs[p].n_path++;
			if(n == MAX_ROUTE)
				rep_error("Exceed max route number", "column_gen_path()");
			pairs[p].paths[n] = tmp;
			new_path = true;
		}
	}
//	printf("end column_gen()\n");
	printf("column_gen_path() new_path (%d)\n", new_path);
	return new_path;
}

void logit_path_direction(){
	int p, r;
	double Sw, min_cost;
	
//	printf("logit_route_direction()\n");
	for(p=0; p<metadata.n_pair; p++){
		if(pairs[p].trip==0.0)
			continue;
		min_cost = INFINITE;
		for(r=0; r<pairs[p].n_path; r++)
			min_cost = min_cost>pairs[p].paths[r].cost?
				pairs[p].paths[r].cost: min_cost;
		Sw = 0.0;
		for(r=0; r<pairs[p].n_path; r++)
			Sw += exp(-metadata.lambda*(pairs[p].paths[r].cost - min_cost));
//		printf("Sw %lf  min_cost %lf\n", Sw, min_cost);

		for(r=0; r<pairs[p].n_path; r++){
			pairs[p].paths[r].direction = 
				pairs[p].trip*exp(-metadata.lambda*(pairs[p].paths[r].cost - min_cost))/Sw
				- pairs[p].paths[r].flow;
//			printf(" P %d  R %d  exp %lf  d %lf  f %lf\n", 
//				p+1, r+1, exp(-theta*(pairs[p].routes[r].cost - min_cost)), pairs[p].routes[r].direction, pairs[p].routes[r].flow);
		}
	}
}

void init_path_set(){
	int i;
	printf("init_path_set()\n");
	for(i=0; i<metadata.n_pair; i++)
		pairs[i].n_path = 0;
	set_flow(0.0);
	set_path_flow(0.0);
	update_travel_time();
	column_gen_path();

	update_path_cost();
	logit_path_direction();
	set_direction(0.0);
	path_to_link();
	update_link_flow(1.0);
	update_travel_time();
	column_gen_path();
}

void logit_path_load(){

	printf("logit_path_load()\n");
	set_flow(0.0);
	set_path_flow(0.0);
	update_path_cost();
	logit_path_direction();
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
	path_to_link();
	update_path_flow(1.0);
	update_link_flow(1.0);
//	for(int i=0; i<metadata.n_link; i++)
//		printf(" link  f %lf  d %lf\n", links[i].flow, links[i].direction);
//	getchar();
}

double master_problem_path(double criterion){
	double eps = INFINITE, _INT = 0.0, INT, step;
	
//	printf("master_porblem()\n");
	while(eps > criterion){
		update_path_cost();
		logit_path_direction();
		set_direction(0.0);
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
		step = golden_section(1e-4, 0.0, 1.0, SUE_path_logit);
		eps = update_path_flow(step);
		update_link_flow(step);
		INT = SUE_path_logit(step);
//		cout<<" step "<<step;
//		cout<<" eps "<<eps;
//		cout<<" D_INT "<<INT - _INT;
//		cout<<" INT "<<INT<<endl;
//		_INT = INT;
	}

	return SUE_path_logit(step);
}

void col_gen_logit(double criterion){
	double step, eps = INFINITE, _INT = INFINITE, INT;
	char ch;
	bool new_path = true;

	init_path_set();
	logit_path_load();
	printf("\ncol_gen_logit()\n");
	while(new_path || eps > criterion){
//		for(int i=0; i<metadata.n_pair; i++)
//			printf("\t %d: %d", i+1, pairs[i].n_route);
//		getchar();
		INT = master_problem_path(metadata.flow_converg_eps);
		new_path = column_gen_path();
		eps = _INT - INT;
//		cout<<"\n-----\n";
		cout<<" eps "<<eps;
		cout<<" INT "<<INT<<endl;
		_INT = INT;
		printf("new path: %d\n\n", new_path);
//		for(int i=0; i<metadata.n_pair; i++)
//		for(int j=0; j<pairs[i].n_route; j++)
//			printf(" P %d, R %d: f = %lf, d = %lf, c = %lf\n ", 
//				i+1, j+1, pairs[i].routes[j].flow, 
//				pairs[i].routes[j].direction, pairs[i].routes[j].cost);
	}
	for(int i=0; i<metadata.n_pair; i++)
		printf("  %4d: p(%d)  ", i+1, pairs[i].n_path);
}
/*
int main(){

//	freopen("debug\\out.txt","w",stdout);

//	load_case("TestNet");
	load_case("SiouxFalls");
//	load_case("Braess");
//	load_case("Barcelona");
	create_adj_list();
	create_rev_list();

	cout<<"start DSD()\n";
	lambda = 2.0;
	col_gen_logit(obj_converg_eps);
	cout<<"end DSD()\n";

//	cout<<"\nVerify assignment\n";
//	verify_assign();
//	save_data("Barcelona");
//	save_data("TestNet");
//	save_data("Braess");
	save_data("SiouxFalls");
	
	return 0;
}
*/
