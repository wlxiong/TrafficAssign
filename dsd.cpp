#include <iostream>
#include <cstdio>
#include <cmath>
#include "func.h"
#include "data_struct.h"
#include "global_var.h"
#include "basic_util.h"
#include "shortest_path.h"
#include "save_data.h"
#include "load_data.h"
#include "show_status.h"
using namespace std;

bool cmp_route(ROUTE& a, ROUTE& b){
	int i, leng;
	
	if(a.leng != b.leng)
		return false;
	else
		leng = a.leng;
	for(i=0; i<leng; i++)
		if(a.links[i] != b.links[i])
			return false;

	return true;
}

bool generate_route(){
	int O = -1, D, t, l, p, r, n;
	ROUTE tmp;
	bool new_route = false;

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
			rep_error("Network is disconnected", "column_gen()");
		tmp.cost = 0.0;
		tmp.flow = 0.0;
		tmp.direction = 0;
		tmp.leng = 0;
		t = D;
		while(t!=O){
			if(tmp.leng == MAX_ROUTE_LENG)
				rep_error("Exceed max route length", "column_gen()");
			l = nodes[t].pre;
			tmp.links[ tmp.leng++ ] = l;
			tmp.cost += links[l].cost;
			t = links[l].init_node;
		}
//		printf(" route length %d\n", tmp.leng++);
		for(r=0; r<pairs[p].n_route; r++){
			if(cmp_route(tmp, pairs[p].routes[r]))
				break;
		}
		if(r == pairs[p].n_route){
			n = pairs[p].n_route++;
			if(n == MAX_ROUTE)
				rep_error("Exceed max route number", "column_gen()");
			pairs[p].routes[n] = tmp;
			new_route = true;
		}
	}
//	printf("end column_gen()\n");
	printf("column_gen() new_route (%d)\n", new_route);
	return new_route;
}

void logit_route_direction(){
	int p, r;
	double Sw, min_cost;
	
//	printf("logit_route_direction()\n");
	for(p=0; p<metadata.n_pair; p++){
		if(pairs[p].trip==0.0)
			continue;
		min_cost = INFINITE;
		for(r=0; r<pairs[p].n_route; r++)
			min_cost = min_cost>pairs[p].routes[r].cost?
				pairs[p].routes[r].cost: min_cost;
		Sw = 0.0;
		for(r=0; r<pairs[p].n_route; r++)
			Sw += exp(-metadata.theta*(pairs[p].routes[r].cost - min_cost));
//		printf("Sw %lf  min_cost %lf\n", Sw, min_cost);

		for(r=0; r<pairs[p].n_route; r++){
			pairs[p].routes[r].direction = 
				pairs[p].trip*exp(-metadata.theta*(pairs[p].routes[r].cost - min_cost))/Sw
				- pairs[p].routes[r].flow;
//			printf(" P %d  R %d  exp %lf  d %lf  f %lf\n", 
//				p+1, r+1, exp(-theta*(pairs[p].routes[r].cost - min_cost)), pairs[p].routes[r].direction, pairs[p].routes[r].flow);
		}
	}
}

void init_route_set(){
	int i;
	printf("init_route_set()\n");
	for(i=0; i<metadata.n_pair; i++)
		pairs[i].n_route = 0;
	set_flow(0.0);
	set_route_flow(0.0);
	update_travel_time();
	generate_route();

	update_travel_time();
	update_route_cost();
	logit_route_direction();
	set_direction(0.0);
	route_to_link();
	update_link_flow(1.0);
	update_travel_time();
	generate_route();
}

void init_route_flow(){

	printf("init_route_flow()\n");
	set_flow(0.0);
	set_route_flow(0.0);
	update_travel_time();
	update_route_cost();
	logit_route_direction();

	set_direction(0.0);
	route_to_link();
	update_route_flow(1.0);
	update_link_flow(1.0);
}

double master_problem(double criterion){
	double eps = INFINITE, _INT = 0.0, INT, step;
	
//	printf("master_porblem()\n");
	while(eps > criterion){
		update_travel_time();
		update_route_cost();
		logit_route_direction();
		set_direction(0.0);
		route_to_link();
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
		eps = update_route_flow(step);
		update_link_flow(step);
		INT = SUE_route_logit(step);
//		cout<<" step "<<step;
//		cout<<" eps "<<eps;
//		cout<<" D_INT "<<INT - _INT;
//		cout<<" INT "<<INT<<endl;
//		_INT = INT;
	}

	return metadata.objective(step);
}

void dsd_logit(double criterion){
	double step, eps = INFINITE, _INT = INFINITE, INT;
	char ch;
	bool new_route = true;

	init_route_set();
	init_route_flow();
	printf("\ndsd_logit()\n");
	while(new_route || eps > criterion){
//		for(int i=0; i<metadata.n_pair; i++)
//			printf("\t %d: %d", i+1, pairs[i].n_route);
//		getchar();
		INT = master_problem(metadata.flow_converg_eps);
		new_route = generate_route();
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
//		printf("  %4d: r(%d)  ", i+1, pairs[i].n_route);
}

/*
int main(){

//	freopen("debug\\out.txt","w",stdout);

//	load_case("TestNet");
	load_case("SiouxFalls");
//	load_case("Braess");
//	load_case("Barcelona");
	create_adj_list();
//	create_rev_list();

	cout<<"start DSD()\n";
	objective = SUE_mult_logit;
	theta = 1.0;
	dsd_logit(obj_converg_eps);
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
