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

//	printf("start column_gen_path()\n");
//	printf("n_pair %d\n", metadata.n_pair);
	for(p=0; p<metadata.n_pair; p++){
		if(pairs[p].trip==0.0)
			continue;
		O = pairs[p].origin;
		D = pairs[p].destination;
		bellman_ford_dist(O);
		bellman_ford_dist_to_go(D);
		bellman_ford_constrained(O, D, metadata.distant_tol*nodes[D].shortest_distant);
//		printf(" O %d, D %d \n", O, D);
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

void search_path_direction(){
	int p, r, k_min;
	double Sw, min_cost;
	
//	printf("search_path_direction()\n");

	for(p=0; p<metadata.n_pair; p++){
		if(pairs[p].trip==0.0)
			continue;
		k_min = 0;
		for(r=1; r<pairs[p].n_path; r++)
			k_min = pairs[p].paths[r].cost<pairs[p].paths[k_min].cost? r: k_min;
		for(r=0; r<pairs[p].n_path; r++)
			pairs[p].paths[r].direction = - pairs[p].paths[r].flow;
		pairs[p].paths[k_min].direction += pairs[p].trip;
	}
}

void init_path_set(){
	int i;
	printf("init_path_set()\n");
	for(i=0; i<metadata.n_pair; i++)
		pairs[i].n_path = 0;
	set_flow(0.0);
	set_path_flow(0.0);
	update_marginal_cost();
	column_gen_path();

//	update_path_cost();
//	search_path_direction();
//	set_direction(0.0);
//	path_to_link_direction();
//	update_link_flow(1.0);
//	update_marginal_cost();
//	column_gen_path();
}

void init_path_flow(){

	printf("init_path_flow()\n");
	set_flow(0.0);
	set_path_flow(0.0);
	update_path_cost();
	search_path_direction();
	set_direction(0.0);
	path_to_link_direction();
	update_path_flow(1.0);
	update_link_flow(1.0);
}

double master_problem_path(double criterion){
	double eps = INFINITE, _INT = 0.0, INT, step;
	
//	printf("master_porblem_path()\n");
	while(eps > criterion){
		update_path_cost();
		search_path_direction();
		set_direction(0.0);
		path_to_link_direction();

		step = golden_section(metadata.line_search_eps, 0.0, 1.0, SO_link_obj);
		eps = update_path_flow(step);
		update_link_flow(step);
//		printf("step %lf, eps %lf\n", step, eps);
//		getchar();
	}

	return SO_link_obj(step);
}

void column_FW(double criterion){
	double step, eps = INFINITE, _INT = INFINITE, INT;
	char ch;
	bool new_path = true;

//	init_link_length();
	init_path_set();
	init_path_flow();
	printf("\ncolumn_FW()\n");
	while(new_path || eps > criterion){
		INT = master_problem_path(metadata.flow_converg_eps);
		new_path = column_gen_path();
		eps = (_INT - INT)/INT;
		printf(" eps %e, _INT %e, INT %e\n", eps, _INT, INT);
//		printf("new path: %d\n\n", new_path);
		_INT = INT;
	}
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
