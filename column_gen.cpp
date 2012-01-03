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

bool generate_path(int p){
	int O, D, t, l, r, n;
	ROUTE tmp;
	bool new_path = false;

//	printf("start column_gen()\n");
	if(pairs[p].trip==0.0)
		return new_path;
//	printf(" %d/%d t(%lf)", p+1, metadata.n_pair, pairs[p].trip);
	O = pairs[p].origin;
	D = pairs[p].destination;
	bellman_ford_leng(O);
	lagrangian_relax(O, D);
		
//	printf("O %d  D %d(%d)\n", O, D, nodes[D].pre);
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
//	printf(" route length %d\n", tmp.leng++);
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
	printf("column_gen_path() new_path (%d)\n", new_path);
	return new_path;
}

double search_path_direction(int p){
	int i, k_max = 0, k_min = 0;
	double d_max, d_min;
//	printf("search_path_direction()\n");
	
	if(pairs[p].trip = 0.0)
		return 0.0;
	if(pairs[p].n_path == 1){
		set_path_direction(0.0);
		pairs[p].paths[0].direction = pairs[p].trip;
		path_to_link(p, 0, pairs[p].trip);
		return 1.0;
	}
	for(i=1; i<pairs[p].n_path; i++){
		if(pairs[p].paths[i].cost<pairs[p].paths[k_min].cost)
			k_min = i;
		if(pairs[p].paths[i].cost>pairs[p].paths[k_max].cost)
			k_max = i;
	}
	if(pairs[p].paths[k_max].cost-pairs[p].paths[k_min].cost < metadata.cost_converg_eps)
		return 0.0;

	d_max = pairs[p].paths[k_min].flow - pairs[p].paths[k_max].flow;
	d_min = -d_max;
	pairs[p].paths[k_max].direction = d_max;
	pairs[p].paths[k_min].direction = d_min;
	path_to_link(p, k_max, d_max);
	path_to_link(p, k_min, d_min);

	return -pairs[p].paths[k_max].flow/pairs[p].paths[k_max].direction;
}

double column_subproblem(int pair){
	double eps = INFINITE, _INT = 0.0, INT, step, ub;
	bool new_path = true;
//	printf("master_porblem()\n");

	if(pairs[pair].trip = 0.0)
		return true;
	update_marginal_cost();
	update_path_cost();
	set_direction(0.0);
	set_path_direction(0.0);
	ub = search_path_direction(pair);
	if(ub = 0.0)
		return true;

	remove_single_flow(pair);
	pairs[pair].n_path = 0;
	generate_path(pair);
	update_path_flow(1.0);
	set_single_flow(0.0);
	update_single_flow(1.0);
	while(new_path){
		update_marginal_cost();
		update_path_cost();
		set_direction(0.0);
		set_path_direction(0.0);
		ub = search_path_direction(pair);
		step = golden_section(metadata.line_search_eps, 0.0, ub, metadata.objective);
		update_path_flow(step);
		update_single_flow(step);
		new_path = generate_path(pair);
	}

	return false;
}

void init_path_set(){
	int i;
	
	printf("init_path_set()\n");
	for(i=0; i<metadata.n_pair; i++)
		pairs[i].n_path = 0;

	set_flow(0.0);
	update_marginal_cost();
	for(i=0; i<metadata.n_pair; i++)
		generate_path(i);
}

void init_path_flow(){
	int i;
	printf("init_path_flow()\n");
	set_direction(0.0);
	set_path_direction(0.0);
	for(i=0; i<metadata.n_pair; i++)
		search_path_direction(i);

	update_link_flow(1.0);
	update_path_flow(1.0);
}

void column_gen(){
	double step, eps = INFINITE, _INT = INFINITE, INT;
	char ch;
	int i, l;
	bool new_path = true;

	printf("\ncolumn_gen()\n");
	init_path_set();
	init_path_flow();

	for(i=0, l=0; l<metadata.n_pair; i++){
		if(pairs[i].trip = 0.0)
			continue;
		i = i%metadata.n_pair;
		if(column_subproblem(i))
			l++;
		else
			l = 0;
	}

	for(i=0; i<metadata.n_pair; i++)
		printf("  %4d: p(%d)  ", i+1, pairs[i].n_path);
}

/*
int main(){

//	freopen("debug\\out.txt","w",stdout);

//	load_case("TestNet");
	load_case();
//	load_case("Braess");
//	load_case("Barcelona");
	create_adj_list();
	create_rev_list();

	cout<<"start COL()\n";
	column_gen(metadata.obj_converg_eps);
	cout<<"end COL()\n";

//	cout<<"\nVerify assignment\n";
//	verify_assign();
//	save_data("Barcelona");
//	save_data("TestNet");
//	save_data("Braess");
	save_data();
	
	return 0;
}
*/
