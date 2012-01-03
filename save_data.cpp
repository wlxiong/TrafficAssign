#include <cstdio>
#include <iostream>
#include <string>
#include "func.h"
#include "basic_util.h"
#include "load_data.h"
#include "show_status.h"
#include "global_var.h"
using namespace std;

void print_flow(const char* ps_file){
	FILE* fout;
	double max_flow = 0.0;
	int i, s, t;

	show_msg("Print flow:", ps_file);
	if((fout = fopen(ps_file, "w")) == NULL)
		rep_error("Can't open output file ", ps_file);

	fprintf(fout, "%%!\n");
	fprintf(fout, "%% Traffic Assignment Output\n");
	fprintf(fout, "%% %s \n\n", ps_file);
	
	for(i=0; i<metadata.n_link; i++){
		max_flow = max_flow > links[i].flow? max_flow:links[i].flow;
	}
	for(i=0; i<metadata.n_link; i++){
		s = links[i].init_node;
		t = links[i].term_node;
		fprintf(fout, "gsave\n");
		fprintf(fout, "  %.3lf setgray\n", 1-(links[i].flow/max_flow));
		fprintf(fout, "  %.0lf setlinewidth\n", links[i].flow/max_flow*10);
		fprintf(fout, "  %d %d moveto %d %d lineto stroke\n", 
			nodes[s].x/1000, nodes[s].y/1000, nodes[t].x/1000, nodes[t].y/1000);
		fprintf(fout, "grestore\n\n");
	}
	fprintf(fout, "0 setgray\n");
	fprintf(fout, "1 setlinewidth\n");
	for(i=1; i<=metadata.n_node; i++){
		fprintf(fout, "%d %d 5 0 360 arc fill\n", nodes[i].x/1000, nodes[i].y/1000);
	}
}

void save_flow(const char* flow_file){
	FILE* fout;
	int i;

	show_msg("Save flows:", flow_file);
	if((fout = fopen(flow_file, "w")) == NULL)
		rep_error("Can't open output file ", flow_file);
	
	fprintf(fout, "<METADATA>\n"); 
	fprintf(fout, "<File Name> %s\n", flow_file);
	fprintf(fout, "<Number of Links> %d\n", metadata.n_link);
	fprintf(fout, "<theta | lambda> %lf | %lf\n", metadata.theta, metadata.lambda);
	fprintf(fout, "<stoch | determ> %lf | %lf\n", metadata.stoch_part, metadata.determ_part);
	fprintf(fout, "<distant tolerance> %lf\n", metadata.distant_tol);
	fprintf(fout, "<optimal objective> %lf\n", metadata.objective(0.0));
	fprintf(fout, "<system cost> %lf\n", SO_link_obj(0.0));

	fprintf(fout, "\n<LINKS>\n");
	fprintf(fout, "~ Init node | Term node | Flow | Cost\n"); 
	for(i=0; i<metadata.n_link; i++){
		fprintf(fout, "%4d  %4d  %lf  %lf\n", 
			links[i].init_node, links[i].term_node, links[i].flow, links[i].cost);
	}
}

void save_route(const char* route_file){
	FILE* fout;
	int i, p, r;
	double mean_cost = 0.0, total_flow = 0.0, trip = 0.0;

	show_msg("Save routes:", route_file);
	if((fout = fopen(route_file, "w")) == NULL)
		rep_error("Can't open output file ", route_file);

	fprintf(fout, "<METADATA>\n"); 
	fprintf(fout, "<File Name> %s\n", route_file);
	fprintf(fout, "<Number of Pairs> %d\n", metadata.n_pair);
	fprintf(fout, "<Number of Zones> %d\n", metadata.n_zone);
	fprintf(fout, "<theta | lambda> %lf | %lf\n", metadata.theta, metadata.lambda);
	fprintf(fout, "<stoch | determ> %lf | %lf\n", metadata.stoch_part, metadata.determ_part);
	fprintf(fout, "<distant tolerance> %lf\n", metadata.distant_tol);
	fprintf(fout, "<optimal objective> %lf\n", metadata.objective(0.0));
	fprintf(fout, "<system cost> %lf\n", SO_link_obj(0.0));

	fprintf(fout, "\n<ROUTES>\n<PATHS>\n");
	fprintf(fout, "~ Origin | Destination | Flow | Cost\n");
	for(p=0; p<metadata.n_pair; p++){
		// ouput route info
		if(pairs[p].n_route != 0){
			trip = 0.0;
			mean_cost = 0.0;
			for(r=0; r<pairs[p].n_route; r++){
				trip += pairs[p].routes[r].flow;
				mean_cost += pairs[p].routes[r].cost*pairs[p].routes[r].flow;
				fprintf(fout, "   <R%d>    %.3lf    %.3lf\n", 
					r+1, pairs[p].routes[r].flow, pairs[p].routes[r].cost);
			}
			fprintf(fout, "OD(%d,%d) route_flow/trip %.3lf, mean_cost %.3lf\n", 
				pairs[p].origin, pairs[p].destination, 
				trip/pairs[p].trip, mean_cost/trip);
			total_flow += trip;
		}
		// ouput path info
		if(pairs[p].n_path != 0){
			trip = 0.0;
			mean_cost = 0.0;
			for(r=0; r<pairs[p].n_path; r++){
				trip += pairs[p].paths[r].flow;
				mean_cost += pairs[p].paths[r].cost*pairs[p].paths[r].flow;
				fprintf(fout, "   <P%d>    %.3lf    %.3lf\n", 
					r+1, pairs[p].paths[r].flow, pairs[p].paths[r].cost);
			}
			fprintf(fout, "OD(%d,%d) path_flow/trip %.3lf, mean_cost %.3lf\n", 
				pairs[p].origin, pairs[p].destination, 
				trip/pairs[p].trip, mean_cost/trip);
			total_flow += trip;
		}
		fputs("\n", fout);
	}
	
	fprintf(fout, "\n<Input Total Flows> %lf\n", metadata.total_flow);
	fprintf(fout, "<Output Total Flows> %lf\n", total_flow);
}

void save_data(){
	char case_file_name[MAX_LINE], ps_file[MAX_LINE], node_file[MAX_LINE], 
		flow_file[MAX_LINE], route_file[MAX_LINE];
	sprintf(case_file_name, "%s_%s_%.2lf_%.2lf_%.2lf_%.2lf_%.2lf", 
		metadata.case_name, metadata.algo, metadata.theta, metadata.lambda, 
		metadata.stoch_part, metadata.determ_part, metadata.distant_tol);
	sprintf(ps_file, "./%s/%s_flow.ps", metadata.case_name, case_file_name);
	sprintf(flow_file, "./%s/%s_flow.txt", metadata.case_name, case_file_name);
	sprintf(route_file, "./%s/%s_route.txt", metadata.case_name, case_file_name);
	sprintf(node_file, "./%s/%s_node.txt", metadata.case_name, metadata.case_name);

	show_msg("Save data:", metadata.case_name);
	save_flow(flow_file);
	save_route(route_file);
	show_msg("Save link flow file:", flow_file);
	if(load_node(node_file)){
		print_flow(ps_file);
		show_msg("Print link flow file:", ps_file);
	}
}

double find_path_flow(int o, int d, int s, double f){
	double min_x = f, x, path_f;
	int i, n, t, l;

//	printf("o %d  d %d s  %d  f  %lf\n", o, d, s, f);
	nodes[s].visited = 1;
	if(s == d)
		return min_x;
	for(i=0; i<nodes[s].n_adj; i++){
		l = nodes[s].adj_list[i];
		x = links[l].direction;
		t = links[l].term_node;
		if(!(nodes[t].visited) && x>0.0){
			min_x = x<f? x:f;
			path_f = find_path_flow(o, d, t, min_x);
			if(path_f > 0.0){
				links[l].direction -= path_f;
				return path_f;
			}
		}
	}
	return 0.0;
}

double find_cycle_flow(int o, int s, double f){
	double min_x = f, x, path_f;
	int i, n, t, l;
	
	printf("o %d  s %d   f %lf\n", o, s, f);
	nodes[s].visited = 1;
	for(i=0; i<nodes[s].n_adj; i++){
		l = nodes[s].adj_list[i];
		x = links[l].direction;
		t = links[l].term_node;
		if(!(nodes[t].visited) && x>0.0){
			min_x = x<f? x:f;
			path_f = find_cycle_flow(o, t, min_x);
			if(path_f > 0.001){
				links[l].direction -= path_f;
				return path_f;
			}
		}
	}
	if(s == o)
		return min_x;
	return 0.0;
}

void verify_assign(){
	int i, j, O, D;
	double F, _F;

	for(i=0; i<metadata.n_link; i++)
		links[i].direction = links[i].flow;
	for(i=0; i<metadata.n_pair; i++){
		O = pairs[i].origin;
		D = pairs[i].destination;
		F = pairs[i].trip;
		_F = F;
		while(F > 0.001){
//			getchar();
			for(j=1; j<=metadata.n_node; j++)
				nodes[j].visited = 0;
			F -= find_path_flow(O, D, O, F);
			if(_F-F == 0.0)
				break;
			_F = F;
		}
		if(F>0.0)
			printf("O %2d D %2d F %lf\n", O, D, F);
	}
	for(i=0; i<metadata.n_pair; i++){
		O = pairs[i].origin;
		find_cycle_flow(O, O, 100000.0);
	}
	for(i=0; i<metadata.n_link; i++){
		if(links[i].direction == 0.0)
			links[i].flow = 0.0;
//		if(links[i].direction>0.0)
//			printf("f[%2d] = %lf\n", i+1, links[i].direction);
	}
}
