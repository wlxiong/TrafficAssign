#include <iostream>
#include <string>
#include <cstdio>
#include <cstring>
#include <ctime>
#include "global_var.h"
#include "data_struct.h"
#include "basic_util.h"
#include "load_data.h"
#include "save_data.h"
#include "show_status.h"
#include "func.h"
#include "msa.h"
#include "dsd.h"
#include "column_gen.h"
#include "frank_wolf.h"
#include "mixed_equ.h"
using namespace std;

bool traffc_assign(){
	if(!strcmp(metadata.algo, "FW")){
		metadata.objective = UE_link_obj;
		frank_wolf(metadata.flow_converg_eps);
	}
	else if(!strcmp(metadata.algo, "COL")){
		metadata.objective = SO_link_obj;
		column_FW(metadata.obj_converg_eps);
	}
	else if(!strcmp(metadata.algo, "MSA")){
		metadata.objective = NULL;
		msa_logit(metadata.flow_converg_eps);
	}
	else if(!strcmp(metadata.algo, "DSD")){
		metadata.objective = SUE_route_logit;
		dsd_logit(metadata.obj_converg_eps);
	}
	else if(!strcmp(metadata.algo, "MIX")){
		metadata.objective = SUE_SO_mixed;
		mixed_equilibrium(metadata.obj_converg_eps);
	}
	else{
		show_msg("Can't find algorithm", metadata.algo);
		return false;
	}
	return true;
}

void save_ans(FILE *fout){
	int i, j;
	static int n_task = 0;
	double mean_route_cost = 0.0, mean_path_cost = 0.0;
	double  mean_n_path = 0.0, mean_n_route = 0.0;

	fprintf(fout, "#%2d %s %s ", ++n_task, metadata.case_name, metadata.algo);
	fprintf(fout, "%.2lf %.2lf %.2lf ", metadata.theta, metadata.determ_part, metadata.distant_tol);
	fprintf(fout, "%.2lf %.2lf ", metadata.objective(0.0), SO_link_obj(0.0));
	
	for(i=0; i<metadata.n_pair; i++){
		mean_n_path += pairs[i].n_path;
		mean_n_route += pairs[i].n_route;
		for(j=0; j<pairs[i].n_path; j++)
			mean_path_cost += pairs[i].paths[j].cost;
		for(j=0; j<pairs[i].n_route; j++)
			mean_route_cost += pairs[i].routes[j].cost;
	}
	if(mean_n_path == 0.0)
		mean_path_cost = 0.0;
	else
		mean_path_cost /= mean_n_path;
	if(mean_n_route == 0.0)
		mean_route_cost = 0.0;
	else
		mean_route_cost /= mean_n_route;
	mean_n_path /= metadata.n_pair;
	mean_n_route /= metadata.n_pair;

	fprintf(fout, "%.3lf %.3lf %.3lf %.3lf\n", 
		mean_n_path, mean_path_cost, mean_n_route, mean_route_cost);
}

void run_case(const char* case_file){
	FILE *fin, *fout;
	char line[MAX_LINE];
	char stat_file[MAX_LINE];
	
	printf(" ... RUN CASE FILE \"%s\" ...\n", case_file);
	if((fin = fopen(case_file, "r")) == NULL){
		rep_error("Can't load case file:", case_file);
	}
	strcpy(stat_file, case_file);
	strcat(stat_file, ".ans");
	if((fout = fopen(stat_file, "a")) == NULL){
		rep_error("Can't open answer file:", stat_file);
	}
	fprintf(fout, "\n no | name | algo | theta | market | tolerance | objective | cost | ");
	fprintf(fout, "mean_n_path | mean_path_cost | mean_n_route | mean_route_cost\n");


// case_name | algorithm | line_search_eps | obj_converg_eps | flow_converg_eps | theta | stoch_part | distant_tol
	while(getln(fin, line) != NULL){
		if(line[0] == '~')
			continue;
		sscanf(line, "%s %s %lf %lf %lf %lf %lf %lf", 
			metadata.case_name, metadata.algo, &metadata.line_search_eps, 
			&metadata.obj_converg_eps, &metadata.flow_converg_eps, 
			&metadata.theta, &metadata.determ_part, &metadata.distant_tol);

		printf("\n >Run case \"%s\"<\n\n", metadata.case_name);
		load_case();
		create_adj_list();
		create_rev_list();
		traffc_assign();
		save_data();
		save_ans(fout);
		printf("\n >End of case \"%s\"<\n", metadata.case_name);
	}
	fclose(fin);
	fclose(fout);
}

int main(int argc, char *argv[]){
	int i;
	char line[MAX_LINE];

	if(argc <2){
        printf("usage: assign script1 script2 script3 ...\n\n");
        printf("  The program will process all the traffic assignment problems\n");
        printf("  specified in files: script1, script2, script3...\n");
        printf("  \"run_script.txt\" in this directory is a simple example to start with. \n\n");
		printf("Path of the run script: ");
		scanf("%s", line);
		run_case(line);
	}
	else{
		for(i=1; i<argc; i++)
			run_case(argv[i]);
	}

	return 0;
}
