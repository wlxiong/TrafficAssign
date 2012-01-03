#include <iostream>
#include <cstdio>
#include <string>
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
#include "mult_equ.h"
using namespace std;


void run_case(const char* case_file){
	FILE* fin;
	char line[MAX_LINE];
	
	printf(" ... RUN CASE FILE \"%s\" ...\n", case_file);
	if((fin = fopen(case_file, "r")) == NULL){
		rep_error("Can't load case file:", case_file);
	}

// ~ case_name | algorithm | line_search_eps | obj_converg_eps | flow_converg_eps | 
//   theta | lambda | stoch_part | determ_part <| toll_factor | distance_factor>
	while(getln(fin, line) != NULL){
		if(line[0] == '~')
			continue;
		sscanf(line, "%s %s %lf %lf %lf %lf %lf %lf %lf", 
			&metadata.case_name, &metadata.algo, &metadata.line_search_eps, 
			&metadata.obj_converg_eps, &metadata.flow_converg_eps, &metadata.theta, 
			&metadata.lambda, &metadata.stoch_part, &metadata.determ_part);
		printf("\n >Run case \"%s\"<\n\n", metadata.case_name);

		load_case();
		create_adj_list();
		if(!strcmp(metadata.algo, "FW")){
			metadata.objective = UE_link_obj;
			frank_wolf(metadata.flow_converg_eps);
		}
		else if(!strcmp(metadata.algo, "MSA")){
			create_rev_list();
			metadata.objective = NULL;
			msa_logit(metadata.flow_converg_eps);
		}
		else if(!strcmp(metadata.algo, "DSD")){
			metadata.objective = SUE_route_logit;
			dsd_logit(metadata.obj_converg_eps);
		}
		else if(!strcmp(metadata.algo, "MLT")){
			metadata.objective = SUE_mult_logit;
			mult_logit(metadata.obj_converg_eps);
		}
		else if(!strcmp(metadata.algo, "COL")){
			metadata.objective = SUE_route_logit;
			col_gen_logit(metadata.obj_converg_eps);
		}
		else{
			show_msg("Can't find algorithm", metadata.algo);
			continue;
		}
		save_data();
		printf("\n >End of case \"%s\"<\n", metadata.case_name);		
	}
	fclose(fin);
}


int main(int argc, char *argv[]){
	int i;
	char line[MAX_LINE];

	printf("\n *** Traffic Assignment Program is Running ***\n\n");
	if(argc <2){
		printf("? ");
		scanf("%s", line);
		run_case(line);
	}
	else{
		for(i=1; i<argc; i++)
			run_case(argv[i]);
	}

	return 0;
}
