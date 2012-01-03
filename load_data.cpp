#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "data_struct.h"
#include "show_status.h"
#include "global_var.h"
using namespace std;

void print_link(FILE* fout, const LINK& link){
	fprintf(fout, "%4d  %4d  %lf  %lf  %lf  %lf  %lf  %lf  %lf  %d\n", 
		link.init_node, link.term_node, link.capacity, 
		link.length, link.free_time, link.b, link.power, 
		link.speed_limit, link.toll, link.type);
}

void print_trip(FILE* fout, const PAIR& pair){
	fprintf(fout, "O %-4d  D %-4d  %lf\n", 
		pair.origin, pair.destination, pair.trip);
}

void verify_data(const char* verify_file){
	FILE* fout;
	int i; 
	
	show_msg("Verify data:", verify_file);
	if((fout = fopen(verify_file, "w")) == NULL)
		rep_error("Can't open input file \"%s\"\n", verify_file);
	
	fprintf(fout, "<METADATA>\n"); 
	fprintf(fout, "<Number of Zones> %d\n", metadata.n_zone);
	fprintf(fout, "<Number of Nodes> %d\n", metadata.n_node); 
	fprintf(fout, "<Number of Links> %d\n", metadata.n_link);
	fprintf(fout, "<Number of Pairs> %d\n", metadata.n_pair);
	fprintf(fout, "<First Through Node> %d\n", metadata.first_node);
	fprintf(fout, "<Total Flows> %lf\n", metadata.total_flow);
	
	fprintf(fout, "\n<LINKS>\n");
	fprintf(fout, "~ Init node | Term node | Capacity | Length | "); 
	fprintf(fout, "Free Flow Time | B | Power | Speed limit | Toll | Type\n");
	for(i=0; i<metadata.n_link; i++){
		print_link(fout, links[i]);
	}
	
	fprintf(fout, "\n<TRIPS>\n");
	for(i=0; i<metadata.n_pair; i++){
		print_trip(fout, pairs[i]); 
	}
	
	fclose(fout);
}

void* getln(FILE* fin, char* ln){
	char ch;
	while(isspace(ch = getc(fin)));
	ungetc(ch,fin);
	return fgets(ln, MAX_LINE, fin);
}

void load_net(const char* net_file){
	FILE* fin; 
	char line[MAX_LINE],field[MAX_LINE], data[MAX_LINE], *p;
	LINK tmp; 
	int n_link = 0; 
	
	show_msg("Load net:", net_file);
	if((fin = fopen(net_file, "r")) == NULL)
		rep_error("Can't open input file ", net_file);
	
	while(getln(fin, line) != NULL){
		if(line[0] == '~')
			continue;
		if(line[0] == '<'){
			for(p = line; *p != '\0'; p++){
				if(*p == '>'){
					*p = '\0';
					break;
				}
			}
			strcpy(field, line+1);
			strcpy(data, p+1);
			if(strcmp(field, "NUMBER OF ZONES") == 0)
				metadata.n_zone = atoi(data);
			else if(strcmp(field, "NUMBER OF NODES") == 0)
				metadata.n_node = atoi(data);
			else if(strcmp(field, "FIRST THRU NODE") == 0)
				metadata.first_node = atoi(data);
			else if(strcmp(field, "NUMBER OF LINKS") == 0)
				metadata.n_link = atoi(data);
			else if(strcmp(field, "END OF METADATA") == 0)
				break;
			else
				rep_error("Unknown metadata title: ", field); 
		}
	}
	
	while(getln(fin, line) && n_link < metadata.n_link){
		if(line[0] == '~')
			continue;
		sscanf(line, "%d %d %lf %lf %lf %lf %lf %lf %lf %d", 
			&tmp.init_node, &tmp.term_node, &tmp.capacity, &tmp.length, 
			&tmp. free_time, &tmp.b, &tmp.power, &tmp.speed_limit, 
			&tmp.toll, &tmp.type); 
		links[n_link++] = tmp; 
		if(n_link == MAX_LINK)
			rep_error("Exceed max link number", "load_net()");
//		disp_link(tmp);
	}
	fclose(fin);
}

void load_trip(const char* trip_file){
	FILE *fin; 
	char line[MAX_LINE], field[MAX_LINE], data[MAX_LINE], *p, ch;
	PAIR tmp; 
	int n_pair = 0; 
	
	show_msg("Load trip:", trip_file);
	if((fin = fopen(trip_file, "r")) == NULL)
		rep_error("Can't open input file ", trip_file);
	
	while(getln(fin, line) != NULL){
		if(line[0] == '~')
			continue;
		if(line[0] == '<'){
			for(p = line; *p != '\0'; p++){
				if(*p == '>'){
					*p = '\0';
					break;
				}
			}
			strcpy(field, line+1);
			strcpy(data, p+1);
			if(strcmp(field, "TOTAL OD FLOW") == 0)
				metadata.total_flow = atof(data);
			else	if(strcmp(field, "NUMBER OF ZONES") == 0)
				metadata.n_zone = atoi(data);
			else if(strcmp(field, "END OF METADATA") == 0)
				break;
			else
				rep_error("Unknown metadata title: ", field); 
		}
	}
	
	while(getln(fin, line)){
		if(line[0] == '~')
			continue;
		if(line[0] == 'O')
			sscanf(line, "Origin %d", &tmp.origin);
		else{
			istringstream lin(line); 
			while(lin>>tmp.destination>>ch>>tmp.trip>>ch){
				pairs[n_pair++] = tmp;
//				disp_trip(tmp);
				if(n_pair == MAX_PAIR)
					rep_error("Exceed max pair number", "load_trip()");
			}
		}
	}
	metadata.n_pair = n_pair;
	fclose(fin);
}

bool load_node(const char* node_file){
	FILE* fin;
	char line[MAX_LINE];
	int n, x, y;
	
	show_msg("Load node:", node_file);
	if((fin = fopen(node_file, "r")) == NULL){
		show_msg("Can't load node coordinate file:", node_file);
		return false;
	}

	getln(fin, line);
	while(getln(fin, line) != NULL){
		sscanf(line, "%d %d %d ;", &n, &x, &y);
		nodes[n].x = x;
		nodes[n].y = y;
		if(n == MAX_NODE)
			rep_error("Exceed max node number", "load_node()");
	}
	fclose(fin);
	return true;
}

void load_case(){
	char net_file[MAX_LINE], trips_file[MAX_LINE], data_file[MAX_LINE];
	sprintf(net_file, "./%s/%s_net.txt", metadata.case_name, metadata.case_name);
	sprintf(trips_file, "./%s/%s_trips.txt", metadata.case_name, metadata.case_name);
	sprintf(data_file, "./%s/%s_data.txt", metadata.case_name, metadata.case_name);

	show_msg("Load case:", metadata.case_name);
	load_net(net_file);
	load_trip(trips_file);
	verify_data(data_file);
}
