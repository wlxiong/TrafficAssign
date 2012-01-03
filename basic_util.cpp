#include <iostream>
#include <cmath>
#include "data_struct.h"
#include "global_var.h"
#include "frank_wolf.h"
#include "func.h"
using namespace std;

void set_flow(double f){
	int i;
	for(i=0; i<metadata.n_link; i++)
		links[i].flow = f;
}

void set_direction(double d){
	int i;
	for(i=0; i<metadata.n_link; i++){
		links[i].direction = d;
	}
}

void set_route_flow(double f){
	int p, r;
	for(p=0; p<metadata.n_pair; p++)
		for(r=0; r<pairs[p].n_route; r++)
			pairs[p].routes[r].flow = f;
}

void set_path_flow(double f){
	int p, r;
	for(p=0; p<metadata.n_pair; p++)
		for(r=0; r<pairs[p].n_path; r++)
			pairs[p].paths[r].flow = f;
}

void set_route_direction(double d){
	int p, r;
	for(p=0; p<metadata.n_pair; p++)
		for(r=0; r<pairs[p].n_route; r++)
			pairs[p].routes[r].direction = d;
}

void set_path_direction(double d){
	int p, r;
	for(p=0; p<metadata.n_pair; p++)
		for(r=0; r<pairs[p].n_path; r++)
			pairs[p].paths[r].direction = d;
}

void init_link_length(){
	int i;
	printf("init_link_length()\n");
	frank_wolf(metadata.flow_converg_eps);
	for(i=0; i<metadata.n_link; i++)
		links[i].length = links[i].cost;
}

double travel_time(int i){
	return links[i].free_time * 
		(1 + links[i].b * pow(links[i].flow/links[i].capacity, links[i].power)); 
}

void update_travel_time(){
	int i;
	for(i=0; i<metadata.n_link; i++)
		links[i].cost = links[i].free_time
			*(1 + links[i].b * pow(links[i].flow/links[i].capacity, links[i].power)); 
}

void update_marginal_cost(){
	int i;
	for(i=0; i<metadata.n_link; i++){
		links[i].cost = links[i].free_time * links[i].b * links[i].power 
			* pow(links[i].flow/links[i].capacity, links[i].power); 
		links[i].cost += links[i].free_time
			*(1 + links[i].b * pow(links[i].flow/links[i].capacity, links[i].power));
	}
}

void update_general_cost(){
	int i;
	for(i=0; i<metadata.n_link; i++)
		links[i].cost = links[i].cost + 
			metadata.toll_factor * links[i].toll + metadata.distance_factor * links[i].length;
}

void update_route_cost(){
	int p, r, l, i;

	update_travel_time();
	for(p=0; p<metadata.n_pair; p++){
		for(r=0; r<pairs[p].n_route; r++){
			pairs[p].routes[r].cost = 0.0;
			for(i=0; i<pairs[p].routes[r].leng; i++){
				l = pairs[p].routes[r].links[i];
				pairs[p].routes[r].cost += links[l].cost;
			}
		}
	}
}

void update_path_cost(){
	int p, r, l, i;

	update_marginal_cost();
	for(p=0; p<metadata.n_pair; p++){
		for(r=0; r<pairs[p].n_path; r++){
			pairs[p].paths[r].cost = 0.0;
			for(i=0; i<pairs[p].paths[r].leng; i++){
				l = pairs[p].paths[r].links[i];
				pairs[p].paths[r].cost += links[l].cost;
			}
		}
	}
}

void route_to_link_direction(){
	int i, j, k, l;
	for(i=0; i<metadata.n_pair; i++){
		for(j=0; j<pairs[i].n_route; j++){
			for(k=0; k<pairs[i].routes[j].leng; k++){
				l = pairs[i].routes[j].links[k];
				links[l].direction += pairs[i].routes[j].direction;
			}
		}
	}
}

void route_to_link_flow(){
	int i, j, k, l;
	for(i=0; i<metadata.n_pair; i++){
		for(j=0; j<pairs[i].n_route; j++){
			for(k=0; k<pairs[i].routes[j].leng; k++){
				l = pairs[i].routes[j].links[k];
				links[l].flow += pairs[i].routes[j].flow;
			}
		}
	}
}

void path_to_link_direction(){
	int i, j, k, l;
	for(i=0; i<metadata.n_pair; i++){
		for(j=0; j<pairs[i].n_path; j++){
			for(k=0; k<pairs[i].paths[j].leng; k++){
				l = pairs[i].paths[j].links[k];
				links[l].direction += pairs[i].paths[j].direction;
			}
		}
	}
}

void path_to_link_flow(){
	int i, j, k, l;
	for(i=0; i<metadata.n_pair; i++){
		for(j=0; j<pairs[i].n_path; j++){
			for(k=0; k<pairs[i].paths[j].leng; k++){
				l = pairs[i].paths[j].links[k];
				links[l].flow += pairs[i].paths[j].flow;
			}
		}
	}
}

double update_link_flow(double step){
	int i;
	double sum_flow = 0.0, sqr_flow = 0.0, last_flow;

	for(i=0; i<metadata.n_link; i++){
		sum_flow += links[i].flow;
		last_flow = links[i].flow;
		links[i].flow += step * links[i].direction;
		sqr_flow += (links[i].flow - last_flow)*(links[i].flow - last_flow);
	}
	if(sum_flow == 0.0)
		return 0.0;
	return sqrt(sqr_flow)/sum_flow;
}

double update_route_flow(double step){
	int i, j, l;
	double sum_flow = 0.0, sqr_flow = 0.0, last_flow;

	for(i=0; i<metadata.n_pair; i++){
		for(j=0; j<pairs[i].n_route; j++){
			last_flow = pairs[i].routes[j].flow;
			sum_flow += pairs[i].routes[j].flow;
			pairs[i].routes[j].flow += step*pairs[i].routes[j].direction;
			sqr_flow += (pairs[i].routes[j].flow - last_flow)
				*(pairs[i].routes[j].flow - last_flow);
		}
	}
	if(sum_flow == 0.0)
		return 0.0;
	return sqrt(sqr_flow)/sum_flow;
}

double update_path_flow(double step){
	int i, j, l;
	double sum_flow = 0.0, sqr_flow = 0.0, last_flow;

	for(i=0; i<metadata.n_pair; i++){
		for(j=0; j<pairs[i].n_path; j++){
			last_flow = pairs[i].paths[j].flow;
			sum_flow += pairs[i].paths[j].flow;
			pairs[i].paths[j].flow += step*pairs[i].paths[j].direction;
			sqr_flow += (pairs[i].paths[j].flow - last_flow)
				*(pairs[i].paths[j].flow - last_flow);
		}
	}
	if(sum_flow == 0.0)
		return 0.0;
	return sqrt(sqr_flow)/sum_flow;
}

void create_adj_list(){
	int i, n;
	for(i=1; i<=metadata.n_node; i++)
		nodes[i].n_adj = 0;
	for(i=0; i<metadata.n_link; i++){
		n = links[i].init_node; 
		nodes[n].adj_list[ nodes[n].n_adj++ ] = i; 
	}
}

void create_rev_list(){
	int i, n;
	for(i=1; i<=metadata.n_node; i++)
		nodes[i].n_rev = 0;
	for(i=0; i<metadata.n_link; i++){
		n = links[i].term_node; 
		nodes[n].rev_list[ nodes[n].n_rev++ ] = i; 
	}
}

double bisection(double eps, double a, double b, double (*diff)(double)){
    double mid;
    double fa, fb, fm;
    int i=1;

    fa = diff(a);
    fb = diff(b);
    mid = (a+b)/2.0;
    do{
        i++;
        fm = diff(mid);
        if(fa*fm < 0.0){
            b = mid;
            fb = diff(b);
        }
        if(fb*fm <= 0.0){
            a = mid;
            fa = diff(a);
        }
        mid = (a+b)/2.0;
//		cout<<" [a b] "<<a<<' '<<b<<endl;
    }while(b-a > eps);

    return (a+b)/2.0;
}

double golden_section(double eps, double a, double b, double (*func)(double)){
    double xL, xR, I = b - a;
    double fa, fb;
    int i = 1;

    xR = a + I*golden_rate;
    xL = b - I*golden_rate;
    fa = (*func)(xL);
    fb = (*func)(xR);
    do{
        i++;
        if(fa >= fb){
            a = xL;
            xL = xR;
            fa = fb;
            I = b - a;
            xR = a + I*golden_rate;
            fb = (*func)(xR);
        }
        else{
            b = xR;
            xR = xL;
            fb = fa;
            I = b - a;
            xL = b - I*golden_rate;
            fa = (*func)(xL);
        }
    }while(xR -xL > eps);

    return (xL+xR)/2.0;
}
