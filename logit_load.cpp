#include "global_var.h"
#include "basic_util.h"
#include "load_data.h"
#include "save_data.h"
#include <iostream>
#include <cstdio>
#include <algorithm>
#include <map>
#include <queue>
#include <cmath>
using namespace std;

void origin_to_node(int o){
	queue<int> Q;
	int n, i, l, t;
	double c;

	for(i=1; i<=metadata.n_node; i++)
		nodes[i].r = INFINITE;
	nodes[o].r = 0.0;
	Q.push(o);
	while(!Q.empty()){
		n = Q.front();
		Q.pop();
		for(i=0; i<nodes[n].n_adj; i++){
			l = nodes[n].adj_list[i];
			c = links[l].cost + nodes[n].r;
			t = links[l].term_node;
			if(t >= metadata.first_node && c < nodes[t].r){
				nodes[t].r = c;
				Q.push(t);
			}
		}
	}
}

void destination_to_node(int d){
	queue<int> Q;
	int n, i, l, t;
	double c;

	for(i=1; i<=metadata.n_node; i++)
		nodes[i].s = INFINITE;
	nodes[d].s = 0.0;
	Q.push(d);
	while(!Q.empty()){
		n = Q.front();
		Q.pop();
		for(i=0; i<nodes[n].n_rev; i++){
			l = nodes[n].rev_list[i];
			c = links[l].cost + nodes[n].s;
			t = links[l].init_node;
			if(t >= metadata.first_node && c < nodes[t].s){
				nodes[t].s = c;
				Q.push(t);
			}
		}
	}
}

void dial_direction(){
	int i, j, k, O = -1, D = -1, s, t, l, n;
	double Dr, Ds, Sw, Sx;
	pair<double, int> order[MAX_NODE];

//	freopen("debug\\out.txt","w",stdout);
//	printf("start dial_direction()\n");
	for(i=0; i<metadata.n_link; i++)
		links[i].direction = -links[i].flow;
	for(i=0; i<metadata.n_pair; i++){
//		printf("\t%5d/%5d", i+1, metadata.n_pair);
		if(pairs[i].trip == 0.0)
			continue;
		if (O != pairs[i].origin){
			O = pairs[i].origin;
			origin_to_node(O);
		}
		D = pairs[i].destination;
		destination_to_node(D);
//		printf("\nforward %lf  backward %lf\n", nodes[D].r, nodes[O].s);
//		printf("O %d  D %d\n", O, D);
// compute L(i,j)
		for(j=0; j<metadata.n_link; j++){
			s = links[j].init_node;
			t = links[j].term_node;
			Dr = nodes[t].r - nodes[s].r;
			Ds = nodes[t].s - nodes[s].s;
//			printf(" D_r(%d,%d) %lf  D_s(%d,%d) %lf  ", s, t, Dr, s, t, Ds);
			if(Dr>0.0 && Ds<0.0)
				links[j].L = exp(metadata.theta*(Dr - links[j].cost));
			else
				links[j].L = 0.0;
//			printf(" L(%d,%d) %lf\n ", s, t, links[j].L);
			links[j].w = 0.0;
			links[j].x = 0.0;
		}
// forward pass
//		printf("forward pass\n");
		for(j=1; j<=metadata.n_node; j++)
			order[j-1] = pair<double, int>(nodes[j].r,j);
		sort(order, order+metadata.n_node);
//		printf("O %d \n", order[0].second);
		// when j == O
		for(k=0; k<nodes[O].n_adj; k++){
			l = nodes[O].adj_list[k];
			links[l].w = links[l].L;
//			printf(" w(%d,%d) %lf  ", links[l].init_node, links[l].term_node, links[l].w);
		}
		// when j != O
		for(j=1; j<metadata.n_node; j++){
			n = order[j].second;
			Sw = 0.0;
			for(k=0; k<nodes[n].n_rev; k++){
				l = nodes[n].rev_list[k];
				Sw += links[l].w;
			}
//			printf("\n node %d Sum w %lf \n", n, Sw);
			for(k=0; k<nodes[n].n_adj; k++){
				l = nodes[n].adj_list[k];
				links[l].w = links[l].L*Sw;
//				printf(" w(%d,%d) %lf  ", links[l].init_node, links[l].term_node, links[l].w);
			}
		}
// backward pass
//		printf("backward pass\n");
		for(j=1; j<=metadata.n_node; j++)
			order[j-1] = pair<double, int>(nodes[j].s,j);
		sort(order, order+metadata.n_node);
//		printf(" D %d\n", order[0].second);
		Sw = 0.0;
		// when j == D
		for(k=0; k<nodes[D].n_rev; k++){
			l = nodes[D].rev_list[k];
			Sw += links[l].w;
		}
//		printf(" node %d Sum w %lf  Sw==0.0? %d\n", D, Sw, Sw==0.0);
		for(k=0; k<nodes[D].n_rev; k++){
			l = nodes[D].rev_list[k];
			links[l].x = (Sw==0.0? 0:pairs[i].trip*links[l].w/Sw);
//			printf(" x(%d,%d) %lf\n", links[l].init_node, links[l].term_node, links[l].x);
		}
		// when j != D
		for(j=1; j<metadata.n_node; j++){
			n = order[j].second;
			Sx = 0.0;
			Sw = 0.0;
			for(k=0; k<nodes[n].n_adj; k++){
				l = nodes[n].adj_list[k];
				Sx += links[l].x;
			}
			for(k=0; k<nodes[n].n_rev; k++){
				l = nodes[n].rev_list[k];
				Sw += links[l].w;
			}
//			printf(" node %d Sum w %lf  Sw==0.0? %d\n", D, Sw, Sw==0.0);
			for(k=0; k<nodes[n].n_rev; k++){
				l = nodes[n].rev_list[k];
				links[l].x = (Sw==0.0? 0:Sx*links[l].w/Sw);
//				printf(" x(%d,%d) %lf\n", links[l].init_node, links[l].term_node, links[l].x);
			}
		}
// update direction
//		printf("update direction\n");
		for(j=0; j<metadata.n_link; j++)
			links[j].direction += links[j].x;
	}
}

void logit_load(){
	dial_direction();
//	for(int i = 0; i<metadata.n_link; i++)
//		printf("d[%2d] = %lf\n", i+1, links[i].direction);
	update_link_flow(1.0);
//	for(int i = 0; i<metadata.n_link; i++)
//		printf("f[%2d] = %lf\n", i+1, links[i].flow);
}
