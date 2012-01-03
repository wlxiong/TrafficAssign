#include <queue>
#include <set>
#include "data_struct.h"
#include "basic_util.h"
#include "global_var.h"
#include "show_status.h"
using namespace std;

void djikstra_all(int s){
	typedef pair<double, int> vertex;
	set<vertex> Q;
	vertex n;
	int i, j, l, t;
	double c;

	for(i=1; i<=metadata.n_node; i++){
		nodes[i].cost = INFINITE;
		nodes[i].pre = -1;
	}
	nodes[s].cost = 0;
	n.first = 0;
	n.second = s;
	Q.insert(n);
	while(~Q.empty()){
		n = *Q.begin();
		Q.erase(Q.begin());
		i = n.second;
		for(j=0; j<nodes[i].n_adj; j++){
			l = nodes[i].adj_list[j];
			c = links[l].cost + nodes[i].cost;
			t = links[l].term_node;
			if(c < nodes[t].cost){
				nodes[t].cost = c;
				nodes[t].pre = l;
				if(t >= metadata.first_node){
					if(nodes[t].cost != INFINITE)
						Q.erase(Q.find(vertex(nodes[t].cost, t)));
					Q.insert(vertex(c, t));
				}
			}
		}
	}
}

void djikstra_one(int s, int e){
	typedef pair<double, int> vertex;
	set<vertex> Q;
	vertex n;
	int i, j, l, t;
	double c;

	for(i=1; i<=metadata.n_node; i++){
		nodes[i].cost = INFINITE;
		nodes[i].pre = -1;
	}
	nodes[s].cost = 0;
	n.first = 0.0;
	n.second = s;
	Q.insert(n);
	while(~Q.empty()){
		n = *Q.begin();
		Q.erase(Q.begin());
		i = n.second;
		if (e == i){
			return;
		}
		for(j=0; j<nodes[i].n_adj; j++){
			l = nodes[i].adj_list[j];
			c = links[l].cost + nodes[i].cost;
			t = links[l].term_node;
			if(c < nodes[t].cost){
				nodes[t].cost = c;
				nodes[t].pre = l;
				if(t >= metadata.first_node){
					if(nodes[t].cost != INFINITE)
						Q.erase(Q.find(vertex(nodes[t].cost, t)));
					Q.insert(vertex(c, t));
				}
			}
		}
	}
}

void bellman_ford(int s){
	queue<int> Q;
	int n, i, l, t;
	double c;

	for(i=1; i<=metadata.n_node; i++){
		nodes[i].cost = INFINITE;
		nodes[i].pre = -1;
	}
	nodes[s].cost = 0.0;
	Q.push(s);
	while(!Q.empty()){
		n = Q.front();
		Q.pop();
		for(i=0; i<nodes[n].n_adj; i++){
			l = nodes[n].adj_list[i];
			c = links[l].cost + nodes[n].cost;
			t = links[l].term_node;
			if(c < nodes[t].cost){
				nodes[t].cost = c;
				nodes[t].pre = l;
				if(t >= metadata.first_node)
					Q.push(t);
			}
		}
	}
}

void bellman_ford_dist(int s){
	queue<int> Q;
	int n, i, l, t;
	double c;

	for(i=1; i<=metadata.n_node; i++){
		nodes[i].shortest_distant = INFINITE;
//		nodes[i].pre = -1;
	}
	nodes[s].shortest_distant = 0.0;
	Q.push(s);
	while(!Q.empty()){
		n = Q.front();
		Q.pop();
		for(i=0; i<nodes[n].n_adj; i++){
			l = nodes[n].adj_list[i];
			c = nodes[n].shortest_distant + links[l].length;
//			c = nodes[n].shortest_distant + travel_time(l);
			t = links[l].term_node;
			if(c < nodes[t].shortest_distant){
				nodes[t].shortest_distant = c;
//				nodes[t].pre = l;
				if(t >= metadata.first_node)
					Q.push(t);
			}
		}
	}
}

void bellman_ford_dist_to_go(int s){
	queue<int> Q;
	int n, i, l, t;
	double c;

	for(i=1; i<=metadata.n_node; i++){
		nodes[i].distant_to_go = INFINITE;
//		nodes[i].pre = -1;
	}
	nodes[s].distant_to_go = 0.0;
	Q.push(s);
	while(!Q.empty()){
		n = Q.front();
		Q.pop();
		for(i=0; i<nodes[n].n_rev; i++){
			l = nodes[n].rev_list[i];
			c = nodes[n].distant_to_go + links[l].length;
//			c = nodes[n].distant_to_go + travel_time(l);
			t = links[l].init_node;
			if(c < nodes[t].distant_to_go){
				nodes[t].distant_to_go = c;
//				nodes[t].pre = l;
				if(t >= metadata.first_node)
					Q.push(t);
			}
		}
	}
}

void bellman_ford_time(int s){
	queue<int> Q;
	int n, i, l, t;
	double c;

	for(i=1; i<=metadata.n_node; i++){
		nodes[i].travel_time = INFINITE;
//		nodes[i].pre = -1;
	}
	nodes[s].travel_time = 0.0;
	Q.push(s);
	while(!Q.empty()){
		n = Q.front();
		Q.pop();
		for(i=0; i<nodes[n].n_adj; i++){
			l = nodes[n].adj_list[i];
			c = nodes[n].travel_time + travel_time(l);
			t = links[l].term_node;
			if(c < nodes[t].travel_time){
				nodes[t].travel_time = c;
//				nodes[t].pre = l;
				if(t >= metadata.first_node)
					Q.push(t);
			}
		}
	}
}

void remove_label(NODE& n, int p){
	int i;

	for(i=p; i<n.n_path-1; i++){
		n.time[i] = n.time[i+1];
		n.distant[i] = n.distant[i+1];
		n.pre_link[i] = n.pre_link[i+1];
		n.pre_pos[i] = n.pre_pos[i+1];
	}
	n.n_path--;
}

void bellman_ford_constrained(int s, int e, double b){
// constrained shortest path
	typedef pair<int, int> vertex;
	queue<vertex> Q;
	vertex v, w;
	int n, i, j, l, t, p;
	double time, distant;

	for(i=1; i<=metadata.n_node; i++)
		nodes[i].n_path = 0;
	nodes[s].time[0] = 0.0;
	nodes[s].distant[0] = 0.0;
	nodes[s].pre_link[0] = -1;
	nodes[s].pre_pos[0] = -1;
	nodes[s].n_path = 1;
	v = vertex(s, 0);
	Q.push(v);
	while(!Q.empty()){
		v = Q.front();
		Q.pop();
		n = v.first;
		p = v.second;
		for(i=0; i<nodes[n].n_adj; i++){
			l = nodes[n].adj_list[i];
			time = nodes[n].time[p] + links[l].cost;
			distant = nodes[n].distant[p] + links[l].length;
//			distant = nodes[n].distant[p] + travel_time(l);
			t = links[l].term_node;
			if(distant + nodes[t].distant_to_go > b)
				continue;

			for(j=0; j<nodes[t].n_path; j++){
				if(nodes[t].time[j]>time && nodes[t].distant[j]>distant){
					remove_label(nodes[t], j);
					j--;
				}
				else if(nodes[t].time[j]<time && nodes[t].distant[j]<distant)
					break;
			}
			if(nodes[t].n_path==0 || j==nodes[t].n_path){
				if(nodes[t].n_path == MAX_LABEL)
					rep_error("Exceed max lable number", "bellman_ford_constrained()");
				nodes[t].time[nodes[t].n_path] = time;
				nodes[t].distant[nodes[t].n_path] = distant;
				nodes[t].pre_link[nodes[t].n_path] = l;
				nodes[t].pre_pos[nodes[t].n_path] = p;
				if(t >= metadata.first_node)
					Q.push(vertex(t, nodes[t].n_path));
				nodes[t].n_path++;
			}
		}
	}

	if(nodes[e].n_path == 0){
		printf(" O%d - D%d\n", s, e);
		printf(" travel time: shortest %lf, equilibrium %lf, constraint %lf\n", 
			nodes[e].travel_time, nodes[e].shortest_distant, b);
		rep_error("No path satisfies the constraint", "bellman_ford_constrained()");
	}
	p = 0;
	for(j=1; j<nodes[e].n_path; j++){
		if(nodes[e].time[j]<nodes[e].time[p])
			p = j;
	}
	
	t = e;
	while(t!=s){
		l = nodes[t].pre_link[p];
		nodes[t].pre = l;
		p = nodes[t].pre_pos[p];
		t = links[l].init_node;
	}
}

/*
double bellman_ford_arg(int s, int e, double max_length, double mu){
	queue<int> Q;
	int n, i, l, t;
	double c;

	for(i=1; i<=metadata.n_node; i++){
		nodes[i].cost_distant = INFINITE;
		nodes[i].pre = -1;
	}
	nodes[s].cost_distant = 0.0;
	nodes[s].distant = 0.0;
	Q.push(s);
	while(!Q.empty()){
		n = Q.front();
		Q.pop();
		for(i=0; i<nodes[n].n_adj; i++){
			l = nodes[n].adj_list[i];
			c = (links[l].length*mu + links[l].cost) + nodes[n].cost_distant;
			t = links[l].term_node;
			if(c < nodes[t].cost_distant){
				nodes[t].cost_distant = c;
//				nodes[t].distant = links[l].length + nodes[n].distant;
				nodes[t].pre = l;
				if(t >= metadata.first_node)
					Q.push(t);
			}
		}
	}
//	printf(" cost_distant %lf, mu %lf, max_length %lf, distant %lf\n", 
//		nodes[e].cost_distant, mu, max_length, nodes[e].distant);
	return nodes[e].cost_distant - mu*max_length;
}

double golden_section_max(int s, int e, double l, double eps, 
	double (*func)(int, int, double, double)){
    double a = 0.0, b = 1e12, _fb = -1.0;
	double xL, xR, I = b - a;
    double fa, fb;

//	for(double ii=0.0; ii<1e2; ii+=1.0){
//		printf(" SP(%.1lf): %.1lf \n", ii, -(*func)(s, e, l, ii));
//	}
//	getchar();

	fb = -(*func)(s, e, l, b);
	while(_fb<fb){
		_fb = fb;
		b = 2*b;
		fb = -(*func)(s, e, l, b);
	}


    xR = a + I*golden_rate;
    xL = b - I*golden_rate;
    fa = -(*func)(s, e, l, xL);
    fb = -(*func)(s, e, l, xR);
    do{
        if(fa >= fb){
            a = xL;
            xL = xR;
            fa = fb;
            I = b - a;
            xR = a + I*golden_rate;
            fb = -(*func)(s, e, l, xR);
        }
        else{
            b = xR;
            xR = xL;
            fb = fa;
            I = b - a;
            xL = b - I*golden_rate;
            fa = -(*func)(s, e, l, xL);
        }
    }while(xR-xL > eps);

    return (xL+xR)/2.0;
}

void lagrangian_relax(int O, int D, double tol){
// tol>=1.0, so there is always a solution. 

//	for(int i=0; i<metadata.n_link; i++){
//		printf(" #%d: flow %lf, cost %lf, length %lf\n", 
//			i+1, links[i].flow, links[i].cost, links[i].length);
//	}
//	getchar();
	bellman_ford_leng(O);
	if(nodes[D].pre == -1)
		rep_error("Network is disconnected", "lagrangian_relax()");
	golden_section_max(O, D, nodes[D].shortest_distant*tol, 
		metadata.line_search_eps, bellman_ford_arg);
}
*/

