#include <queue>
#include <set>
#include "data_struct.h"
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
//		cout<<" n "<<n<<endl;
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

void bellman_ford_leng(int s){
	queue<int> Q;
	int n, i, l, t;
	double c;

	for(i=1; i<=metadata.n_node; i++){
		nodes[i].shortest_distant = INFINITE;
		nodes[i].pre = -1;
	}
	nodes[s].shortest_distant = 0.0;
	Q.push(s);
	while(!Q.empty()){
		n = Q.front();
//		cout<<" n "<<n<<endl;
		Q.pop();
		for(i=0; i<nodes[n].n_adj; i++){
			l = nodes[n].adj_list[i];
			c = links[l].length + nodes[n].shortest_distant;
			t = links[l].term_node;
			if(c < nodes[t].shortest_distant){
				nodes[t].shortest_distant = c;
				nodes[t].pre = l;
				if(t >= metadata.first_node)
					Q.push(t);
			}
		}
	}
}

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
//		cout<<" n "<<n<<endl;
		Q.pop();
		for(i=0; i<nodes[n].n_adj; i++){
			l = nodes[n].adj_list[i];
			c = (links[l].length*mu + links[l].cost) + nodes[n].cost_distant;
			t = links[l].term_node;
			if(c < nodes[t].cost_distant){
				nodes[t].cost_distant = c;
				nodes[t].distant = links[l].length + nodes[n].distant;
				nodes[t].pre = l;
				if(t >= metadata.first_node)
					Q.push(t);
			}
		}
	}
	
	return nodes[e].cost_distant - mu*max_length;
}

double golden_section_max(int s, int e, double l, double eps, double (*func)(int, int, double, double)){
    double a = 0.0, b = 1.0, _fb = INFINITE;
	double xL, xR, I = b - a;
    double fa, fb;

	fb = -(*func)(s, e, l, b);
	while(_fb>fb){
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

void lagrangian_relax(int O, int D){
	if(nodes[D].shortest_distant == INFINITE)
		rep_error("Network is disconnected", "lagrangian_reflax()");
	golden_section_max(O, D, nodes[D].shortest_distant*metadata.distant_tol, 
		metadata.line_search_eps, bellman_ford_arg);
}
