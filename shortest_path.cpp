#include <queue>
#include <set>
#include "data_struct.h"
#include "global_var.h"
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
