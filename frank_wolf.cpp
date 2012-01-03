
#include "func.h"
#include "global_var.h"
#include "basic_util.h"
#include "shortest_path.h"
#include "load_data.h"
#include "save_data.h"
#include "show_status.h"
#include <iostream>
#include <cmath>
using namespace std;

void search_direction(){
	int i, Origin = -1, t, l;
	
	for(i=0; i<metadata.n_link; i++)
		links[i].direction = -links[i].flow;
	for(i=0; i<metadata.n_pair; i++){
//		cout<<i+1<<endl;
		if(pairs[i].trip == 0.0)
			continue;
		if (Origin != pairs[i].origin){
			Origin = pairs[i].origin;
			bellman_ford(Origin);
		}
		t = pairs[i].destination;
		if(nodes[t].pre == -1)
			rep_error("Network is disconnected", "search_direction()");
		while(t != Origin){
//			cout<<" t "<<t<<endl;
			l = nodes[t].pre;
			links[l].direction += pairs[i].trip;
			t = links[l].init_node;
		}
	}
}

void all_or_nothing(){
	search_direction();
	update_link_flow(1.0);
}

void frank_wolf(double criterion){
	double step, eps = INFINITE, _INT = 0.0, INT;
	char ch;

	printf("frank_wolf()\n");
	set_flow(0.0);
	update_travel_time();
	all_or_nothing();
	while(eps > criterion){
		update_travel_time();
		search_direction();
//		step = bisection(1e-12, 0.0, 1.0, UE_link_diff);
		step = golden_section(metadata.line_search_eps, 0.0, 1.0, UE_link_obj);
		eps = update_link_flow(step);
//		INT = UE_link_obj(step);
//		cout<<" step "<<step<<endl;
//		cout<<" eps "<<eps<<endl;
//		cout<<" INT"<<INT<<endl;
//		cout<<" D_INT "<<INT - _INT<<endl;
//		_INT = INT;
	}
}

/*
int main(){

//	freopen("debug\\out.txt","w",stdout);

//	load_case("SiouxFalls");
//	load_case("Braess");
//	load_case("TestNet");
	load_case("Barcelona");
	create_adj_list();

	cout<<"start frank_wolf()\n";
	frank_wolf(1e-3);
	cout<<"end frank_wolf()\n";
	
//	save_data("SiouxFalls");
//	save_data("Braess");
//	save_data("TestNet");
	save_data("Barcelona");
//	verify_assign();
	
	return 0;
}
*/
