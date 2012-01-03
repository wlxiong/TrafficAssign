
#include "global_var.h"
#include "basic_util.h"
#include "load_data.h"
#include "save_data.h"
#include "logit_load.h"
#include <iostream>
#include <cstdio>
#include <cmath>
using namespace std;


void msa_logit(double criterion){
	double eps = INFINITE, n = 0.0;

	set_flow(0.0);
	update_travel_time();
	logit_load();
	while(eps > criterion){
//		cin>>ch;
		update_travel_time();
		dial_direction();
//		for(int i = 0; i<metadata.n_link; i++)
//			cout<<"direction "<<i<<' '<<links[i].direction<<endl;
//		for(int i = 0; i<metadata.n_link; i++)
//			cout<<"flow "<<i<<' '<<links[i].flow<<endl;
//		cout<<" step 1/"<<n+1<<endl;
		eps = update_link_flow(1.0/++n);
		cout<<" eps "<<eps<<endl;
	}
}

/*
int main(){

//	freopen("debug\\out.txt","w",stdout);

//	load_case("TestNet");
//	load_case("Braess");
	load_case("Barcelona");
	create_adj_list();
	create_rev_list();

	cout<<"start MSA()\n";
	theta = 4.0;
	MSA(1e-4);
//	logit_load();
	cout<<"end MSA()\n";

	cout<<"\nVerify assignment\n";
//	verify_assign();
//	save_data("TestNet");
//	save_data("Braess");
	save_data("Barcelona");
	
	return 0;
}
*/
