#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H

// max input line length
#define MAX_LINE 512
// max paths of an OD pair
#define MAX_ROUTE 64
// max path length
#define MAX_ROUTE_LENG 128
// max degree of a node
#define MAX_DEG 32

struct LINK{
	int init_node, term_node, type;
	double capacity, length, free_time, b, power, speed_limit, toll; 
	double flow, cost, direction;
	double L, w, x;
};

struct META{
	int n_zone, n_node, n_link, n_pair, first_node;
	double total_flow, toll_factor, distance_factor;
	double theta, lambda, determ_part, stoch_part, 
		line_search_eps, obj_converg_eps, flow_converg_eps;
	double (*objective)(double);
	char case_name[MAX_LINE], algo[MAX_LINE];
};

struct ROUTE{
	int links[MAX_ROUTE_LENG], leng;
	double cost, flow, direction;
};

struct PAIR{
	int origin, destination, n_route, n_path;
	double trip;
	ROUTE routes[MAX_ROUTE], paths[MAX_ROUTE];
};

struct NODE{
	int node_id, n_adj, n_rev, pre, x, y;
	int adj_list[MAX_DEG], rev_list[MAX_DEG];
	double cost, r, s;
	int visited;
};

#endif
