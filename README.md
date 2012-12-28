Summary
=======

This project built a prototype of an advanced traveler information system 
hose objective is to reduce system cost with recurrent network congestion 
through provision of route guidance to travelers.
The design of route guidance system faces a well-known dilemma. 
The approach that theoretically yields the _system-optimal_ traffic pattern 
may _discriminate_ against some users in favor of others. 

We proposed a novel mathematical programming model to resolve this dilemma. 
The route choice behavior is modeled as a convex minimization problem and 
an efficient column generation algorithm is used to solve this mixed behavior 
equilibrium problem.

The essence of this study is that system-optimal routing of traffic flow with 
explicit integration of user constraints leads to a better performance than 
stochastic user equilibrium, while _simultaneously_ guaranteeing superior 
fairness compared to the pure system optimum.

This research is part of my undergraduate thesis in Chinese, 信息环境下的混合网络均衡模型 
([PDF](https://dl.dropbox.com/u/4415273/thesis-swjtu/thesis_4th.pdf), 
[HTML](https://dl.dropbox.com/u/4415273/thesis-swjtu/thesis_2st.htm)).


How to run the program
======================
- Run make in the source directory to compile the program:

        make

- A binary file named _assign_ is created. It can be run as follows: 

        assign script1 script2 script3 ...

- The program will process all the traffic assignment problems
specified in files: script1, script2, script3 ...


How to write a script file
==========================

- "run_script.txt" in this directory is a simple example to start with. 
- In a script file, any line starts with "~" is ignored. 
- Each line specifies a traffic assignment problem. 
- There are 8 columns in each line: 
	- case_name: the name of the folder containing the network data;
	- algorithm: traffic assignment algorithms:
		- FW: Frank-Wolf
		- COL: column generation
		- MSA: method of successive average
		- DSD: disaggregated simplicial decomposition
		- MIX: mixed stochastic user equilibrium (SUE) and system optimal (SO)
	- line_search_eps: convergence criterion of linear search
	- obj_converg_eps: convergence criterion of objective function
	- flow_converg_eps: convergence criterion of link flows
	- (Note that the computation process terminates if and only if the results meet all the three criteria.)
	- theta: parameter of perception error of SUE
	- stoch_part: proportion of travelers with SUE behavior
	- distant_tol: tolerance of travel distance multiplier (for travelers acting in accordance with SO principle)
- A sample script file is shown as follows. 

        ~ Columns: case_name | algorithm | line_search_eps | obj_converg_eps | flow_converg_eps | theta | stoch_part | distant_tol
        TestNet		    FW	    1e-8	1e-4	1e-4	0.2	    1.0	    0.5
        TestNet		    FW	    1e-8	1e-4	1e-4	0.2	    1.0	    0.5
        SiouxFalls	    FW	    1e-8	1e-4	1e-4	0.02	0.1     2.0
        SiouxFalls	    COL	    1e-8	1e-4	1e-4	0.02	0.1     2.0
        SiouxFalls	    DSD	    1e-8	1e-4	1e-4	0.02	0.1     2.0
        SiouxFalls	    MIX	    1e-8	1e-4	1e-4	0.02	0.1     1.05
        SiouxFalls	    MIX	    1e-8	1e-4	1e-4	0.02	0.1     1.1
