/*==============================================================================
	ERIC NAULI SIHITE
	Made for a coding challenge, 11/30/2017
==============================================================================*/
#ifndef ASTAR_H
#define ASTAR_H

// Included dependencies
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
using namespace std;

// Class Definition
class AStar	{
// Private members
	// Struct declaration
	struct map_struct {
		vector<vector<int>> map;	// Map data, must be rectangle
		int nx;						// Map size	x (column)
		int ny;						// Map size y (row)
		vector<int> s;				// Start position (row,col)
		vector<int> t;				// Target position (row,col)
		bool hammer;				// May use hammer
	};
	map_struct map;

	struct node {
		vector<int> pos;			// Node coordinate
		vector<vector<int>> path;	// Path from start to finish
		float f;
		float g;
		float h;
		bool hammer_used;
	};

	// Constant variables
	const float REALLY_BIG_NUMBER = 1e9;	// A really big number

	// Function list
	int astar_algorithm(map_struct map, vector<vector<int>>* sol, float* f);
	int read_input_file(string file_name, map_struct* map);
	float distance(vector<int> p1, vector<int> p2);
	float distance_euclidean(vector<int> p1, vector<int> p2);
	void create_node(node* n_new, node n_old, vector<int> p, \
		map_struct map, bool use_hammer);
	void print_solution(map_struct map, vector<vector<int>> sol);
	int generate_open_nodes(vector<node>* open_n, vector<node>* openh_n,\
		node n, vector<vector<int>>* node_map, \
		vector<vector<int>>* wall_map, map_struct map);

	// Debug function list
	void print_map_data(map_struct map);
	void print_node_data(vector<vector<int>> map);

public:
	// Variable list
	vector<vector<int>> solution;	// Solution path
	float cost = 0;					// Solution cost
	
	// Function list
	int astar_solve(string file_name, bool hammer);
	int test();
};

#endif // ASTAR_H
