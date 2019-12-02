/*==============================================================================
	ERIC NAULI SIHITE
	Made for a coding challenge, 11/30/2017
==============================================================================*/
#include "astar.h"

/*==============================================================================
	PUBLIC FUNCTIONS
==============================================================================*/
int AStar::astar_solve(string file_name, bool hammer) {
	// Main function. Solve for the shortest path to the target location.
	
	if (read_input_file(file_name, &map)) {
		// Map data read successful		
		map.hammer = hammer;	// Record the hammer setting	
		print_map_data(map);	// Debug, print out map data
		
		// Begin the A* algorithm
		if (astar_algorithm(map, &solution, &cost)) {
			// Solution is found, print out results
			cout << "Solution found. Cost: " << cost << endl;
			print_solution(map, solution);	// Print solution
			return 1;	// Success
		}
		else {
			// Solution not found. Print the best path
			cout << "Solution not found. No path to the target." << endl;
			print_solution(map, solution);	// Print solution
			return 0;	// Failed
		}
	}
	else {
		return 0;		// Failed
	}			 	
}

int AStar::test() {
	cout << endl  << "TESTING FROM ASTAR" << endl;
	vector<int> pika;
	pika = { 1,2 };
	cout << pika[0] << " " << pika[1] << endl;

	pika = { 10,20 };
	cout << pika[0] << " " << pika[1] << endl;

	return 0;
}


/*==============================================================================
	PRIVATE FUNCTIONS
==============================================================================*/

int AStar::astar_algorithm(map_struct map, vector<vector<int>>* sol, float* f) {
	// The A* algorithm. Returns the shortest path to the target position.
	// Returns 1 if solution found, 0 if not.

	// Initialize the A* algorithm
	bool solution_found = false;
	bool finished = false;
	vector<node> open_n; // open node list
	vector<node> openh_n; // after hammer open node list

	// Open node and cost map. 1 = open node, 0 = closed
	vector<vector<int>> open_map, node_map, wall_map, nodeh_map;
	for (int i = 0; i < map.ny; i++) {		// Row
		vector<int> temp1;
		vector<int> temp2;
		for (int j = 0; j < map.nx; j++) {	// Col
			temp1.push_back(1);	// All open nodes
			if (map.map[i][j] == 1) {
				temp2.push_back(1);	// Wall only
			}
			else {
				temp2.push_back(0);	// Wall only
			}
		}
		node_map.push_back(temp1);
		wall_map.push_back(temp2);
	}
	open_map = node_map;
	nodeh_map = node_map;

	// Initial node
	node n, n_best;	// Temporary node
	n.pos = map.s;
	n.path.push_back(map.s);
	n.g = 0;
	n.h = distance_euclidean(n.pos, map.t);
	n.f = n.g + n.h;
	n.hammer_used = false;
	open_n.push_back(n);
	node_map[n.pos[0]][n.pos[1]] = 0;
	n_best = n;
	float f_best = n.f;
	bool hammer_finished = false;

	// Begin recursion
	cout << "Begin the A* algorithm." << endl << endl;
	int count = 0;
	bool hammer_start = false;
	while (!finished) {
		// Find the node with the lowest f from the open_n list
		unsigned int m = 0;
		float f_min = open_n[0].f;
		for (unsigned int i = 1; i < open_n.size(); i++) {
			if (f_min >= open_n[i].f) {
				m = i;
				f_min = open_n[i].f;
			}
		}

		// Record the node and erase it from the open node list
		n = open_n[m];						// Record node
		open_n.erase(open_n.begin() + m);	// Erase the node from the list

		if (map.map[n.pos[0]][n.pos[1]] == 3) {
			// Found the target
			if (!solution_found) {
				// First time
				n_best = n;
				f_best = n.f;
				solution_found = true;
			}
			else {
				if (n.f < f_best) {
					n_best = n;
					f_best = n.f;
				}
			}
		}
		else {
			node_map[n.pos[0]][n.pos[1]] = 0;	// Close the node
		}

		// Form neighboring nodes
		if (generate_open_nodes(&open_n, &openh_n, n, \
			&node_map, &wall_map, map)) {
			// We just used hammer, solve regularly with A*
			// Find the node with the lowest f from the open_n list
			hammer_finished = false;
			nodeh_map = open_map;
			cout << "Used hammer" << endl;
			while (!hammer_finished) {	  				
				// 2nd layer while loop
				unsigned int a = 0;
				float fh_min = openh_n[0].f;
				for (unsigned int i = 1; i < openh_n.size(); i++) {
					if (fh_min >= openh_n[i].f) {
						a = i;
						fh_min = openh_n[i].f;
					}
				}

				// Record the node and erase it from the open node list
				n = openh_n[a];						// Record node
				openh_n.erase(openh_n.begin() + a);	// Erase the node from the list

				if (map.map[n.pos[0]][n.pos[1]] == 3) {
					// Found the target
					if (!solution_found) {
						// First time
						n_best = n;
						f_best = n.f;
						solution_found = true;
					}
					else {
						if (n.f < f_best) {
							n_best = n;
							f_best = n.f;
						}
					}
				}
				else {
					nodeh_map[n.pos[0]][n.pos[1]] = 0;	// Close the node
				}

				generate_open_nodes(&openh_n, &openh_n, n, \
					&nodeh_map, &wall_map, map);

				if (openh_n.size() == 0) {
					hammer_finished = true; // Termination condition
				}
			}
		}

		// Debug: stream out some data
		cout << "Loop # " << count << ", open node size: ";
		cout << open_n.size() << ", f: " << f_min;
		cout << ", g: " << n.g << endl;

		if (open_n.size() == 0) {
			finished = true; // Termination condition
		}
		count++;
	}

	*sol = n_best.path;
	*f = n_best.f;

	cout << endl;
	//print_node_data(node_map); // debug

	// If solution is found, return 1
	if (solution_found)	return 1;
	else return 0;
}

int AStar::generate_open_nodes(vector<node>* open_n, vector<node>* openh_n,\
	node n, vector<vector<int>>* node_map,\
	vector<vector<int>>* wall_map, map_struct map) {
	// Create viable neighboring nodes of the node n, and put it in the open_n.

	node nt; // Temporary node
	int hammer_used = 0;

	// All 8 neighbors, many ifs
	for (int i = n.pos[0] - 1; i <= n.pos[0] + 1; i++) {
		for (int j = n.pos[1] - 1; j <= n.pos[1] + 1; j++) {
			vector<int> p = { i,j };

			// Check if the index is inside the map and not the previous node
			if (i >= 0 && j >= 0 && i < map.ny && j < map.nx \
				&& !(i == n.pos[0] && j == n.pos[1])) {

				// Check for walls
				if (map.map[i][j] == 1) {
					// Is a wall
					// If we can use hammer, use it
					if (map.hammer) {
						// Allows hammer
						create_node(&nt, n, p, map, 1); // Create temp. node
						if (!(n.hammer_used)) {
							// Haven't used hammer
							if ((*wall_map)[i][j] == 1) {
								
								// Node open, use hammer if f is lower								
								// Put into a separate node list
								
								//cout << "asdfasdf" << endl;
								openh_n->push_back(nt);
								(*wall_map)[i][j] = 0;
								hammer_used = 1;
																							
							}
							else {
								// Node closed.

								//if (nt.f < n.f) {
									//open_n->push_back(nt);
									//(*node_map)[i][j] = 1;
									//cout << "hammer time closed" << endl;
							}
						}
					}
					else {
						// No hammer, close the node
						(*node_map)[i][j] = 0;
					}
				}
				else {
					// Not a wall. Check if it's open
					create_node(&nt, n, p, map, 0); // Create temp. node
					if ((*node_map)[i][j] == 1) {
						// Node open, check if it's already in the open_n.
						bool in_open = false;
						for (unsigned int k = 0; k < open_n->size(); k++) {
							if (i == (*open_n)[k].pos[0] && j == (*open_n)[k].pos[1]) {
								// Node is in the list.
								in_open = true;
							}
						}
						// Not in the list, so add it
						if (!in_open) {
							open_n->push_back(nt);
							//cout << "new open in" << endl;
						}
					}
					else {
						// Node closed
						// If the node has lower f, add into the list
						if (nt.f < n.f) {
							// f is lower, add to the list
							open_n->push_back(nt);
							//(*node_map)[i][j] = 1;
							//cout << "closed in" << endl;
						}
					}
				}
			}

		}
	}

	return hammer_used;
}



void AStar::create_node(node* n_new, node n_old, vector<int> p, \
	map_struct map, bool use_hammer) {
	// Create a new node.
	n_new->pos = p;
	n_new->path = n_old.path;
	n_new->path.push_back(p);
	n_new->g = n_old.g + distance(p,n_old.pos);
	n_new->h = distance_euclidean(p,map.t);
	n_new->f = n_new->g + n_new->h;
	
	// Hammer version only
	if (n_old.hammer_used) {
		// Hammer already used
		n_new->hammer_used = true;
	}
	else {
		// Hammer has not been used
		if (use_hammer) {
			n_new->hammer_used = true;	// Use hammer
		}
		else {
			n_new->hammer_used = false;	// Don't use hammer
		}

	}
}

int AStar::read_input_file(string file_name, map_struct* map) {
	// Read the text file and generate the map data.
	// Return 1 if successful, 0  if not.

	ifstream file(file_name);	// Open file

	// Parse data
	if (file) {
		cout << "Map file was opened successfully." << endl;

		// Some sanity check
		bool no_start = true;
		bool no_target = true;

		// stringstream setup
		stringstream ss;
		ss << file.rdbuf();	// Put the text file in the string stream
		file.close();		// Close the file
		char c;

		// Read map size
		ss >> map->ny >> map->nx;

		// Read map data		
		int count = 0;
		vector<int> xy;	// Coordinate (row,col)
		for (int i = 0; i < map->ny; i++) {
			vector<int> temp;
			for (int j = 0; j < map->nx; j++) {
				ss >> c;
				if (c == 'S') {
					if (no_start) {
						temp.push_back(2);		// 2 = start
						xy = { i,j };
						map->s = xy;
						no_start = false;
					}
					else {
						cout << "ERROR: more than 1 start position." << endl;
						return 0;
					}
				}
				else if (c == 'T') {
					if (no_target) {
						temp.push_back(3);		// 3 = target
						xy = { i,j };
						map->t = xy;
						no_target = false;
					}
					else {
						cout << "ERROR: more than 1 target position." << endl;
						return 0;
					}
				}
				else if (c == '1') {
					temp.push_back(1);		// 1 = wall
				}
				else if (c == '0') {
					temp.push_back(0);		// 0 = path
				}
				else {
					cout << "ERROR: unexpected input: " << c << endl;
					return 0;
				}
				count++;
			}
			map->map.push_back(temp);
		}


		if (no_start) {
			cout << "ERROR: no starting position." << endl;
			return 0;
		}
		else if (no_target) {
			cout << "ERROR: no target position." << endl;
			return 0;
		}
		else if (count != (map->nx*map->ny)) {
			cout << "ERROR: map dimension mismatch." << endl;
			return 0;
		}

		cout << "Read was completed successfuly." << endl;
		return 1;	// Read successful
	}
	else {
		cout << "ERROR: read failure." << endl;
		return 0;	// Read failed
	}
}

float AStar::distance(vector<int> p1, vector<int> p2) {
	// Calculate the minimum distance between 2 positions.
	float output;

	int dy = abs(p1[0] - p2[0]);
	int dx = abs(p1[1] - p2[1]);

	if (dx <= dy) {
		output = dx*sqrtf(2) + (dy-dx);
	}
	else {
		output = dy*sqrtf(2) + (dx-dy);
	}

	return output;
}

float AStar::distance_euclidean(vector<int> p1, vector<int> p2) {
	// Calculate the minimum distance between 2 positions.
	int dy = abs(p1[0] - p2[0]);
	int dx = abs(p1[1] - p2[1]);

	return sqrtf(dx*dx + dy*dy);
}

void AStar::print_solution(map_struct map, vector<vector<int>> sol) {
	// Print out the solution path. Shows if hammer is used to break the wall.
	cout << endl << "Printing out the solution." << endl;
	cout << "Path: '*', hammered wall: '@'" << endl << endl;

	for (int i = 0; i < map.ny; i++) {
		for (int j = 0; j < map.nx; j++) {
			// Print out the values
			if (map.map[i][j] == 2) {
				cout << "S" << " ";
			}
			else if (map.map[i][j] == 3) {
				cout << "T" << " ";
			}
			else {
				cout << map.map[i][j] << " ";
			}
		}

		cout << "  ";

		for (int j = 0; j < map.nx; j++) {
			// Print out the solution. Check if the coordinate has path on it.
			bool path_found = false;
			for (unsigned int k = 0; k < sol.size(); k++) {
				if (i == sol[k][0] && j == sol[k][1]) {
					path_found = true;
				}					  
			}

			if (path_found) {
				if (map.map[i][j] == 1) {
					cout << '@' << " "; // Hammered
				}
				else {
					cout << '*' << " ";
				}					   				
			}
			else {
				cout << map.map[i][j] << " ";
			}								 			
		}
		cout << endl;
	}
}

/*==============================================================================
	DEBUG FUNCTIONS
==============================================================================*/
void AStar::print_map_data(map_struct map) {
	// Print out the map data.
	cout << endl << "Map Data:" << endl;
	cout << "Symbol: 0: path, 1: wall, 2: start, 3: target" << endl << endl;
	for (int i = 0; i < map.ny; i++) {
		for (int j = 0; j < map.nx; j++) {
			// Print out the values
			cout << map.map[i][j] << " ";
		}
		cout << endl;
	}
	cout << endl;
	cout << "Dimension: (" << map.ny << "," << map.nx << ")" << endl;
	cout << "Start pos.: (" << map.s[0] << "," << map.s[1] << ")" << endl;
	cout << "Target pos.: (" << map.t[0] << "," << map.t[1] << ")" << endl;
	
	if (map.hammer)	cout << "Allows hammer" << endl;
	else cout << "No hammer" << endl;

	cout << endl;
	//cout << "Start to target dist: " << distance(map.s, map.t) << endl;
}

void AStar::print_node_data(vector<vector<int>> map) {
	// Print out the map data.
	cout << endl << "Node Map Data:" << endl;
	cout << endl << "1: open node, 0: closed node" << endl << endl;

	for (unsigned int i = 0; i < map.size(); i++) {
		for (unsigned int j = 0; j < map[0].size(); j++) {
			// Print out the values
			cout << map[i][j] << " ";
		}
		cout << endl;
	}

}
