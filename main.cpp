/*==============================================================================
	ERIC NAULI SIHITE
	Made for a job coding challenge, 11/30/2017
==============================================================================*/

// Dependencies
#include "astar.h"
//#include <string>

// Class declaration
AStar astar;			

/*==============================================================================
	MAIN FUNCTION
==============================================================================*/
int main() {
	// Load the map data and find the shortest path using A* algorithm. 
	// Inputs: file name, allows hammer or not (1 = hammer, 0 = no hammer).

	// Test list:
	// 0 = example 1 from the challenge
	// 1 = example 1 from the challenge, blocked
	// 2 = example 2 from the challenge
	// 3 = example 2 from the challenge, blocked
	// 4 = concave wall
	// 5 = concave wall, blocked
	// 6 = convex wall
	// 7 = convex wall, blocked
	// 8 = snake?
	// 9 = snake?, blocked

	cout << "Map list:" << endl;
	cout << "     0 = example 1 from the challenge" << endl;
	cout << "     1 = example 1 from the challenge, target blocked" << endl;
	cout << "     2 = example 2 from the challenge" << endl;
	cout << "     3 = example 2 from the challenge, target blocked" << endl;
	cout << "     4 = concave wall" << endl;
	cout << "     5 = concave wall, target blocked" << endl;
	cout << "     6 = convex wall" << endl;
	cout << "     7 = convex wall, target blocked" << endl;
	cout << "     8 = snake?" << endl;
	cout << "     9 = snake?, target blocked" << endl;	 
	
	cout << "Input the map number: ";
	int test_number;
	cin >> test_number;
	string str = to_string(test_number);

	cout << endl << "Use hammer? Input 1 for yes, 0 for no: ";
	int use_hammer;
	cin >> use_hammer;

	string file_name;
	file_name.append("test");
	file_name.append(str);
	file_name.append(".txt");

	// filename, 1 = hammer, 0 = no hammer
	if (!astar.astar_solve(file_name, use_hammer)) {
		cout << endl << "Solution not found. Target unreachable." << endl;
	}

	//cout << endl << "PRESS ANY KEY TO CLOSE";
	system("PAUSE");
	//char c;
	//cin >> c;	// Pause before closing, press key to close
	return 0;
}
