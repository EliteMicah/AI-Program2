#include <iostream>
#include <fstream>
#include <queue>
#include <vector>
#include <stack>
#include <map>
#include <iomanip>
using namespace std;


struct nodeType { //structure of each node in the tree

	int state; //the index of the vertex
	int parent; //the state of the parent node
	double costG; //g(n), path cost
	double costH; //h(n), estimated cost by heuristic from present to goal

		//overload the < operator so we can use it for the priority queue
	friend bool operator<(nodeType x, nodeType y) {
		return x.costG + x.costH > y.costG + y.costH;
	}
};

struct navigationGraph {

	int size; //number of vertices
	double** adjM; //adjacency matrix
	vector<double> xloc;
	vector<double> yloc;

};

//given a file, populate the graph
bool readGraphFromFile(string filename, navigationGraph& g1); 

//given a current node curr and the graph, return the nodes you can reach from curr
vector<nodeType> Expand(nodeType curr, navigationGraph g1); 

//Helper function: returns the solution path given the map of explored nodes by the algorithm
vector<nodeType> Solution(int currentId, int startId, map<int, nodeType>
	exploredNodes);

//uniform cost search between two nodes, returns a path
vector<nodeType> uniform_cost_search_Nav(navigationGraph g1, int start, int goal);

//A* search between two nodes, returns a path
vector<nodeType> astar_search_Nav(navigationGraph g1, int start, int goal); 

// Find heuristic
int getH2(int, int);


int main() {

	string files[10] = { "P2x2m.txt",
	"P2x4s.txt","P3x3.txt","P3x4s.txt","P3x8s.txt",
	"P4x6.txt","P3x10.txt", "P5x10.txt", "P10x10.txt", "P15x10.txt"};
	string filename;
	navigationGraph g1;

	cout << setw(10) << "Filename" << setw(10) << "Cost" << "\tSolution Path (UC/A*)" << endl;
	for (int i = 0; i < 10; i++) {
		vector<int> pu, pa;
		filename = files[i];

		cout << setw(10) << filename;
		if (!readGraphFromFile(filename, g1))
			exit(1);

		int start = 0, goal = g1.size - 1, su = 0, sa = 0;

		// Using Uniform-Cost search to find the solution
		vector<nodeType> pathUni = uniform_cost_search_Nav(g1, start, goal);

		// Using A* search to find the solution
		vector<nodeType> pathAstar = astar_search_Nav(g1, start, goal);

		cout << fixed << setprecision(1);
		cout << setw(10) << pathUni[0].costG;

		// Inserting the state? into the vector of int's for Uni search
		for (auto &x : pathUni)
			pu.insert(pu.begin(), x.state);

		// Inserting the state? into the vector of int's for A* search
		for (auto &x : pathAstar)
			pa.insert(pa.begin(), x.state);

		cout << setw(5) << " ";

		// Printing the result for Uni search
		for (auto x : pu)
			cout << setw(4) << x;
		cout << endl;
		cout << setw(25) << " ";

		// Printing the result for A* search
		for (auto x : pa)
			cout << setw(4) << x;
		cout << endl;
	}

	system("pause");
	return 0;
}


bool readGraphFromFile(string filename, navigationGraph& g1) {

	// Grab the file name
	ifstream gfile(filename);
	if (!gfile)	{
		cout << "Error opening file ..." << endl;
		return false;
	}

	int m, n; //#of vertices, #of edges,
	gfile >> m >> n;
	g1.size = m; //set the size of the graph

	//allocate memory for the adjacacy matrix
	g1.adjM = new double* [n];

	for (int i = 0; i < n; i++)
		g1.adjM[i] = new double[n] {0};
	int id; //vertex id
	double x, y;//x-y coordinates

	for (int i = 0; i < m; i++) { //read the vertex information
		gfile >> id >> x >> y;
		g1.xloc.push_back(x);
		g1.yloc.push_back(y);
	}

	int a, b; //read the edges and fill out the adjacency matrix
	for (int i = 0; i < n; i++)	{
		gfile >> a >> b;
		g1.adjM[a][b] = 1;
		g1.adjM[b][a] = 1;
	}

	gfile.close();
	return true;
}


vector<nodeType> Expand(nodeType curr, navigationGraph g1) {
	vector<nodeType> children;
	int id = curr.state; //the id of current node
	nodeType newnode{};
	int numChildren = 0;

	// Parent shows that it was already visited
	newnode.parent = id;

	// Search through the navigation Graph
	for (int i = 0; i < g1.size; i++) {

		// There is a connection between vertices id and i
		// adjm = adjacency matrix (double pointer)
		if (g1.adjM[id][i] > 0) { 

			// square root of the power of (x[id] - x[i])^2 + (y[id] - y[i])^2
			double d = sqrt(pow(g1.xloc[id] - g1.xloc[i], 2) +
			pow(g1.yloc[id] - g1.yloc[i], 2));

			//update the cost g(n)=cost of parent + cost of transition
			newnode.costG = curr.costG + d; 
			newnode.state = i;

			//the heuristic cost is assumed 0 (the informed searches will update this themselves)
			newnode.costH = 0; 
			children.push_back(newnode);
		}
	}
	return children;
}


vector<nodeType> Solution(int currentId, int startId, map<int, nodeType> exploredNodes) {

	// Create the solution path
	vector<nodeType> path;

	// Get the current for the explored nodes / id's
	nodeType curr = exploredNodes[currentId];

	// Push those back in the solution path
	path.push_back(curr);

	// While the parent still exists
	while (curr.parent >= 0) {

		// Add the parent's id to the solution
		curr = exploredNodes[curr.parent];
		path.push_back(curr);
	}

	return path;
}

// Uniform cost search between two nodes, returns a path
vector<nodeType> uniform_cost_search_Nav(navigationGraph g1, int start, int goal) {

	// Starting node
	// Start
	// -1 = parent
	// 0 = g(n), path cost so far
	// 0 = h(n), estimated cost by heuristic from present to goal
	nodeType startNode = { start, -1, 0, 0 };

	vector<nodeType> path;
	priority_queue<nodeType> frontier; //for A* the frontier is a priority queue

	//initialize the frontier with the start node
	frontier.push(startNode);

	//The set of all nodes already explored
	map<int, nodeType> reached; 
	reached[start] = startNode;

	// Cycle through the frontier
	while (!frontier.empty()) {

		// Place current at top and pop
		nodeType curr = frontier.top();
		frontier.pop();

		// Check if goal is reached, return solution if true
		if (curr.state == goal) {
			reached[goal] = curr;
			return Solution(goal, start, reached);
		}

		// Find all the possible children
		vector<nodeType> children = Expand(curr, g1);
		for (auto& x : children) {

			// Check if the child state has not been explored or has a lower costG
			if ((reached.find(x.state) == reached.end()) || (reached[x.state].costG > x.costG)) {

				// Update frontier and reached
				frontier.push(x);
				reached[x.state] = x;
			}
		}
	}

	return path;
}

//A* search between two nodes, returns a path
vector<nodeType> astar_search_Nav(navigationGraph g1, int start, int goal) {
	
	vector<nodeType> path;
	priority_queue<nodeType> frontier; //for A* the frontier is a priority queue
	nodeType startNode = { start, -1, 0, 0 }; //initialize the start node
	// Idk if costH needs to be = getH2(start, goal)
	// Doesn't seem to change anything if I do that 

	//initialize the frontier with the start node
	frontier.push(startNode); 

	map<int, nodeType> reached; //The set of all nodes already explored
	reached[start] = startNode;

	// Cycle through the frontier
	while (!frontier.empty()) {

		// Place current at top and pop
		nodeType curr = frontier.top(); 
		frontier.pop();

		// Check if goal is reached, return solution if true
		if (curr.state == goal) {
			reached[goal] = curr;
			return Solution(goal, start, reached);
		}

		// Find all the possible children
		vector<nodeType> children = Expand(curr, g1);
		for (auto &x : children) {

			// Check if the child state has not been explored or has a lower costG
			if ((reached.find(x.state) == reached.end()) || (reached[x.state].costG > x.costG)) {

				// Check heuristic
				x.costH = getH2(x.state, goal);

				// Update frontier and reached
				frontier.push(x);
				reached[x.state] = x;
			}
		}
	}

	return path;
}

// Finding Heuristic function
int getH2(int X, int Y) {

	// Solution variable
	int d = 0;

	for (int i = 0; i < 9; i++) {
		int s = X;
		int w = Y;

		// Heuristic search
		if (s > 0) {
			int x0 = s / 3; 
			int x1 = s % 3;
			int y0 = w / 3; 
			int y1 = w % 3;
			d += abs(x0 - y0) + abs(x1 - y1);
		}
	}

	// Return solution
	return d;
}
