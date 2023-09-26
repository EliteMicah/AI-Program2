#include <iostream> //cin,cout
#include <fstream> //files
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


int main() {

	string files[10] = { "P2x2m.txt",
	"P2x4s.txt","P3x3.txt","P3x4s.txt","P3x8s.txt",
	"P4x6.txt","P3x10.txt", "P5x10.txt", "P10x10.txt", "P15x10.txt", };
	string filename;
	navigationGraph g1;

	cout << setw(10) << "Filename" << setw(10) << "Cost" << "\tSolution Path (UC/A*)" << endl;
	for (int i = 0; i < 10; i++) {
		int su, sa;
		vector<int> pu, pa;
		filename = files[i];
		cout << setw(10) << filename;
		if (!readGraphFromFile(filename, g1))
			exit(1);
		int start = 0, goal = g1.size - 1;
		su = 0;
		sa = 0;
		vector<nodeType> pathu = uniform_cost_search_Nav(g1, start, goal);
		vector<nodeType> patha = astar_search_Nav(g1, start, goal);
		cout << fixed << setprecision(1);
		cout << setw(10) << pathu[0].costG;
		for (auto x : pathu)
			pu.insert(pu.begin(), x.state);
		for (auto x : patha)
			pa.insert(pa.begin(), x.state);
		cout << setw(5) << " ";
		for (auto x : pu)
			cout << setw(4) << x;
		cout << endl;
		cout << setw(25) << " ";
		for (auto x : pa)
			cout << setw(4) << x;
		cout << endl;
	}

	system("pause");
	return 0;
}


bool readGraphFromFile(string filename, navigationGraph& g1) {
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
	nodeType newnode;
	int numChildren = 0;
	newnode.parent = id;

	for (int i = 0; i < g1.size; i++) {

		//there is a connection between vertices id and i
		if (g1.adjM[id][i] > 0) { 
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

	vector<nodeType> path;
	nodeType curr = exploredNodes[currentId];
	path.push_back(curr);

	while (curr.parent >= 0) {
		curr = exploredNodes[curr.parent];
		path.push_back(curr);
	}

	return path;
}

//Put here your function implementations

//uniform cost search between two nodes, returns a path
vector<nodeType> uniform_cost_search_Nav(navigationGraph g1, int start, int goal) {

}

//A* search between two nodes, returns a path
vector<nodeType> astar_search_Nav(navigationGraph g1, int start, int goal) {

}
