#include "graph.hpp"

int main() {
    Graph myGraph(10);

    myGraph.addEdge(0, 1);
    myGraph.addEdge(0, 2);
    myGraph.addEdge(1, 3);
    myGraph.addEdge(2, 4);
    myGraph.addEdge(3, 5);
    myGraph.addEdge(4, 5);
    myGraph.addEdge(5, 6);
    myGraph.addEdge(6, 7);
    myGraph.addEdge(7, 8);
    myGraph.addEdge(8, 9);


    // Print the graph
    myGraph.printGraph();

    // Test BFS
    std::cout << "BFS starting from vertex 0: ";
    myGraph.BFS(0);
    std::cout << std::endl;

    // Test DFS
    std::cout << "DFS starting from vertex 8: ";
    myGraph.DFS(8);
    std::cout << std::endl;

    // Test DFS Recursive
    std::cout << "DFS (recursive) starting from vertex 0: ";
    myGraph.DFSRec(0);
    std::cout << std::endl;

    // Test the number of components
    std::cout << "Number of connected components: " << myGraph.getNumberOfComponents() << std::endl;

    // Test nodes on each component
    std::vector<std::vector<int>> components = myGraph.getNodesOnEachComponent();
    for (size_t i = 0; i < components.size(); ++i) {
        std::cout << "Nodes in component " << i << ": ";
        for (int node : components[i]) {
            std::cout << node << " ";
        }
        std::cout << std::endl;
    }

    // Test shortest path
    size_t shortestPathLength = myGraph.getShortestPath(0, 6);
    std::cout << "Shortest path length from 0 to 6: " << shortestPathLength << std::endl;

    // Test nodes at a certain level
    std::queue<int> nodesAtLevel = myGraph.getNodesAtLevel(0, 2);
    std::cout << "Nodes at level 2 starting from vertex 0: ";
    while (!nodesAtLevel.empty()) {
        std::cout << nodesAtLevel.front() << " ";
        nodesAtLevel.pop();
    }
    std::cout << std::endl;

    // Test all possible paths
    std::vector<std::vector<int>> allPaths = myGraph.getAllPossiblePaths(0, 6);
    std::cout << "All possible paths from 0 to 6:" << std::endl;
    for (const auto& path : allPaths) {
        for (int node : path) {
            std::cout << node << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
