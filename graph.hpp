#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <iostream>
#include <queue>
#include <stack>
#include <algorithm>

class Graph {
public:
    explicit Graph(int);
    void addEdge(int, int);
    void addEdgeDir(int, int);
    void printGraph() const;
    size_t getDegree(int);
    void BFS(int);
    void DFS(int);
    void DFSRec(int);
    size_t getNumberOfComponents() const;
    std::vector<std::vector<int>> getNodesOnEachComponent() const;
    size_t getShortestPath(int, int) const;
    std::queue<int> getNodesAtLevel(int, size_t) const;
    std::vector<std::vector<int>> getAllPossiblePaths(int, int) const;
    bool hasCycleDir() const;
    bool hasCycle() const;
    std::vector<int> topSortDFS() const;
    std::vector<int> topSortKahn() const; // Kahn's algorithm (for directed graph) (take adj list as if those are the vertices that get into that vertex)
    size_t getNumberOfSCC() const; // Kosaraju's algorithm
    void transpose();

private:
    std::vector<std::vector<int>> _graph;

    void DFSUtil(int, std::vector<bool>&) const;
    void DFSUtilForComponents(int, std::vector<bool>&) const;
    void DFSUtilForComponentNodes(int, std::vector<int>&, std::vector<bool>&) const;
    void DFSUtilForAllPossiblePaths(int, int, int, std::vector<bool>&, std::vector<int>&, std::vector<std::vector<int>>&) const;
    bool DFSUtilForCycleDir(int, std::vector<bool>, std::vector<bool>) const;
    bool DFSUtilForCycle(int, int, std::vector<bool>) const;
    void DFSUtilForSCC1() const;
    void DFSUtilForSCC2() const;
    std::vector<int> reconstruct(int, int, std::vector<int>&) const;
    void getInDegree(std::vector<int>&) const;
};

#endif // GRAPH_HPP