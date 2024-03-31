#include "graph.hpp"

Graph::Graph(int V)
    : _graph(V) {}

void Graph::addEdge(int u, int v)
{
    _graph[u].push_back(v);
    _graph[v].push_back(u);
}

void Graph::addEdgeDir(int from, int to)
{
    _graph[from].push_back(to);
}

void Graph::printGraph() const
{
    std::cout << "Adjacency list of the graph:" << std::endl;
    for (size_t i {0}; i < _graph.size(); ++i)
    {
        std::cout << i << ": ";
        for (auto& neighbour : _graph[i])
        {
            std::cout << neighbour << " ";
        }
        std::cout << std::endl;
    }
}

size_t Graph::getDegree(int vertex)
{
    return _graph[vertex].size();
}

void Graph::BFS(int startVertex)
{
    std::vector<bool> visited(_graph.size(), false);
    std::queue<int> q;

    visited[startVertex] = true;
    q.push(startVertex);

    while (!q.empty())
    {
        int currentVertex = q.front();
        std::cout << currentVertex << " ";
        q.pop();

        for (auto& neighbour : _graph[currentVertex])
        {
            if (!visited[neighbour])
            {
                visited[neighbour] = true;
                q.push(neighbour);
            }
        }
    }
}

void Graph::DFS(int startVertex)
{
    std::vector<bool> visited(_graph.size(), false);
    std::stack<int> s;

    s.push(startVertex);

    while (!s.empty())
    {
        int currentVertex = s.top();
        s.pop();

        if (!visited[currentVertex])
        {
            std::cout << currentVertex << " ";
            visited[currentVertex] = true;
        }

        for (auto& neighbour : _graph[currentVertex])
        {
            if (!visited[neighbour])
            {
                s.push(neighbour);
            }
        }
    }
}

void Graph::DFSRec(int startVertex)
{
    std::vector<bool> visited(_graph.size(), false);
    DFSUtil(startVertex, visited);
}

void Graph::DFSUtil(int currentVertex, std::vector<bool>& visited) const
{
    visited[currentVertex] = true;
    //pre
    std::cout << currentVertex << " ";

    for (auto& neighbour : _graph[currentVertex])
    {
        if (!visited[neighbour])
        {
            DFSUtil(neighbour, visited);
        }
        //post
    }
}

size_t Graph::getNumberOfComponents() const
{
    std::vector<bool> visited(_graph.size(), false);
    size_t components = 0;

    for (int i {0}; i < _graph.size(); ++i)
    {
        if (!visited[i])
        {
            DFSUtilForComponents(i, visited);
            ++components;
        }
    }

    return components;
}

void Graph::DFSUtilForComponents(int currentVertex, std::vector<bool>& visited) const
{
    visited[currentVertex] = true;

    for (auto& neighbour : _graph[currentVertex])
    {
        if (!visited[neighbour])
        {
            DFSUtilForComponents(neighbour, visited);
        }
    }
}

std::vector<std::vector<int>> Graph::getNodesOnEachComponent() const
{
    std::vector<bool> visited(_graph.size(), false);
    std::vector<std::vector<int>> components;

    for (int i {0}; i < _graph.size(); ++i)
    {
        if (!visited[i])
        {
            std::vector<int> componentNodes;
            componentNodes.push_back(i);
            DFSUtilForComponentNodes(i, componentNodes, visited);
            components.push_back(componentNodes);
        }
    }

    return components;
}

void Graph::DFSUtilForComponentNodes(int currentVertex, std::vector<int>& componentNodes, std::vector<bool>& visited) const
{
    visited[currentVertex] = true;

    for (auto& neighbour : _graph[currentVertex])
    {
        if (!visited[neighbour])
        {
            componentNodes.push_back(neighbour);
            DFSUtilForComponentNodes(neighbour, componentNodes, visited);
        }
    }
}

size_t Graph::getShortestPath(int u, int v) const
{
    std::vector<bool> visited(_graph.size(), false);
    std::queue<int> q;

    visited[u] = true;
    q.push(u);

    size_t level = 0;

    while (!q.empty())
    {
        size_t currentLevelSize = q.size();

        for (size_t i {0}; i < currentLevelSize; ++i)
        {
            int currentVertex = q.front();
            q.pop();

            for (auto& neighbour : _graph[currentVertex])
            {
                if (!visited[neighbour])
                {
                    visited[neighbour] = true;

                    if (neighbour == v)
                    {
                        return level + 1;
                    }

                    q.push(neighbour);
                }
            }
        }

        ++level;
    }

    return level;
}

std::queue<int> Graph::getNodesAtLevel(int vertex, size_t lvl) const
{
    std::vector<bool> visited(_graph.size(), false);
    std::queue<int> q;

    visited[vertex] = true;
    q.push(vertex);

    size_t level = 0;

    while (!q.empty())
    {
        if (level == lvl)
        {
            return q;
        }

        size_t currentLevelSize = q.size();

        for (size_t i {0}; i < currentLevelSize; ++i)
        {
            int currentVertex = q.front();
            q.pop();

            for (auto& neighbour : _graph[currentVertex])
            {
                if (!visited[neighbour])
                {
                    visited[neighbour] = true;
                    q.push(neighbour);
                }
            }
        }

        ++level;
    }

    return std::queue<int>{};
}

std::vector<std::vector<int>> Graph::getAllPossiblePaths(int u, int v) const
{
    std::vector<std::vector<int>> paths;
    std::vector<bool> visited(_graph.size(), false);
    std::vector<int> parents(_graph.size(), -1);
    DFSUtilForAllPossiblePaths(u, u, v, visited, parents, paths);

    return paths;
}

void Graph::DFSUtilForAllPossiblePaths(int src, int currentVertex, int dest, std::vector<bool>& visited, std::vector<int>& parents, std::vector<std::vector<int>>& paths) const
{
    visited[currentVertex] = true;

    if (currentVertex == dest)
    {
        paths.push_back(reconstruct(src, dest, parents));
    }

    for (auto& neighbour : _graph[currentVertex])
    {
        if (!visited[neighbour])
        {
            parents[neighbour] = currentVertex;
            DFSUtilForAllPossiblePaths(src, neighbour, dest, visited, parents, paths);
        }
    }

    visited[currentVertex] = false;
}

std::vector<int> Graph::reconstruct(int src, int dest, std::vector<int>& parents) const
{
    std::vector<int> path;
    int current = dest;

    while (current != src)
    {
        path.push_back(current);
        current = parents[current];
    }

    path.push_back(src);
    std::reverse(path.begin(), path.end());

    return path;
}

bool Graph::hasCycleDir() const
{
    std::vector<bool> visited(_graph.size(), false);
    std::vector<bool> onStack;

    for (int i {0}; i < _graph.size(); ++i)
    {
        if (!visited[i] && DFSUtilForCycleDir(i, visited, onStack))
        {
            return true;
        }
    }

    return false;
}

bool Graph::DFSUtilForCycleDir(int currentVertex, std::vector<bool> visited, std::vector<bool> onStack) const
{
    visited[currentVertex] = true;
    onStack[currentVertex] = true;

    for (auto& v : _graph[currentVertex])
    {
        if (!visited[v])
        {
            if (DFSUtilForCycleDir(v, visited, onStack))
            {
                return true;
            }
        }
        else if (onStack[v])
        {
            return true;
        }
    }

    onStack.pop_back();
    return false;
}

bool Graph::hasCycle() const
{
    std::vector<bool> visited(_graph.size(), false);

    for (int i {0}; i < _graph.size(); ++i)
    {
        if (!visited[i] && DFSUtilForCycle(i, -1, visited))
        {
            return true;
        }
    }

    return false;
}

bool Graph::DFSUtilForCycle(int currentVertex, int parent, std::vector<bool> visited) const
{
    visited[currentVertex] = true;

    for (auto& v : _graph[currentVertex])
    {
        if (!visited[v])
        {
            if (DFSUtilForCycle(v, currentVertex, visited))
            {
                return true;
            }
        }
        else if (v != currentVertex)
        {
            return true;
        }
    }

    return false;
}

std::vector<int> Graph::topSortKahn() const
{
    std::vector<int> inDegree(_graph.size(), 0);
    getInDegree(inDegree);

    std::vector<int> topSorted;
    std::vector<bool> visited(_graph.size(), false);
    std::queue<int> q;

    for (int i {0}; i < _graph.size(); ++i)
    {
        if (inDegree[i] == 0)
        {
            q.push(i);
        }

        while (!q.empty())
        {
            int currentVertex = q.front();
            q.pop();
            topSorted.push_back(currentVertex);

            for (auto& v : _graph[currentVertex])
            {
                --inDegree[v];
                if (inDegree[v] == 0)
                {
                    q.push(v);
                }
            }
        }
    }

    if (topSorted.size() != _graph.size())
    {
        std::cout << "The graph contains loops.";
        return topSorted;
    }

    return topSorted;
}

void Graph::getInDegree(std::vector<int>& inDegree) const
{
    for (size_t i {0}; i < _graph.size(); ++i)
    {
        for (auto& v : _graph[i])
        {
            ++inDegree[v];
        }
    }
}

size_t Graph::getNumberOfSCC() const
{
    std::vector<int> procTime;


}

