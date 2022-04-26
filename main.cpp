// Code submitted by -->
// Abhinav Tyagi 2020A7PS2043H
// Darshan Chandak 2020A7PS2085H
// Dhruv Merchant 2020A7PS2063H

#include <iostream>
#include <vector>
#include <climits>
#include <list>
#include <queue>
#include <algorithm>

using namespace std;

int economical_path = 0;
//#define V 9
int V;
vector<int> priorityOrder;
vector<int> bus_route, temp_route;

class Graph
{
    int Vertex;

    list<pair<int, int>> *adjacencyList;

public:
    Graph(int Vertex);

    void addPath(int stop1, int stop2, int distance);

    void shortestPath(int);
};

// create graph using adjacency list
Graph::Graph(int Vertex)
{
    this->Vertex = Vertex;
    adjacencyList = new list<pair<int, int>>[Vertex];
}

// weighted roads created between stops
void Graph::addPath(int stop1, int stop2, int distance)
{
    adjacencyList[stop1].push_back(make_pair(stop2, distance));
    adjacencyList[stop2].push_back(make_pair(stop1, distance));
}

// find shortest path without priority order specified by user
void Graph::shortestPath(int Vertex)
{
    int stopDistance[Vertex][1LL << Vertex];

    for (int i = 0; i < Vertex; i++)
        for (int j = 0; j < (1LL << Vertex); j++)
            stopDistance[i][j] = INT32_MAX;

    priority_queue<pair<int, int>> priorityOrder;

    priorityOrder.push({0, 1LL << 0});

    stopDistance[0][1LL << 0] = 0;

    while (!priorityOrder.empty())
    {
        int currentStop = priorityOrder.top().first;
        int mask = priorityOrder.top().second;
        priorityOrder.pop();
        list<pair<int, int>>::iterator i;
        for (i = adjacencyList[currentStop].begin(); i != adjacencyList[currentStop].end(); i++)
        {
            int adjacentStop = i->first;
            int distance = i->second;
            if (stopDistance[adjacentStop][(1LL << adjacentStop) | mask] > stopDistance[currentStop][mask] + distance)
            {
                priorityOrder.push(make_pair(adjacentStop, mask | (1LL << adjacentStop)));
                stopDistance[adjacentStop][mask | (1LL << adjacentStop)] = stopDistance[currentStop][mask] + distance;
            }
        }
    }

    int minimum_path = INT32_MAX;

    for (int i = 0; i < Vertex; i++)
    {
        minimum_path = min(minimum_path, stopDistance[i][(1LL << Vertex) - 1]);
    }
    cout << "The Minimum Distance to be travelled by the Bus is: " << minimum_path;
}

int shortestPathBetweenStops(int distance[], bool track_path[])
{
    int min = INT_MAX, min_index;

    for (int i = 0; i < V; i++)

        if (track_path[i] == false and distance[i] <= min)
            min = distance[i], min_index = i;

    return min_index;
}

void createRoot(int parent[], int current_stop)
{
    if (parent[current_stop] == -1)
        return;

    createRoot(parent, parent[current_stop]);

    temp_route.push_back(current_stop);
}

void finalRoute(int destination, int parent[])
{
    temp_route.clear();

    createRoot(parent, destination);

    reverse(temp_route.begin(), temp_route.end());

    for (int i : temp_route)
        bus_route.push_back(i);
}

void inputPriority()
{
    priorityOrder.push_back(0);

    cout << "Specify priority order: " << endl;

    for (int i = 1; i < V; i++)
    {
        int priority;
        cin >> priority;
        priorityOrder.push_back(priority);
    }
}

// find shortest path between two nodes
void dijkstra(vector<vector<int>> &graph, vector<vector<int>> &new_graph, int current_priority)
{
    // updating new graph by adding paths between the current stop and stops that have already been visited by the bus
    for (int i = 0; i <= current_priority; ++i)
    {
        if (graph[priorityOrder[i]][priorityOrder[current_priority]])
        {
            new_graph[priorityOrder[i]][priorityOrder[current_priority]] = new_graph[priorityOrder[current_priority]][priorityOrder[i]] = graph[priorityOrder[i]][priorityOrder[current_priority]];
        }
    }

    int distance[V];
    bool track_path[V] = {false};
    int parent[V] = {0};
    parent[priorityOrder[current_priority]] = -1;
    for (int i = 0; i < V; i++)
        distance[i] = INT_MAX;
    distance[priorityOrder[current_priority]] = 0;
    for (int i = 0; i < V - 1; i++)
    {
        int previous_stop = shortestPathBetweenStops(distance, track_path);
        track_path[previous_stop] = true;
        for (int current_stop = 0; current_stop < V; current_stop++)
            if (!track_path[current_stop] and new_graph[previous_stop][current_stop] && distance[previous_stop] + new_graph[previous_stop][current_stop] < distance[current_stop])
            {
                parent[current_stop] = previous_stop;
                distance[current_stop] = distance[previous_stop] + new_graph[previous_stop][current_stop];
            }
    }

    finalRoute(priorityOrder[current_priority - 1], parent);

    if (current_priority == V - 1)
        bus_route.push_back(priorityOrder[current_priority]);

    economical_path += distance[priorityOrder[current_priority - 1]];
    if (current_priority + 1 < V)
        dijkstra(graph, new_graph, current_priority + 1);
}

void printFinalPathWithPriority(vector<vector<int>> &graph, vector<vector<int>> &new_graph)
{
    dijkstra(graph, new_graph, 1);
    cout << "The Minimum Distance to be travelled by the Bus is: " << economical_path << endl;
    cout << "The Path travelled by the Bus is: " << endl;
    for (int i = 0; i < bus_route.size(); i++)
    {
        cout << bus_route[i];
        if (i != bus_route.size() - 1)
            cout << " -> ";
    }
    cout << endl;
}

int main()
{
    cout << "Specify Number of Stops : " << endl;
    cin >> V;
    V = V + 1;
    vector<vector<int>> graph(V, vector<int>(V, 0));
    for (int i = 0; i < V; i++)
    {
        cout << "Specify Distance of all Stops including itself from Stop " << i << ":" << endl;

        for (int j = 0; j < V; j++)
        {
            cin >> graph[i][j];
        }
    }
    vector<vector<int>> new_graph(V, vector<int>(V, 0));

    Graph G(V);
    for (int i = 0; i < V; ++i)
    {
        for (int j = 0; j < V; ++j)
        {
            if (graph[i][j])
            {
                G.addPath(i, j, graph[i][j]);
            }
        }
    }
    cout << "Do you wish to specify the priority order? (Y/N)" << endl;
    char response;
    cin >> response;
    while (response)
    {
        if (response == 'y' || response == 'Y')
        {
            inputPriority();
            printFinalPathWithPriority(graph, new_graph);
            break;
        }
        else if (response == 'n' || response == 'N')
        {
            G.shortestPath(V);
            break;
        }
        else
        {
            cout << "Please enter a valid response!" << endl;
            cin >> response;
            continue;
        }
    }
    return 0;
}
// TEST1
// int graph[V][V] = {{0, 4, 2, 6, 9, 7, 8, 4, 3},
//                    {4, 0, 6, 0, 0, 0, 0, 0, 0},
//                    {2, 6, 0, 3, 0, 0, 7, 0, 0},
//                    {6, 0, 3, 0, 4, 0, 0, 0, 0},
//                    {9, 0, 0, 4, 0, 1, 0, 0, 8},
//                    {7, 0, 0, 0, 1, 0, 10, 0, 0},
//                    {8, 0, 7, 0, 0, 10, 0, 8, 0},
//                    {4, 0, 0, 0, 0, 0, 8, 0, 9},
//                    {3, 0, 0, 0, 8, 0, 0, 9, 0}};

// PRIORITY ORDER: 6 5 7 3 1 2 4 8
// PATH: 0 6 5 0 7 0 3 0 1 2 3 4 8

// TEST2
// 4 0 1 1 3 4 1 0 0 0 1000 1 0 0 1 1000 3 0 1 0 0 4 1000 1000 0 0 Y 2 4 3 1
// PATH: 0 2 0 4 0 2 3 2 0 1