// As usual, here's our boring I/O redirect code
#include <stdio.h>
void RedirectIO(const char* filename) {
    char buf[256];
    
    buf[0] = '\0';
    sprintf(buf, "%s.in", filename);
    freopen(buf, "r", stdin);


    buf[0] = '\0';
    sprintf(buf, "%s.out", filename);
    freopen(buf, "w", stdout);
}

/*
Since the problem explictly states that we can only visit *discrete* cells, we can apply a graph algorithm here where every cell is a node
My solution works in two steps:
1) First, initialize the edges for every cell. Here, I take into account obstructs from obstacles 
2) Then apply a shortest path algorithm. Since n,m<=5000, Dijkstra's algorithm works within the time limit, but we can do better, so I use an A* algorithm
*/

#include <iostream>
#include <algorithm>
#include <vector>
#include <set>
#include <queue>
#include <float.h>
#include <math.h>

using namespace std; 

// I don't like using pairs, so I created my own struct for coordinates
struct Coordinate {
    int x, y;
    Coordinate() : x(-1), y(-1) {}
    Coordinate(int x, int y) : x(x), y(y) {}

    bool operator<(const Coordinate& other) const {
        if(other.x == x) {
            return y < other.y;
        } else {
            return x < other.x; 
        }
    }
};

// I store my graph in a 1D array for better efficiency, so I need a function to convert 2D indices to 1D indices
int get_index(int n, int x, int y) {
    return (n * y + x);
}

int get_index(int n, const Coordinate& c) {
    return get_index(n, c.x, c.y);
}

// check if points x or y are on the segment from A to B
bool on_segment(const Coordinate& a, const Coordinate& b, double x, double y) {
    int slope = (a.y - b.y) / (a.x - b.x);
    return (
        (abs((y - a.y) - slope * (x - a.x)) < 1e-6) && // on the line
        min(a.x, b.x) <= x && x <= max(a.x, b.x) && // x coord in range
        min(a.y, b.y) <= y && y <= max(a.y, b.y)    // y coord in range 
    );
}

// check if the segment from a to b is obstructed by polygon
bool obstructed(const vector<Coordinate>& polygon, const Coordinate& a, const Coordinate& b) {
    bool good = true;
    for(int i = 0; i < polygon.size(); i++) {
        Coordinate c = polygon[i];
        Coordinate d = polygon[(i + 1) % polygon.size()];

        // for all 3 segment types (vertical, horizontal, and diagonal), if the end point b is on the segment cd, then we say that segment ab is obstructed by cd
        // for the diagonal case, we also have to take into account whether the segment crosses over the segment

        if(c.x == d.x) {
            if(d.y < c.y)
                swap(c, d);

            if(b.x == c.x && c.y <= b.y && b.y <= d.y) {
                good = false;
                break;
            }

        } else if(c.y == d.y) {
            if(d.x < c.x)
                swap(c, d);

            if(b.y == c.y && c.x <= b.x && b.x <= d.x) {
                good = false;
                break;
            }
        } else {
            // does the edge cross through the line?
            // if so, then one point must be above the line and the other must be below

            if(d.x > c.x)
                swap(c, d);

            int slope = (c.y - d.y) / (c.x - d.x);

            bool on_diagonal = on_segment(c, d, b.x, b.y);

            // if the ab crosses over cd, then the solution for the system of linear equations represented by the segments must be on both segments
            bool cross_diagonal = false;
            if(a.x != b.x) {
                double x = 0.5 * (double(a.y - c.y) / slope + a.x + c.x);
                double y = c.y + slope * (x - c.x);
                cross_diagonal = on_segment(a, b, x, y) && on_segment(c, d, x, y);
            }

            if(on_diagonal || cross_diagonal) {
                good = false;
                break;
            }

            
        }
    }

    return !good;
}

// if the done is at cell (x, y), then it can visit 8 adjacent cells from (x, y)
// this array stores the offset for each adjacent cell (ie, the edges of the graph)
Coordinate offsets[] = {
    Coordinate( 0,  1),
    Coordinate( 1,  1), // 1
    Coordinate( 1,  0),
    Coordinate( 1, -1), // 3
    Coordinate( 0, -1),
    Coordinate(-1, -1), // 5
    Coordinate(-1,  0),
    Coordinate(-1,  1) // 7
};

// close off all edges to a certain point (and reopens them later)
// this is useful in making sure our drone visits only the next waypoint and not future waypoint by accident in the process
// I keep 3 states for an edge
//  1: this edge is on
//  0: this edge is off
// -1: this edge was on but has been turned off due to locking
// the reason I have -1 is because it makes it easier to know what an edge originally was when unlocking
void lock_coordinate(int n, int m, vector<int8_t> edges, Coordinate waypoint, bool enable) {
    for(int i = 0; i < 8; i++) {
        Coordinate from = waypoint;
        from.x -= offsets[i].x;
        from.y -= offsets[i].y;
        if(from.x >= 0 && from.y >= 0 && from.x < n && from.y < m) {
            int idx = get_index(n, from);
            int off = (i + 4) % 8;
            if(enable) {
                edges[idx + off] = (edges[idx + off] == 0 ? 0 : -1);
            } else {
                edges[idx + off] = (edges[idx + off] == -1 ? 1 : 0);
            }
        }
    }
}

int main() {
    RedirectIO("navigate");
    
    int n, m, k, p;
    cin >> n >> m >> k >> p;
    
    // store our waypoints into memory
    vector<Coordinate> waypoints(k);
    for(auto& c : waypoints) {
        cin >> c.x >> c.y;
        c.x--; // I want to recenter the grid on (0, 0) to make programming this more intuitive
        c.y--;
    }

    // store our obstacles into memory
    vector<vector<Coordinate>> obstacles;
    for(int i = 0; i < p; i++) {
        int L;
        cin >> L;

        vector<Coordinate>& polygon = obstacles.emplace_back(L);
        for(auto& c : polygon) {
            cin >> c.x >> c.y;
            c.x--;
            c.y--;
        }        
    }

    // build our list of edges
    // we have 8 possible edges for each vertex because of 8 possible offsets
    // to make accessing the edges easier, the 8i+j edge corresponds to the jth offset
    // the reason why I use uint8_t over bool is because I need more information to allow for node locking/unlocking that simply on/off
    // (see the lock_coordinate function for what node locking is) 
    vector<int8_t> edges(n*m*8);
    fill(edges.begin(), edges.end(), 0);
    for(int y = 0; y < m; y++) {
        for(int x = 0; x < n; x++) {
            Coordinate node = Coordinate(x, y);
            int idx = get_index(n, node);

            for(int i = 0; i < 8; i++) {
                Coordinate to = node;
                to.x += offsets[i].x;
                to.y += offsets[i].y;

                bool good = true;
                if(to.x < 0 || to.y < 0 || to.x >= n || to.y >= m) {
                    good = false;
                } else {
                    // check, over all polygons, if the edge goes through an obstacle
                    for(const auto& polygon : obstacles) {
                        if(obstructed(polygon, node, to)) {
                            good = false;
                            break;
                        }
                    }
                }

                edges[8 * idx + i] = (good ? 1 : 0);
            }
        }
    }

    // lock each future waypoint so our drone doesn't visit it early by accident
    for(const auto& point : waypoints) {
        lock_coordinate(n, m, edges, point, true);
    }

    Coordinate start = waypoints[0];
    vector<Coordinate> full_path; // the entire path our drone takes 
    full_path.push_back(start);

    for(int i = 1; i < waypoints.size(); i++) {
        Coordinate goal = waypoints[i];
        // unlock our goal coordinate
        lock_coordinate(n, m, edges, goal, false);

        // this stores information about each node in the graph
        struct NodeInfo {
            bool processed; // has this node been processed by the shortest path algorithm already?
            double distance; // what is the shortest distance to this node from our start node?
            Coordinate from; // which adjacent node did the shortest path to this node come from (basically, we store the path backwards so we can reconstruct it later)

            NodeInfo() : processed(false), distance(FLT_MAX) {}
        };

        // this array stores the information about each node
        vector<NodeInfo> graph(n*m);

        // use a work queue to go to the nodes with shortest distance first
        priority_queue<pair<double, Coordinate>> work_queue;
        work_queue.push({0, start});
        while(!work_queue.empty()) {
            // this is an implementation of the A* algorithm
            
            // get the next closest node
            auto current = work_queue.top();
            work_queue.pop();

            // since priority_queue looks for maximum value, we can make it look for the shortest distance by negating the distances
            // we negate the distances here to make them positive again
            // see "A Guide to Competitive Programming" by Antti Laaksonen for more details
            current.first = -current.first;

            // if this node was already processed, then skip
            if(graph[get_index(n, current.second)].processed) 
                continue;

            // mark this node as processed
            graph[get_index(n, current.second)].processed = true;

            // if we are at the goal, then we are done
            if(current.second.x == goal.x && current.second.y == goal.y)
                break;

            // visit all visitable adjacent nodes
            for(int i = 0; i < 8; i++) {
                Coordinate to = current.second;
                to.x += offsets[i].x;
                to.y += offsets[i].y;

                double path_length = (i % 2 == 1 ? sqrt(2) : 1) + current.first;
                double heuristic = abs(goal.x - to.x) + abs(goal.y - to.y); // use manhattan distance for our heuristic (this is the magic of the A* algorithm)
                double score = path_length + 0 * heuristic; 

                // if we get a better path with this vertex, let's push it on to the queue
                if(
                    edges[8 * get_index(n, current.second) + i] == 1 && 
                    !graph[get_index(n, to)].processed &&
                    score < graph[get_index(n, to)].distance
                    ) {
                    graph[get_index(n, to)].distance = score;
                    graph[get_index(n, to)].from = current.second;
                    work_queue.push({-graph[get_index(n, to)].distance, to});
                }
            }
        }

        // rebuild our path
        vector<Coordinate> reverse_path;
        Coordinate backtracker = goal;
        while(backtracker.x != start.x || backtracker.y != start.y) {
            reverse_path.push_back(backtracker);
            backtracker = graph[get_index(n, backtracker)].from;
        }
        reverse(reverse_path.begin(), reverse_path.end());
        full_path.insert(full_path.end(), reverse_path.begin(), reverse_path.end()); // record our path before reverse_path goes out of scope

        start = goal;
    }

    for(const auto& c : full_path) {
        cout << c.x + 1 << ' ' << c.y + 1 << '\n'; // bring back into 1-indexing 
    }
    
}