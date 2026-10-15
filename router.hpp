#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>
#include <string>
#include <cassert>
#include <vector>
#include <set>
#include <map>
#include <queue>
#include <string>

#ifndef ROUTER_HPP
#define ROUTER_HPP 1

struct Node {
    int x_idx; 
    int y_idx;   
    int layer; 

    bool operator==(const Node& other) const {
        return x_idx == other.x_idx && y_idx == other.y_idx && layer == other.layer;
    }
    bool operator<(const Node& other) const {
        if (layer != other.layer) return layer < other.layer;
        if (x_idx != other.x_idx) return x_idx < other.x_idx;
        return y_idx < other.y_idx;
    }
};

enum class EdgeType { HORIZONTAL, VERTICAL, VIA };

struct Edge {
    int id;
    Node u;
    Node v;
    EdgeType type;
    int weight; 
};

struct Net {
    int id;
    std::string name;
    Node source;
    Node target;
    
    int min_x_idx, max_x_idx;
    int min_y_idx, max_y_idx;
    int min_layer, max_layer;

    std::vector<Node> routed_path;
};

class Router {
private:
    int current_iteration = 0;
    int max_layer;
    std::vector<int> boundary; 
    int min_pitch_size;
    int via_cost;

    std::vector<int> x_coors;
    std::vector<int> y_coors;
    std::map<int, int> x_coord_to_idx; 
    std::map<int, int> y_coord_to_idx; 

    std::vector<Net> nets;
    std::vector<Edge> edges;
    
    std::vector<std::vector<int>> node_to_edge_ids; 

    int total_sat_vars;
    long long top_weight; 
    
    std::map<std::pair<int, int>, int> edge_net_to_var;

    std::map<std::pair<int, int>, int> node_net_to_var;
    
    std::map<int, std::pair<int, int>> var_to_edge_net;

    int getNodeIndex(const Node& n) const; 
    int getVarId(int edge_id, int net_id);
    int getNodeVarId(int node_idx, int net_id);
    void setupBoundingBoxes(); 
    
    void addHardClause(std::ofstream& out, const std::vector<int>& literals);
    void addSoftClause(std::ofstream& out, int weight, const std::vector<int>& literals);

public:
    Router() = default;
    void readCircuitSpec(std::ifstream& inputFile);
    void generateGraph();
    std::string getSysCommand(int);
    void generateClauses(std::ofstream& outputFile);
    bool readSolverResult(std::ifstream& inputFile, int);
    void printRoutingResult(std::ofstream& outputFile);
};
#endif