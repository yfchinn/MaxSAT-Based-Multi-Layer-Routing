#include "router.hpp"
#include <sstream>
#include <cmath>
#include <set>
#include <map>
#include <queue>
#include <iostream>
#include <cassert>

int Router::getNodeIndex(const Node& n) const {
    return n.layer * (x_coors.size() * y_coors.size()) + 
           n.x_idx * y_coors.size() + 
           n.y_idx;
}

int Router::getVarId(int edge_id, int net_id) {
    auto key = std::make_pair(edge_id, net_id);
    if (edge_net_to_var.find(key) != edge_net_to_var.end()) {
        return edge_net_to_var[key];
    }
    return -1;
}

void Router::addHardClause(std::ofstream& out, const std::vector<int>& literals) {
    out << top_weight << " ";
    for (int lit : literals) out << lit << " ";
    out << "0\n";
}

void Router::addSoftClause(std::ofstream& out, int weight, const std::vector<int>& literals) {
    out << weight << " ";
    for (int lit : literals) out << lit << " ";
    out << "0\n";
}

int Router::getNodeVarId(int node_idx, int net_id) {
    auto key = std::make_pair(node_idx, net_id);
    if (node_net_to_var.find(key) != node_net_to_var.end()) {
        return node_net_to_var[key];
    }
    return -1;
}

void Router::readCircuitSpec(std::ifstream& inputFile) {
    std::string token;
    
    inputFile >> token >> max_layer;

    boundary.resize(4);
    
    inputFile >> token >> boundary[0] >> boundary[1] >> boundary[2] >> boundary[3];
    
    inputFile >> token >> min_pitch_size;
    inputFile >> token >> via_cost;
    
    int num_nets;
    inputFile >> token >> num_nets;
    nets.resize(num_nets);
    
    std::set<int> unique_x;
    std::set<int> unique_y;

    for (int i = 0; i < num_nets; ++i) {
        nets[i].id = i;
        inputFile >> nets[i].name 
                  >> nets[i].source.x_idx >> nets[i].source.y_idx >> nets[i].source.layer 
                  >> nets[i].target.x_idx >> nets[i].target.y_idx >> nets[i].target.layer;
        
        unique_x.insert(nets[i].source.x_idx);
        unique_x.insert(nets[i].target.x_idx);
        unique_y.insert(nets[i].source.y_idx);
        unique_y.insert(nets[i].target.y_idx);
    }

    int min_x_pin = *unique_x.begin();
    int max_x_pin = *unique_x.rbegin();
    int min_y_pin = *unique_y.begin();
    int max_y_pin = *unique_y.rbegin();

    for (int x = min_x_pin - min_pitch_size; x >= boundary[0]; x -= min_pitch_size) unique_x.insert(x);
    for (int x = max_x_pin + min_pitch_size; x <= boundary[2]; x += min_pitch_size) unique_x.insert(x);

    for (int y = min_y_pin - min_pitch_size; y >= boundary[1]; y -= min_pitch_size) unique_y.insert(y);
    for (int y = max_y_pin + min_pitch_size; y <= boundary[3]; y += min_pitch_size) unique_y.insert(y);

    std::vector<int> sorted_x(unique_x.begin(), unique_x.end());
    std::vector<int> sorted_y(unique_y.begin(), unique_y.end());

    std::set<int> final_x(unique_x.begin(), unique_x.end());
    std::set<int> final_y(unique_y.begin(), unique_y.end());

    for (size_t i = 0; i < sorted_x.size() - 1; ++i) {
        int start = sorted_x[i];
        int end = sorted_x[i+1];
        int current = start + min_pitch_size;
        
        while (end - current >= min_pitch_size) {
            final_x.insert(current);
            current += min_pitch_size;
        }
    }

    for (size_t i = 0; i < sorted_y.size() - 1; ++i) {
        int start = sorted_y[i];
        int end = sorted_y[i+1];
        int current = start + min_pitch_size;
        
        while (end - current >= min_pitch_size) {
            final_y.insert(current);
            current += min_pitch_size;
        }
    }

    for (int x : final_x) x_coors.push_back(x);
    for (int y : final_y) y_coors.push_back(y);
    
    for (int i = 0; i < (int)x_coors.size(); ++i) x_coord_to_idx[x_coors[i]] = i;
    for (int i = 0; i < (int)y_coors.size(); ++i) y_coord_to_idx[y_coors[i]] = i;
    
    for (int i = 0; i < num_nets; ++i) {
        nets[i].source.x_idx = x_coord_to_idx[nets[i].source.x_idx];
        nets[i].source.y_idx = y_coord_to_idx[nets[i].source.y_idx];
        nets[i].target.x_idx = x_coord_to_idx[nets[i].target.x_idx];
        nets[i].target.y_idx = y_coord_to_idx[nets[i].target.y_idx];
    } 
}

void Router::setupBoundingBoxes() {
    int padding = current_iteration; 
    
    for (auto& net : nets) {
        net.min_x_idx = std::max(0, std::min(net.source.x_idx, net.target.x_idx) - padding);
        net.max_x_idx = std::min((int)x_coors.size() - 1, std::max(net.source.x_idx, net.target.x_idx) + padding);
        
        net.min_y_idx = std::max(0, std::min(net.source.y_idx, net.target.y_idx) - padding);
        net.max_y_idx = std::min((int)y_coors.size() - 1, std::max(net.source.y_idx, net.target.y_idx) + padding);
        
        net.min_layer = 0;
        net.max_layer = max_layer - 1; 
    }
}

void Router::generateGraph() {
    edges.clear();
    node_to_edge_ids.clear();
    edge_net_to_var.clear();
    var_to_edge_net.clear();
    node_net_to_var.clear();
    setupBoundingBoxes();
    current_iteration++;
    
    int num_x = x_coors.size();
    int num_y = y_coors.size();
    int total_nodes = max_layer * num_x * num_y;
    node_to_edge_ids.resize(total_nodes);
    
    int edge_id_counter = 0;
    
    // Create 3D grid edges
    for (int l = 0; l < max_layer; ++l) {
        for (int x = 0; x < num_x; ++x) {
            for (int y = 0; y < num_y; ++y) {
                Node u = {x, y, l};
                
                // Horizontal Edge (X-direction)
                if (x + 1 < num_x) {
                    Node v = {x + 1, y, l};
                    Edge e = {edge_id_counter++, u, v, EdgeType::HORIZONTAL, std::abs(x_coors[x+1] - x_coors[x])};
                    edges.push_back(e);
                    node_to_edge_ids[getNodeIndex(u)].push_back(e.id);
                    node_to_edge_ids[getNodeIndex(v)].push_back(e.id);
                }
                
                // Vertical Edge (Y-direction)
                if (y + 1 < num_y) {
                    Node v = {x, y + 1, l};
                    Edge e = {edge_id_counter++, u, v, EdgeType::VERTICAL, std::abs(y_coors[y+1] - y_coors[y])};
                    edges.push_back(e);
                    node_to_edge_ids[getNodeIndex(u)].push_back(e.id);
                    node_to_edge_ids[getNodeIndex(v)].push_back(e.id);
                }
                
                // Via Edge (Z-direction)
                if (l + 1 < max_layer) {
                    Node v = {x, y, l + 1};
                    Edge e = {edge_id_counter++, u, v, EdgeType::VIA, via_cost};
                    edges.push_back(e);
                    node_to_edge_ids[getNodeIndex(u)].push_back(e.id);
                    node_to_edge_ids[getNodeIndex(v)].push_back(e.id);
                }
            }
        }
    }
    
    // Create SAT variable mappings 
    total_sat_vars = 0;
    long long total_soft_weight = 0;
    
    for (const auto& net : nets) {
        for (const auto& e : edges) {
            bool in_x = (e.u.x_idx >= net.min_x_idx && e.u.x_idx <= net.max_x_idx) && 
                        (e.v.x_idx >= net.min_x_idx && e.v.x_idx <= net.max_x_idx);
            bool in_y = (e.u.y_idx >= net.min_y_idx && e.u.y_idx <= net.max_y_idx) && 
                        (e.v.y_idx >= net.min_y_idx && e.v.y_idx <= net.max_y_idx);
                        
            if (in_x && in_y) {
                total_sat_vars++;
                edge_net_to_var[{e.id, net.id}] = total_sat_vars;
                var_to_edge_net[total_sat_vars] = {e.id, net.id};
                total_soft_weight += e.weight;
            }
        }
    }

    for (const auto& net : nets) {
        for (int l = net.min_layer; l <= net.max_layer; ++l) {
            for (int x = net.min_x_idx; x <= net.max_x_idx; ++x) {
                for (int y = net.min_y_idx; y <= net.max_y_idx; ++y) {
                    Node curr_node = {x, y, l};
                    int node_idx = getNodeIndex(curr_node);
                    total_sat_vars++;
                    node_net_to_var[{node_idx, net.id}] = total_sat_vars;
                }
            }
        }
    }

    top_weight = total_soft_weight + 999999;
}

std::string Router::getSysCommand(int iteration) {
    return "./open-wbo -cpu-lim=480 clause.sat > sat_result.txt";
}

void Router::generateClauses(std::ofstream& outputFile) {
    std::vector<std::vector<int>> hard_clauses;
    std::vector<std::pair<int, std::vector<int>>> soft_clauses;

    bool all_nets_straight_same_layer = true;
    for (const auto& net : nets) {
        bool same_layer = (net.source.layer == net.target.layer);
        bool straight_line = (net.source.x_idx == net.target.x_idx) || 
                             (net.source.y_idx == net.target.y_idx);
        
        if (!same_layer || !straight_line) {
            all_nets_straight_same_layer = false;
            break;
        }
    }
    int penalty_weight = all_nets_straight_same_layer ? 0 : min_pitch_size; 
    int turn_cost = via_cost;

    for (const auto& net : nets) {
        int min_target_layer = std::min(net.source.layer, net.target.layer);
        int max_target_layer = std::max(net.source.layer, net.target.layer);
        
        for (const auto& e : edges) {
            int var_id = getVarId(e.id, net.id);
            if (var_id != -1) {
                int current_weight = e.weight;

                // Even layers prefer horizontal routing; odd layers prefer vertical routing
                if (e.type == EdgeType::VERTICAL && (e.u.layer % 2 == 0)) {
                    current_weight += penalty_weight;
                } 
                else if (e.type == EdgeType::HORIZONTAL && (e.u.layer % 2 == 1)) {
                    current_weight += penalty_weight;
                }
                
                // Soft Clause: Discourage using this edge to minimize wirelength and vias
                soft_clauses.push_back({current_weight, {-var_id}});
            }
        }
    }

    int num_x = x_coors.size();
    int num_y = y_coors.size();
    
    for (const auto& net : nets) {
        for (int l = 0; l < max_layer; ++l) {
            for (int x = net.min_x_idx; x <= net.max_x_idx; ++x) {
                for (int y = net.min_y_idx; y <= net.max_y_idx; ++y) {
                    Node curr_node = {x, y, l};
                    const auto& connected_edges = node_to_edge_ids[getNodeIndex(curr_node)];
                    
                    std::vector<int> active_vars;
                    std::vector<int> h_vars; 
                    std::vector<int> v_vars; 
                    for (int eid : connected_edges) {
                        int var_id = getVarId(eid, net.id);
                        if (var_id != -1) {
                            active_vars.push_back(var_id);
                            if (edges[eid].type == EdgeType::HORIZONTAL) {
                                h_vars.push_back(var_id);
                            } else if (edges[eid].type == EdgeType::VERTICAL) {
                                v_vars.push_back(var_id);
                            }
                        }
                    }
                    
                    if (active_vars.empty()) continue;

                    // Soft Clause: turn_cost
                    if(via_cost == 10000){
                        for (int h_var : h_vars) {
                            for (int v_var : v_vars) {
                                soft_clauses.push_back({turn_cost, {-h_var, -v_var}});
                            }
                        }
                    }
                    
                    bool is_pin = (curr_node == net.source || curr_node == net.target);

                    if (is_pin) {
                        // Constraint: A pin must connect to exactly one edge
                        hard_clauses.push_back(active_vars); // At least 1
                        for (size_t i = 0; i < active_vars.size(); ++i) {
                            for (size_t j = i + 1; j < active_vars.size(); ++j) {
                                hard_clauses.push_back({-active_vars[i], -active_vars[j]}); // At most 1
                            }
                        }
                    } else {
                        // Constraint: An internal node must connect to two edges if used (Degree = 0 or 2)
                        for (size_t i = 0; i < active_vars.size(); ++i) {
                            std::vector<int> clause = {-active_vars[i]};
                            for (size_t j = 0; j < active_vars.size(); ++j) {
                                if (i != j) clause.push_back(active_vars[j]);
                            }
                            hard_clauses.push_back(clause);
                        }
                        // No three edges may be used simultaneously
                        for (size_t i = 0; i < active_vars.size(); ++i) {
                            for (size_t j = i + 1; j < active_vars.size(); ++j) {
                                for (size_t k = j + 1; k < active_vars.size(); ++k) {
                                    hard_clauses.push_back({-active_vars[i], -active_vars[j], -active_vars[k]});
                                }
                            }
                        }
                    }
                }
            }
        }

        // Edge -> Node implication: If a net uses edge E, it occupies nodes U and V: (-E v U) and (-E v V)
        for (const auto& net : nets) {
            for (int l = net.min_layer; l <= net.max_layer; ++l) {
                for (int x = net.min_x_idx; x <= net.max_x_idx; ++x) {
                    for (int y = net.min_y_idx; y <= net.max_y_idx; ++y) {
                        Node curr_node = {x, y, l};
                        int node_idx = getNodeIndex(curr_node);
                        int node_var = getNodeVarId(node_idx, net.id);
                        
                        if (node_var != -1) {
                            for (int eid : node_to_edge_ids[node_idx]) {
                                int e_var = getVarId(eid, net.id);
                                if (e_var != -1) {
                                    hard_clauses.push_back({-e_var, node_var});
                                }
                            }
                        }
                    }
                }
            }
        }

        int strict_min_x = std::min(net.source.x_idx, net.target.x_idx);
        int strict_max_x = std::max(net.source.x_idx, net.target.x_idx);
        int strict_min_y = std::min(net.source.y_idx, net.target.y_idx);
        int strict_max_y = std::max(net.source.y_idx, net.target.y_idx);

        // Eq 21: Each column within the MRRR must be crossed by at least one horizontal edge
        for (int x = strict_min_x; x < strict_max_x; ++x) {
            std::vector<int> col_h_edges;
            for (int y = net.min_y_idx; y <= net.max_y_idx; ++y) {
                for (int l = net.min_layer; l <= net.max_layer; ++l) {
                    Node curr_node = {x, y, l};
                    int node_idx = getNodeIndex(curr_node);
                    for (int eid : node_to_edge_ids[node_idx]) {
                        const Edge& e = edges[eid];
                        if (e.type == EdgeType::HORIZONTAL && std::min(e.u.x_idx, e.v.x_idx) == x) {
                            int var_id = getVarId(e.id, net.id);
                            if (var_id != -1) col_h_edges.push_back(var_id);
                        }
                    }
                }
            }
            if (!col_h_edges.empty()) {
                hard_clauses.push_back(col_h_edges);
            }
        }

        // Eq 22: Each row within the MRRR must be crossed by at least one vertical edge
        for (int y = strict_min_y; y < strict_max_y; ++y) {
            std::vector<int> row_v_edges;
            for (int x = net.min_x_idx; x <= net.max_x_idx; ++x) {
                for (int l = net.min_layer; l <= net.max_layer; ++l) {
                    Node curr_node = {x, y, l};
                    int node_idx = getNodeIndex(curr_node);
                    for (int eid : node_to_edge_ids[node_idx]) {
                        const Edge& e = edges[eid];
                        if (e.type == EdgeType::VERTICAL && std::min(e.u.y_idx, e.v.y_idx) == y) {
                            int var_id = getVarId(e.id, net.id);
                            if (var_id != -1) {
                                row_v_edges.push_back(var_id);
                            }
                        }
                    }
                }
            }
            if (!row_v_edges.empty()) {
                hard_clauses.push_back(row_v_edges);
            }
        }
    }


    // Node-based mutual exclusion + Sequential Counter Encoding (Ladder Encoding)
    for (int l = 0; l < max_layer; ++l) {
        for (int x = 0; x < num_x; ++x) {
            for (int y = 0; y < num_y; ++y) {
                Node curr_node = {x, y, l};
                int node_idx = getNodeIndex(curr_node);

                std::vector<int> active_node_vars;
                for (size_t i = 0; i < nets.size(); ++i) {
                    int n_var = getNodeVarId(node_idx, nets[i].id);
                    if (n_var != -1) {
                        active_node_vars.push_back(n_var);
                    }
                }

                int k = active_node_vars.size();
                if (k <= 1) continue; 

                // At-Most-One (AMO) 
                if (k == 2) {
                    hard_clauses.push_back({-active_node_vars[0], -active_node_vars[1]});
                } else {
                    std::vector<int> s_vars(k - 1);
                    for (int i = 0; i < k - 1; ++i) {
                        total_sat_vars++;
                        s_vars[i] = total_sat_vars;
                    }
                    hard_clauses.push_back({-active_node_vars[0], s_vars[0]});
                    for (int i = 1; i < k - 1; ++i) {
                        hard_clauses.push_back({-active_node_vars[i], s_vars[i]});
                        hard_clauses.push_back({-s_vars[i-1], s_vars[i]});
                        hard_clauses.push_back({-active_node_vars[i], -s_vars[i-1]});
                    }
                    hard_clauses.push_back({-active_node_vars[k-1], -s_vars[k-2]});
                }
            }
        }
    }
            

    // Write the WCNF file
    long long actual_total_soft_weight = 0;
    for (const auto& pair : soft_clauses) {
        actual_total_soft_weight += pair.first;
    }
    top_weight = actual_total_soft_weight + 1; 
    outputFile << "p wcnf " << total_sat_vars << " " << hard_clauses.size() + soft_clauses.size() << " " << top_weight << "\n";

    for (const auto& clause : hard_clauses) {
        addHardClause(outputFile, clause);
    }
    for (const auto& pair : soft_clauses) {
        addSoftClause(outputFile, pair.first, pair.second);
    }
}

bool Router::readSolverResult(std::ifstream& inputFile, int iteration) {
    std::string line;
    std::vector<int> active_vars;
    bool optimum_found = false;

    while (std::getline(inputFile, line)) {
        if (line.find("s OPTIMUM FOUND") != std::string::npos || line.find("s SATISFIABLE") != std::string::npos) {
            optimum_found = true;
        } else if (line[0] == 'v') {
            std::stringstream ss(line.substr(2));
            int var;
            while (ss >> var) {
                if (var > 0) active_vars.push_back(var);
            }
        }
    }

    if (!optimum_found) {
        std::cerr << "Solver couldn't find a solution in this iteration.\n";
        return false;
    }

    std::vector<std::vector<Edge>> net_edges(nets.size());
    for (int var : active_vars) {
        if (var_to_edge_net.find(var) != var_to_edge_net.end()) {
            int edge_id = var_to_edge_net[var].first;
            int net_id = var_to_edge_net[var].second;
            net_edges[net_id].push_back(edges[edge_id]);
        }
    }

    // Perform DFS to reconstruct the path for each net
    for (auto& net : nets) {
        net.routed_path.clear();
        Node current = net.source;
        net.routed_path.push_back(current);
        
        std::set<int> used_edges;
        while (!(current == net.target)) {
            bool found_next = false;
            for (const auto& e : net_edges[net.id]) {
                if (used_edges.count(e.id)) continue;
                
                if (e.u == current) {
                    current = e.v;
                    net.routed_path.push_back(current);
                    used_edges.insert(e.id);
                    found_next = true;
                    break;
                } else if (e.v == current) {
                    current = e.u;
                    net.routed_path.push_back(current);
                    used_edges.insert(e.id);
                    found_next = true;
                    break;
                }
            }
            if (!found_next) break; 
        }
    }
    
    return true; 
}

void Router::printRoutingResult(std::ofstream& outputFile) {
    outputFile << "x_coors " << x_coors.size() << "\n";
    for (int x : x_coors) outputFile << x << " ";
    outputFile << "\n";
    
    outputFile << "y_coors " << y_coors.size() << "\n";
    for (int y : y_coors) outputFile << y << " ";
    outputFile << "\n";
    
    for (const auto& net : nets) {
        outputFile << net.name << " " << net.routed_path.size() << "\n";
        for (const auto& node : net.routed_path) {
            outputFile << node.x_idx << " " << node.y_idx << " " << node.layer << "\n";
        }
    }
}