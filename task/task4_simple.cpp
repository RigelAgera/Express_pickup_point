// T4简化版: 纯最近邻贪心求解 TSP
#include "task4_simple.h"
#include "../common/dijkstra.h"
#include <algorithm>

Task4Simple::Task4Simple(const input_data& data)
    : g(data.g), ap(all_pairs_dijkstra(data.g)) {}

double Task4Simple::tour_length(const std::vector<int>& order)
{
    if (order.empty()) return 0.0;
    double total = ap.dist[0][order[0]];
    for (size_t i = 0; i + 1 < order.size(); ++i)
        total += ap.dist[order[i]][order[i + 1]];
    total += ap.dist[order.back()][0];
    return total;
}

std::vector<int> Task4Simple::nearest_neighbor_order(const std::vector<int>& nodes)
{
    if (nodes.empty()) return {};
    std::vector<int> remaining = nodes;
    std::vector<int> result;
    int cur = 0;
    while (!remaining.empty())
    {
        int best_idx = 0;
        double best_dist = 1e18;
        for (size_t i = 0; i < remaining.size(); ++i)
        {
            double d = ap.dist[cur][remaining[i]];
            if (d < best_dist)
            {
                best_dist = d;
                best_idx = (int)i;
            }
        }
        cur = remaining[best_idx];
        result.push_back(cur);
        remaining.erase(remaining.begin() + best_idx);
    }
    return result;
}

Task4SimpleResult Task4Simple::solve(const std::vector<int>& return_nodes)
{
    Task4SimpleResult result;
    result.destination = nearest_neighbor_order(return_nodes);
    return result;
}