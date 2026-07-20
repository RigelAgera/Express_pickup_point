// T4简化版: 纯最近邻贪心求解 TSP, 无遗传算法
#pragma once
#include "../common/models.h"
#include "../common/graph.h"
#include "../common/schedule_utils.h"
#include "../common/input_reader.h"
#include <vector>

struct Task4SimpleResult
{
    std::vector<int> destination; // 访问退货地点的顺序（不含 0）
};

class Task4Simple
{
public:
    explicit Task4Simple(const input_data& data);
    Task4SimpleResult solve(const std::vector<int>& return_nodes);
    double tour_length(const std::vector<int>& order);

private:
    graph g;
    AllPairResult ap;

    std::vector<int> nearest_neighbor_order(const std::vector<int>& nodes);
};
