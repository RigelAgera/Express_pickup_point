// 标准TSP —— 遗传算法求解
#pragma once
#include "../common/models.h"
#include "../common/graph.h"
#include "../common/schedule_utils.h"
#include "../common/input_reader.h"
#include <vector>
#include <random>

using std::vector;

struct Task4Result
{
    vector<int> destination; // 访问退货地点的顺序（不含 0）
};

class Task4
{
    private:
        graph g;
        double v;
        AllPairResult ap;

        // 计算 order 对应回路的总距离（0 → order[0] → … → order.back() → 0）
        double tour_length(const vector<int>& order);
        // 最近邻贪心
        vector<int> nearest_neighbor_order(const vector<int>& nodes);
        // OX 顺序交叉
        vector<int> ox_crossover(const vector<int>& a, const vector<int>& b, std::mt19937& rng);
        // 复合变异
        void mutate(vector<int>& order, std::mt19937& rng);
        // 初始种群
        vector<vector<int>> init_population(const vector<int>& nodes, int pop_size, std::mt19937& rng);
        // 锦标赛选择
        int tournament_select(const vector<double>& fitness, std::mt19937& rng, int t_size = 3);
    public:
        Task4(const input_data& data);
        Task4Result solve(const vector<int>& return_nodes);
};
