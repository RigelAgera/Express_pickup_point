// T3简化版: 纯EDD贪心 + nearest-first趟内排序, 无SA优化
// 约束: Si=0, 容量 w_lim, 有 deadline Ti
// 评估: 字典序 (超时数, 总成本)
#pragma once
#include "../common/input_reader.h"
#include "../common/schedule_utils.h"
#include <vector>

struct Task3SimpleResult
{
    std::vector<Trip> trips;
    double sumCost;
    int exTime;
};

class Task3Simple
{
public:
    explicit Task3Simple(const input_data& data);
    Task3SimpleResult solve();

private:
    graph g;
    std::vector<package> pkgs;
    car c;
    AllPairResult ap;
    int n_pkg;

    std::vector<int> unique_dests(const std::vector<int>& pkg_ids);
    double trip_weight(const std::vector<int>& pkg_ids);

    // 趟内排序: 最近邻贪心
    std::vector<int> nearest_dest_order(const std::vector<int>& pkg_ids);

    // 评估: 返回 (超时数, 总成本)
    std::pair<int, double> evaluate_solution(
        const std::vector<std::vector<int>>& trip_plans,
        const std::vector<std::vector<int>>& dest_orders,
        std::vector<double>& deliver_time);

    // EDD贪心分趟
    void greedy_init(std::vector<std::vector<int>>& trip_plans,
                     std::vector<std::vector<int>>& dest_orders);
};