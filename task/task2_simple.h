// T2简化版: 纯贪心(按到达时间分批装车) + nearest-first趟内排序, 无SA优化
// 约束: deadline=INF, 容量 w_lim, 包裹在 Si 之后才能配送
// 目标: 最小化不满意度之和 Σ(Di - Si)
#pragma once
#include "../common/input_reader.h"
#include "../common/schedule_utils.h"
#include <vector>

struct Task2SimpleResult
{
    std::vector<Trip> trips;
    double total_dissatisfaction;
    std::vector<double> deliver_time;
    std::vector<double> dissatisfaction_per_pkg;
};

class Task2Simple
{
public:
    explicit Task2Simple(const input_data& data);
    Task2SimpleResult solve();

private:
    std::vector<package> pkgs;
    car c;
    AllPairResult ap;
    int n_pkg;

    std::vector<int> unique_dests(const std::vector<int>& pkg_ids);
    double trip_weight(const std::vector<int>& pkg_ids);

    // 趟内排序: 最近邻贪心
    std::vector<int> nearest_dest_order(const std::vector<int>& pkg_ids);

    // 评估: 返回总不满意度
    double evaluate_solution(
        const std::vector<std::vector<int>>& trip_plans,
        const std::vector<std::vector<int>>& dest_orders,
        std::vector<double>& deliver_time,
        std::vector<double>& dissatisfaction);

    // 贪心初解: 按包裹到达时间升序分批装车
    void greedy_init(std::vector<std::vector<int>>& trip_plans,
                     std::vector<std::vector<int>>& dest_orders);
};