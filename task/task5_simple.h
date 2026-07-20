// T5简化版: K-means分区 + EDD贪心分趟 + nearest-first趟内排序, 无SA/ALNS
// 双车独立配送, 各自按T3简化版逻辑
// 目标: 最小化总成本 (超时仅统计)
#pragma once
#include "../common/models.h"
#include "../common/graph.h"
#include "../common/schedule_utils.h"
#include "../common/input_reader.h"
#include <vector>
#include <random>

struct Task5SimpleResult
{
    std::vector<Trip> trips1;
    std::vector<Trip> trips2;
    double sumCost;
    int exTime;
};

class Task5Simple
{
public:
    explicit Task5Simple(const input_data& data);
    Task5SimpleResult solve();

private:
    graph g;
    std::vector<package> pkgs;
    car c1, c2;
    AllPairResult ap;
    int n_pkg;
    std::mt19937 rng;

    struct CarPlan
    {
        std::vector<std::vector<int>> trip_plans;
        std::vector<std::vector<int>> dest_orders;
    };

    // K-means分区 (k=2), 按目的地坐标
    std::pair<std::vector<int>, std::vector<int>> kmeans_partition();

    // 单车贪心: EDD分趟 + nearest-first趟内排序
    void greedy_init_one(const std::vector<int>& pkg_indices, CarPlan& plan, const car& c);

    // 辅助
    std::vector<int> unique_dests(const std::vector<int>& pkg_ids);
    double trip_weight(const std::vector<int>& pkg_ids);
    std::vector<int> nearest_dest_order(const std::vector<int>& pkg_ids);

    // 单车评估: 返回(成本, 超时数)
    std::pair<double, int> evaluate_car(
        const std::vector<std::vector<int>>& trip_plans,
        const std::vector<std::vector<int>>& dest_orders,
        std::vector<double>& deliver_time,
        const car& c);
};