// T2: 最小化不满意度之和
// 约束：不考虑超时（Ti=INF），包裹在其 Si 之后才可配送，容量 w_lim 约束
// 算法: 贪心初解 + 模拟退火(SA) 优化
// 编译: g++ -std=c++17 -O2 task/task2.cpp common/input_reader.cpp -o task2.exe
#pragma once
#include <vector>
#include "../common/input_reader.h"
#include "../common/schedule_utils.h"
#include <random>

// ---- T2 结果 ----
struct Task2Result
{
    std::vector<Trip> trips;
    double total_dissatisfaction;
    std::vector<double> deliver_time;           // 每个包裹的送达时间 Di（按原始 id 索引）
    std::vector<double> dissatisfaction_per_pkg; // 每个包裹的不满意度 Di - Si
};

// ---- Task2 类 ----
class Task2
{
public:
    explicit Task2(const input_data& data);
    Task2Result solve();

private:
    std::vector<package> pkgs;
    car c;
    AllPairResult ap;
    int n_pkg;
    std::mt19937 rng;

    // 辅助
    // 返回 pkg_ids 中去重后的目的地列表
    std::vector<int> unique_dests(const std::vector<int>& pkg_ids);
    // 计算 pkg_ids 对应包裹的总重量
    double trip_weight(const std::vector<int>& pkg_ids);

    // 核心
    // 评估一组 trip 的总不满意度：按 dest_orders 顺序递送，计算每个包裹的送达时间与不满意度
    double evaluate_solution(
        const std::vector<std::vector<int>>& trip_plans,
        const std::vector<std::vector<int>>& dest_orders,
        std::vector<double>& deliver_time,
        std::vector<double>& dissatisfaction);

    // 为给定的包裹列表生成最近邻目的地的访问顺序
    std::vector<int> nearest_dest_order(const std::vector<int>& pkg_ids);

    // 算法
    // 贪心构造初始解：按包裹到达时间分批，每趟尽量装满
    void greedy_init(std::vector<std::vector<int>>& trip_plans,
                     std::vector<std::vector<int>>& dest_orders);

    // 尝试一次随机扰动（迁移包裹/调整顺序），若更优则接受
    bool try_random_move(std::vector<std::vector<int>>& trip_plans,
                         std::vector<std::vector<int>>& dest_orders);

    // 模拟退火主循环：逐步降温，通过随机扰动搜索更优解
    void sa_optimize(std::vector<std::vector<int>>& trip_plans,
                     std::vector<std::vector<int>>& dest_orders);
};