
// T3带容量运送成本
// 约束：Si=0, 容量 w_lim, 有 deadline Ti
// 算法: EDD贪心分趟 + 双初解择优(nearest/heaviest) + 多邻域SA优化
// 评估: 用大 M 加权评分近似字典序，优先压超时、同超时下压成本
#pragma once
#include "../common/input_reader.h"
#include "../common/schedule_utils.h"
#include <vector>
#include <random>

struct Task3Result
{
    std::vector<Trip> trips; // 一趟配送的描述
    double sumCost;             // 总运送成本 (int)
    int exTime;              // 超时包裹数
};

class Task3
{
public:
    explicit Task3(const input_data& data);
    Task3Result solve();

private:
    graph g;
    std::vector<package> pkgs;
    car c;
    AllPairResult ap;
    int n_pkg;
    std::mt19937 rng;   // 伪随机数生成器

    // ---- 辅助 ----
    std::vector<int> unique_dests(const std::vector<int>& pkg_ids); // 返回一趟涉及的去重目的地
    double trip_weight(const std::vector<int>& pkg_ids);             // 返回一趟所有包裹的重量之和
    bool try_random_move(std::vector<std::vector<int>>& trip_plans,
                         std::vector<std::vector<int>>& dest_orders); // 多邻域随机扰动

    // ---- 趟内排序: 按目的地总重量降序 ----
    std::vector<int> heaviest_dest_order(const std::vector<int>& pkg_ids);

    // ---- 评估: 返回 (超时数, 总成本) ----
    std::pair<int, double> evaluate_solution(
        const std::vector<std::vector<int>>& trip_plans,     // 每趟承运的包裹 id 集合
        const std::vector<std::vector<int>>& dest_orders,    // 每趟访问目的地顺序
        std::vector<double>& deliver_time);

    // ---- 贪心初解: 按 deadline 升序贪心装车，先生成 heaviest-first 顺序 ----
    void greedy_init(std::vector<std::vector<int>>& trip_plans,
                     std::vector<std::vector<int>>& dest_orders);

    // ---- 模拟退火 ----
    void sa_optimize(std::vector<std::vector<int>>& trip_plans,
                     std::vector<std::vector<int>>& dest_orders);
};