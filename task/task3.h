// T3带容量运送成本
// 约束：Si=0, 容量 w_lim, 有 deadline Ti
// 算法: EDD贪心分趟 + heaviest-first趟内排序 + SA优化
// 评估: 字典序 (超时数, 总成本)，优先压超时、同超时下压成本
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
    std::vector<int> unique_dests(const std::vector<int>& pkg_ids); // 返回目的地？
    double trip_weight(const std::vector<int>& pkg_ids);    // 返回一趟所有包裹的重量之和
    bool try_random_move(std::vector<std::vector<int>>& trip_plans,
                         std::vector<std::vector<int>>& dest_orders);   // 邻域移动

    // ---- 趟内排序: 最重优先 ----
    std::vector<int> heaviest_dest_order(const std::vector<int>& pkg_ids);

    // ---- 评估: 返回 (超时数, 总成本) 字典序 ----
    // 需要送达时间，目的地顺序，trip_plans是何意味？
    std::pair<int, double> evaluate_solution(
        const std::vector<std::vector<int>>& trip_plans,    // 每趟的包裹 id，无序
        const std::vector<std::vector<int>>& dest_orders,
        std::vector<double>& deliver_time);

    // ---- 贪心初解: 按 deadline 升序贪心装车 ----
    void greedy_init(std::vector<std::vector<int>>& trip_plans,
                     std::vector<std::vector<int>>& dest_orders);

    // ---- 模拟退火 ----
    void sa_optimize(std::vector<std::vector<int>>& trip_plans,
                     std::vector<std::vector<int>>& dest_orders);
};