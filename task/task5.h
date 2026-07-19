// T5: 双车协同配送 (2-CVRP + deadline)
// 算法: K-means 初始分区 + ALNS 扰动 + SA 温控
// 优化目标: total_cost = 车1成本 + 车2成本, 越小越好
// 超时包裹数仅作统计项, 不参与 SA 评分
#pragma once
#include "../common/models.h"
#include "../common/graph.h"
#include "../common/schedule_utils.h"
#include "../common/input_reader.h"
#include <vector>
#include <random>

using std::vector;

struct Task5Result
{
    vector<Trip> trips1;   // 车1的各趟配送
    vector<Trip> trips2;   // 车2的各趟配送
    double sumCost;        // 总运送成本 (两车成本之和)
    int exTime;            // 超时包裹数 (仅统计)
};

class Task5
{
private:
    graph g;
    vector<package> pkgs;
    car c1, c2;
    AllPairResult ap;
    int n_pkg;
    std::mt19937 rng;

    // ---- 已构造的 Trip 结果 (供 compute_cost 使用) ----
    vector<Trip> trips1;
    vector<Trip> trips2;

    // ---- 单辆车的趟计划 ----
    struct CarPlan
    {
        vector<vector<int>> trip_plans;  // 每趟的包裹 id 列表
        vector<vector<int>> dest_orders; // 每趟的目的地顺序
    };

    // ========== K-means 初始分区 ==========
    // 返回 (car0_pkg_ids, car1_pkg_ids)
    std::pair<vector<int>, vector<int>> kmeans_partition();

    // ========== 贪心初解 (单车) ==========
    // 与 T3 完全一致: EDD 分趟 + heaviest-first 趟内排序
    void greedy_init_one(const vector<int>& pkg_indices, CarPlan& plan);

    // ========== 辅助函数 ==========
    vector<int> unique_dests(const vector<int>& pkg_ids);
    double trip_weight(const vector<int>& pkg_ids);
    vector<int> heaviest_dest_order(const vector<int>& pkg_ids);

    // ========== 评估函数 ==========
    // 评估单车方案, 返回 (成本, 超时数), 填入送达时间
    std::pair<double, int> evaluate_car(
        const vector<vector<int>>& trip_plans,
        const vector<vector<int>>& dest_orders,
        vector<double>& deliver_time,
        const car& c);

    // 评估双车方案, 返回 (总成本, 总超时)
    std::pair<double, int> evaluate_full(
        const CarPlan& car0, const CarPlan& car1,
        vector<double>& deliv0, vector<double>& deliv1);

    // ========== ALNS 破坏算子 (Destroy) ==========
    // 各算子修改 cars[2] 并返回被移除的包裹 id 列表 (插入池)
    // 均使用廉价代理指标, 不跑完整 evaluate

    // Worst-removal (代理版): 按 dist[0][dest] * weight 排序, 移除分数最高的 k 个
    vector<int> destroy_worst(CarPlan cars[2]);

    // Shaw-removal: 移除与随机种子最相关的 k-1 个包裹
    // 相关度 = γ1×地理距离 + γ2×deadline差异 + γ3×重量差异 (归一化加权)
    vector<int> destroy_shaw(CarPlan cars[2]);

    // Trip-destroy: 随机选一辆车的一整趟, 全部移除
    vector<int> destroy_trip(CarPlan cars[2]);

    // Time-window-removal: 移除所有当前超时的包裹 + 随机同趟包裹
    vector<int> destroy_time_window(CarPlan cars[2]);

    // ========== ALNS 修复算子 (Repair) ==========
    // 将 pool 中的包裹逐个插回 cars[2], 支持跨车插入

    // Greedy-cheapest (代理版): 每次选 Δ 最小的位置插入
    void repair_greedy(vector<int>& pool, CarPlan cars[2]);

    // Regret-2: 用 regret 值决定插入顺序 (最佳Δ - 次佳Δ)
    void repair_regret2(vector<int>& pool, CarPlan cars[2]);

    // ========== SA 优化主循环 ==========
    void sa_optimize(CarPlan cars[2]);

public:
    explicit Task5(const input_data& data);
    Task5Result solve();
    double compute_cost();
};