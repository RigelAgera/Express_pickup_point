// T5: 双车协同配送 (2-CVRP + deadline)
// 算法框架: K-means 初始分区 → 贪心初解 → ALNS 产生新解 → SA 决定接受/拒绝
// 优化目标: score = total_cost + overtime_penalty * overtime_count
#include "task5.h"
#include "../common/dijkstra.h"
#include "../common/schedule_utils.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <unordered_set>

using std::vector;

namespace
{
// 加权评分: 总成本 + 超时罚分 × 超时包裹数
double weighted_score(double cost, int overtime, double overtime_penalty)
{
    return cost + overtime_penalty * static_cast<double>(overtime);
}

// 比较两个解: 先比加权评分, 再比超时数, 再比成本
bool better_solution(double lhs_cost, int lhs_ot,
                     double rhs_cost, int rhs_ot,
                     double overtime_penalty)
{
    const double EPS = 1e-9;
    double lhs_score = weighted_score(lhs_cost, lhs_ot, overtime_penalty);
    double rhs_score = weighted_score(rhs_cost, rhs_ot, overtime_penalty);
    if (lhs_score < rhs_score - EPS) return true;
    if (lhs_score > rhs_score + EPS) return false;
    if (lhs_ot != rhs_ot) return lhs_ot < rhs_ot;
    return lhs_cost < rhs_cost - EPS;
}
}

// ====================================================================
// 构造函数
// ====================================================================
Task5::Task5(const input_data& data, double overtime_penalty)
    : g(data.g), pkgs(data.packages), c1(data.c), c2(data.c),         // 图, 包裹列表, 两辆车(同型号)
      ap(all_pairs_dijkstra(data.g)),                                 // 全源最短路结果: ap.dist[i][j] 为最短距离
      n_pkg((int)data.packages.size()),                                // 包裹总数
    rng(std::random_device{}()),                                      // Mersenne Twister 随机数生成器
    overtime_penalty(overtime_penalty)                                 // 超时罚分系数, 用于权衡成本与守时
{
}

// ====================================================================
// 辅助函数 (与 T3 一致)
// ====================================================================

// 返回 pkg_ids 中去重后的目的地节点列表
vector<int> Task5::unique_dests(const vector<int>& pkg_ids)
{
    vector<int> dests;
    for (int pid : pkg_ids) dests.push_back(pkgs[pid].dest);
    std::sort(dests.begin(), dests.end());
    dests.erase(std::unique(dests.begin(), dests.end()), dests.end());
    return dests;
}

// 返回一趟所有包裹的总重量
double Task5::trip_weight(const vector<int>& pkg_ids)
{
    double w = 0.0;
    for (int pid : pkg_ids) w += pkgs[pid].weight;
    return w;
}

// 趟内排序: 按目的地总重量降序, 重者先送 (减少搬运耗能)
vector<int> Task5::heaviest_dest_order(const vector<int>& pkg_ids)
{
    vector<int> dests = unique_dests(pkg_ids);
    vector<std::pair<double, int>> scored;  // (总重量, 目的地节点id)
    for (int d : dests)
    {
        double tw = 0.0;  // 该目的地的包裹总重
        for (int pid : pkg_ids)
            if (pkgs[pid].dest == d) tw += pkgs[pid].weight;
        scored.emplace_back(tw, d);
    }
    std::sort(scored.rbegin(), scored.rend());  // 降序
    vector<int> order;
    for (auto& p : scored) order.push_back(p.second);
    return order;
}

// ====================================================================
// 评估: 单车方案 (成本, 超时数)
// trip_plans: 每趟包裹id列表, dest_orders: 每趟目的地顺序
// deliver_time: 输出, 每个包裹的送达时间 (未分配为 -1)
// c: 执行该方案的车
// ====================================================================
std::pair<double, int> Task5::evaluate_car(
    const vector<vector<int>>& trip_plans,
    const vector<vector<int>>& dest_orders,
    vector<double>& deliver_time,
    const car& c)
{
    deliver_time.assign(n_pkg, -1.0);  // 初始化为未送达
    double cur_time = 0.0;              // 当前时刻 (从驿站出发)
    double total_cost = 0.0;            // 累计成本
    int overtime = 0;                   // 超时包裹计数

    for (size_t t = 0; t < trip_plans.size(); ++t)
    {
        const auto& pkg_ids = trip_plans[t];
        const auto& dest_order = dest_orders[t];
        if (pkg_ids.empty()) continue;

        // 构建路线 & 模拟本趟
        vector<int> route = build_route(dest_order, ap);           // 由目的地顺序生成完整路径
        Trip sim = simulate_trip(pkg_ids, route, cur_time, pkgs, c, ap);  // 模拟一趟, 返回 Trip 结构

        cur_time = sim.end_time;       // 本趟结束后回到驿站的时刻
        total_cost += trip_cost(sim, g, pkgs, c, ap);  // 累加本趟成本 (行驶 + 搬运)

        // 记录送达时间, 统计超时
        for (size_t i = 0; i < pkg_ids.size(); ++i)
        {
            int pid = pkg_ids[i];
            deliver_time[pid] = sim.deliver_time[i];
            if (sim.deliver_time[i] > pkgs[pid].deadline + 1e-9)
                ++overtime;
        }
    }
    return {total_cost, overtime};
}

// ====================================================================
// 评估: 双车方案 (总成本, 总超时)
// CarPlan: 单车计划结构, 含 trip_plans (每趟包裹id) 和 dest_orders (每趟目的地顺序)
// deliv0/deliv1: 输出, 两车各自的送达时间
// ====================================================================
std::pair<double, int> Task5::evaluate_full(
    const CarPlan& car0, const CarPlan& car1,
    vector<double>& deliv0, vector<double>& deliv1)
{
    auto [cost0, ot0] = evaluate_car(car0.trip_plans, car0.dest_orders, deliv0, c1);
    auto [cost1, ot1] = evaluate_car(car1.trip_plans, car1.dest_orders, deliv1, c2);
    return {cost0 + cost1, ot0 + ot1};
}

// ====================================================================
// K-means 初始分区 (k=2)
// 按包裹目的地坐标聚为两类, 各自分配给一辆车
// 返回: (车0的包裹id列表, 车1的包裹id列表)
// ====================================================================
std::pair<vector<int>, vector<int>> Task5::kmeans_partition()
{
    int n = n_pkg;
    if (n == 0) return {{}, {}};
    if (n == 1)
    {   // 只有一个包裹时分配给车1
        return {{0}, {}};
    }

    // 提取每个包裹目的地坐标作为特征向量
    vector<std::pair<double, double>> coords(n);  // (x, y) 坐标对
    for (int i = 0; i < n; ++i)
    {
        int dest = pkgs[i].dest;
        coords[i] = {g.get_nodes()[dest].x, g.get_nodes()[dest].y};
    }

    // K-means: 随机选两个初始质心
    std::uniform_int_distribution<int> uid(0, n - 1);  // 均匀整数分布 [0, n-1]
    int c0 = uid(rng);                                   // 质心0 对应的包裹索引
    int c1_idx;
    do { c1_idx = uid(rng); } while (c1_idx == c0);     // 质心1 对应的包裹索引, 确保不同
    vector<double> cx(2), cy(2);  // 两个质心的 (x, y) 坐标
    cx[0] = g.get_nodes()[pkgs[c0].dest].x;
    cy[0] = g.get_nodes()[pkgs[c0].dest].y;
    cx[1] = g.get_nodes()[pkgs[c1_idx].dest].x;
    cy[1] = g.get_nodes()[pkgs[c1_idx].dest].y;

    vector<int> labels(n, 0);       // 每个包裹所属的簇标签 (0 或 1)
    const int MAX_ITER = 100;       // 最大迭代次数, 防止不收敛

    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        bool changed = false;  // 本轮是否有包裹换了簇

        // E-step: 分配每个包裹到最近的质心
        for (int i = 0; i < n; ++i)
        {
            double d0 = (coords[i].first - cx[0]) * (coords[i].first - cx[0])
                      + (coords[i].second - cy[0]) * (coords[i].second - cy[0]);  // 到质心0的欧氏距离平方
            double d1 = (coords[i].first - cx[1]) * (coords[i].first - cx[1])
                      + (coords[i].second - cy[1]) * (coords[i].second - cy[1]);  // 到质心1的欧氏距离平方
            int new_label = (d0 <= d1) ? 0 : 1;
            if (new_label != labels[i]) { labels[i] = new_label; changed = true; }
        }

        if (!changed) break;  // 收敛, 聚类不再变化

        // M-step: 更新质心为簇内所有点的均值
        cx[0] = cy[0] = cx[1] = cy[1] = 0.0;
        int cnt0 = 0, cnt1 = 0;  // 各簇的点数
        for (int i = 0; i < n; ++i)
        {
            if (labels[i] == 0)
            {
                cx[0] += coords[i].first;
                cy[0] += coords[i].second;
                ++cnt0;
            }
            else
            {
                cx[1] += coords[i].first;
                cy[1] += coords[i].second;
                ++cnt1;
            }
        }
        if (cnt0 > 0) { cx[0] /= cnt0; cy[0] /= cnt0; }
        if (cnt1 > 0) { cx[1] /= cnt1; cy[1] /= cnt1; }
    }

    // 收集两簇的包裹 id
    vector<int> cluster0, cluster1;
    for (int i = 0; i < n; ++i)
    {
        if (labels[i] == 0) cluster0.push_back(i);
        else                cluster1.push_back(i);
    }

    // ---- 负载均衡: 保留 K-means 空间聚类, 仅在失衡时做局部调整 ----
    {
        double w0 = 0.0, w1 = 0.0;  // 两簇的总重量
        for (int pid : cluster0) w0 += pkgs[pid].weight;
        for (int pid : cluster1) w1 += pkgs[pid].weight;

        // 若 K-means 完全给了同一簇 (一车为空) → 按 EDD 交替分配
        if (cluster0.empty() || cluster1.empty())
        {
            vector<int> all;  // 所有包裹合并
            all.reserve(cluster0.size() + cluster1.size());
            for (int pid : cluster0) all.push_back(pid);
            for (int pid : cluster1) all.push_back(pid);
            std::sort(all.begin(), all.end(),
                      [&](int a, int b) { return pkgs[a].deadline < pkgs[b].deadline; });  // EDD: 按截止时间升序
            cluster0.clear();
            cluster1.clear();
            w0 = w1 = 0.0;
            for (int pid : all)
            {
                if (w0 <= w1) { cluster0.push_back(pid); w0 += pkgs[pid].weight; }
                else          { cluster1.push_back(pid); w1 += pkgs[pid].weight; }
            }
        }
        // 若两车总重差距 > 容量 → 从重簇移 deadline 最早的包裹到轻簇
        else if (std::abs(w0 - w1) > c1.capacity + 1e-9)
        {
            // 确定重簇和轻簇的引用
            vector<int>& heavy = (w0 > w1) ? cluster0 : cluster1;
            vector<int>& light = (w0 > w1) ? cluster1 : cluster0;

            // 按 deadline 升序, 搬包裹直到差距 ≤ 容量
            std::sort(heavy.begin(), heavy.end(),
                      [&](int a, int b) { return pkgs[a].deadline < pkgs[b].deadline; });
            double wh = std::max(w0, w1);  // 重簇重量
            double wl = std::min(w0, w1);  // 轻簇重量
            double target = (wh - wl) / 2.0;  // 需要搬走的目标重量

            vector<int> new_heavy;   // 搬迁后剩余的包裹 (仍留在重车)
            double moved = 0.0;      // 已搬走的重量
            for (int pid : heavy)
            {
                if (moved < target - 1e-9 || wh - moved - wl > c1.capacity + 1e-9)
                {
                    light.push_back(pid);
                    moved += pkgs[pid].weight;
                }
                else
                {
                    new_heavy.push_back(pid);
                }
            }
            heavy = new_heavy;
        }
    }

    return {cluster0, cluster1};
}

// ====================================================================
// 贪心初解 (单车)
// EDD 分趟 + nearest-first 趟内排序
// pkg_indices: 分配给该车的包裹id列表
// plan: 输出, 构造的 CarPlan (趟计划)
// ====================================================================
void Task5::greedy_init_one(const vector<int>& pkg_indices, CarPlan& plan)
{
    if (pkg_indices.empty()) return;

    // 按 deadline 升序 (EDD: Earliest Due Date)
    vector<int> order = pkg_indices;
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return pkgs[a].deadline < pkgs[b].deadline; });

    plan.trip_plans.clear();
    plan.dest_orders.clear();

    vector<int> cur_trip;      // 当前趟的包裹列表
    double cur_weight = 0.0;   // 当前趟已装重量

    for (int pid : order)
    {
        double w = pkgs[pid].weight;
        // 容量约束: 当前包裹装不下就封趟
        if (cur_weight + w > c1.capacity + 1e-9 && !cur_trip.empty())
        {
            plan.trip_plans.push_back(cur_trip);
            plan.dest_orders.push_back(nearest_first_order(unique_dests(cur_trip), ap));  // 贪心最近目的地排序
            cur_trip.clear();
            cur_weight = 0.0;
        }
        cur_trip.push_back(pid);
        cur_weight += w;
    }

    // 最后一趟 (如果有剩余)
    if (!cur_trip.empty())
    {
        plan.trip_plans.push_back(cur_trip);
        plan.dest_orders.push_back(nearest_first_order(unique_dests(cur_trip), ap));
    }
}

// ====================================================================
// ALNS 破坏算子: Worst-removal (代理版)
// 代理指标: dist[0][dest_i] * weight_i, 移除分数最高的 k 个
//   - dist[0][dest]: 驿站到目的地的距离 (越远代理成本越高)
//   - weight: 包裹重量 (越重搬运成本越高)
// k = max(1, round(n_total * ratio)), ratio ~ 0.2
// cars[2]: 两车的 CarPlan, 原地修改并返回被移除的包裹id列表 (插入池)
// ====================================================================
vector<int> Task5::destroy_worst(CarPlan (&cars)[2])
{
    const double ratio = 0.2;       // 移除比例
    int total_pkgs = 0;             // 两车总包裹数

    // 统计总包裹数
    for (int c = 0; c < 2; ++c)
        for (const auto& t : cars[c].trip_plans)
            total_pkgs += (int)t.size();

    int k = std::max(1, (int)std::round(total_pkgs * ratio));  // 要移除的包裹数

    // 对每个包裹打分: dist[0][dest] * weight
    vector<std::pair<double, int>> scored; // (分数, 包裹id), 分数越高越"差"
    for (int c = 0; c < 2; ++c)
        for (const auto& t : cars[c].trip_plans)
            for (int pid : t)
            {
                double score = ap.dist[0][pkgs[pid].dest] * pkgs[pid].weight;
                scored.emplace_back(score, pid);
            }

    // 按分数降序, 取前 k 个不重复的 pid
    std::sort(scored.rbegin(), scored.rend());
    vector<int> removed;               // 被移除的包裹id列表 (插入池)
    std::unordered_set<int> removed_set;  // 快速查找已移除的包裹

    for (int i = 0; i < (int)scored.size() && (int)removed.size() < k; ++i)
    {
        int pid = scored[i].second;
        if (removed_set.count(pid)) continue;
        removed.push_back(pid);
        removed_set.insert(pid);
    }

    // 安全删除: 重建两车的趟, 与 destroy_shaw 一致的删除逻辑
    for (int c = 0; c < 2; ++c)
    {
        for (int ti = (int)cars[c].trip_plans.size() - 1; ti >= 0; --ti)  // 倒序遍历以安全删除
        {
            auto& t = cars[c].trip_plans[ti];
            vector<int> kept;  // 本趟保留的包裹
            for (int pid : t)
                if (!removed_set.count(pid)) kept.push_back(pid);
            if (kept.empty())
            {
                // 整趟被清空, 删除该趟
                cars[c].trip_plans.erase(cars[c].trip_plans.begin() + ti);
                cars[c].dest_orders.erase(cars[c].dest_orders.begin() + ti);
            }
            else if ((int)kept.size() != (int)t.size())
            {
                t = kept;
                cars[c].dest_orders[ti] = nearest_first_order(unique_dests(t), ap);  // 重新计算目的地顺序
            }
        }
    }

    return removed;
}

// ====================================================================
// ALNS 破坏算子: Shaw-removal
// 随机选种子, 移除与其最相关的 k-1 个包裹
// 相关度 = γ1×地理距离 + γ2×deadline差异 + γ3×重量差异 (归一化)
//   - 地理距离: 包裹目的地之间的最短路径距离
//   - deadline差异: 两个包裹截止时间差的绝对值
//   - 重量差异: 两个包裹重量差的绝对值
// γ1=0.5, γ2=0.3, γ3=0.2
// 相关度值越小, 表示两个包裹越"相似", 应一起被移除
// ====================================================================
vector<int> Task5::destroy_shaw(CarPlan (&cars)[2])
{
    const double ratio = 0.2;                                 // 移除比例
    const double gamma1 = 0.5, gamma2 = 0.3, gamma3 = 0.2;   // 三项归一化指标的权重

    // 收集所有包裹及位置信息, 用于定位和删除
    struct PkgInfo { int pid; int car; int trip; int pos; };  // 包裹id, 所属车(0/1), 趟索引, 趟内位置
    vector<PkgInfo> all_pkgs;
    for (int c = 0; c < 2; ++c)
        for (int ti = 0; ti < (int)cars[c].trip_plans.size(); ++ti)
            for (int pos = 0; pos < (int)cars[c].trip_plans[ti].size(); ++pos)
                all_pkgs.push_back({cars[c].trip_plans[ti][pos], c, ti, pos});

    int total = (int)all_pkgs.size();
    if (total <= 1) return {};

    int k = std::max(1, std::min((int)std::round(total * ratio), total));  // 移除数量

    // 随机选种子包裹
    std::uniform_int_distribution<int> uid(0, total - 1);
    int seed_idx = uid(rng);       // 种子在 all_pkgs 中的索引
    int seed_pid = all_pkgs[seed_idx].pid;  // 种子的包裹id

    // 计算归一化参数的上下界 (用于 min-max 归一化)
    double max_dist = 0.0, max_dl = 0.0, max_w = 0.0;
    double min_dist = 1e18, min_dl = 1e18, min_w = 1e18;
    for (const auto& info : all_pkgs)
    {
        double dist = ap.dist[pkgs[seed_pid].dest][pkgs[info.pid].dest];
        max_dist = std::max(max_dist, dist);
        min_dist = std::min(min_dist, dist);

        double dl = std::abs(pkgs[seed_pid].deadline - pkgs[info.pid].deadline);
        max_dl = std::max(max_dl, dl);
        min_dl = std::min(min_dl, dl);

        double w = std::abs(pkgs[seed_pid].weight - pkgs[info.pid].weight);
        max_w = std::max(max_w, w);
        min_w = std::min(min_w, w);
    }
    // 避免除零: 若上下界相等则拉开 1.0
    if (max_dist - min_dist < 1e-12) max_dist = min_dist + 1.0;
    if (max_dl - min_dl < 1e-12) max_dl = min_dl + 1.0;
    if (max_w - min_w < 1e-12) max_w = min_w + 1.0;

    // 计算相关度 (越小越相关), 与种子的相似度
    vector<std::pair<double, int>> related; // (相关度值, 在 all_pkgs 中的索引)
    for (int i = 0; i < total; ++i)
    {
        if (i == seed_idx) continue;
        int pid = all_pkgs[i].pid;

        // min-max 归一化到 [0, 1]
        double dist_norm = (ap.dist[pkgs[seed_pid].dest][pkgs[pid].dest] - min_dist) / (max_dist - min_dist);
        double dl_norm   = (std::abs(pkgs[seed_pid].deadline - pkgs[pid].deadline) - min_dl) / (max_dl - min_dl);
        double w_norm    = (std::abs(pkgs[seed_pid].weight - pkgs[pid].weight) - min_w) / (max_w - min_w);

        double rel = gamma1 * dist_norm + gamma2 * dl_norm + gamma3 * w_norm;  // 加权归一化相关度
        related.emplace_back(rel, i);
    }

    std::sort(related.begin(), related.end()); // 升序, 相关度最高的在前 (值最小)

    vector<int> removed;  // 被移除的包裹id列表 (插入池)
    removed.push_back(seed_pid);  // 种子自身也被移除

    for (int i = 0; i < k - 1 && i < (int)related.size(); ++i)
        removed.push_back(all_pkgs[related[i].second].pid);

    // 从方案中删除 (重新扫描删除, 比逐个删除更简单可靠)
    std::unordered_set<int> removed_set(removed.begin(), removed.end());
    for (int c = 0; c < 2; ++c)
    {
        for (int ti = (int)cars[c].trip_plans.size() - 1; ti >= 0; --ti)
        {
            auto& t = cars[c].trip_plans[ti];
            vector<int> kept;
            for (int pid : t)
                if (!removed_set.count(pid)) kept.push_back(pid);
            if (kept.empty())
            {
                cars[c].trip_plans.erase(cars[c].trip_plans.begin() + ti);
                cars[c].dest_orders.erase(cars[c].dest_orders.begin() + ti);
            }
            else if ((int)kept.size() != (int)t.size())
            {
                t = kept;
                cars[c].dest_orders[ti] = nearest_first_order(unique_dests(t), ap);
            }
        }
    }

    return removed;
}

// ====================================================================
// ALNS 破坏算子: Trip-destroy
// 随机选一辆车的一整趟, 全部移除
// 返回被移除的包裹id列表
// ====================================================================
vector<int> Task5::destroy_trip(CarPlan (&cars)[2])
{
    // 收集所有非空趟
    vector<std::pair<int, int>> trips; // (车辆索引0/1, 趟索引)
    for (int c = 0; c < 2; ++c)
        for (int ti = 0; ti < (int)cars[c].trip_plans.size(); ++ti)
            if (!cars[c].trip_plans[ti].empty())
                trips.emplace_back(c, ti);

    if (trips.empty()) return {};

    std::uniform_int_distribution<int> uid(0, (int)trips.size() - 1);
    auto [sel_c, sel_t] = trips[uid(rng)];  // 随机选中: 车辆索引, 趟索引

    vector<int> removed = cars[sel_c].trip_plans[sel_t];  // 整趟的包裹全部移除

    cars[sel_c].trip_plans.erase(cars[sel_c].trip_plans.begin() + sel_t);
    cars[sel_c].dest_orders.erase(cars[sel_c].dest_orders.begin() + sel_t);

    return removed;
}

// ====================================================================
// ALNS 破坏算子: Time-window-removal
// 移除所有当前超时的包裹 + 随机若干同趟包裹
// 思想: 超时包裹大概率需要重新安排, 并扰动其同趟相邻包裹以增加探索
// ====================================================================
vector<int> Task5::destroy_time_window(CarPlan (&cars)[2])
{
    // 需要先评估当前方案获取超时包裹
    vector<double> deliv0, deliv1;  // 两车各包裹的送达时间
    evaluate_full(cars[0], cars[1], deliv0, deliv1);

    vector<int> overtime_pkgs;  // 所有超时包裹id
    for (int pid = 0; pid < n_pkg; ++pid)
    {
        // 每个包裹只属于一辆车, 取有效送达时间 (>= 0)
        double d_time = (deliv0[pid] >= 0.0) ? deliv0[pid] : deliv1[pid];
        if (d_time > pkgs[pid].deadline + 1e-9)
            overtime_pkgs.push_back(pid);
    }

    if (overtime_pkgs.empty()) return {};

    std::unordered_set<int> removed_set(overtime_pkgs.begin(), overtime_pkgs.end());  // 已确定的移除集合

    // 额外移除若干同趟包裹 (约 30% 的同趟包裹), 增加扰动范围
    vector<int> extra;  // 额外移除的候选
    for (int c = 0; c < 2; ++c)
        for (int ti = 0; ti < (int)cars[c].trip_plans.size(); ++ti)
        {
            bool has_ot = false;  // 本趟是否有超时包裹
            for (int pid : cars[c].trip_plans[ti])
                if (removed_set.count(pid)) { has_ot = true; break; }
            if (!has_ot) continue;

            // 随机移除该趟中约 30% 的包裹
            int n_in_trip = (int)cars[c].trip_plans[ti].size();
            int n_remove = std::max(1, (int)std::round(n_in_trip * 0.3));
            vector<int> candidates;  // 该趟中尚未被标记移除的候选包裹
            for (int pid : cars[c].trip_plans[ti])
                if (!removed_set.count(pid)) candidates.push_back(pid);
            std::shuffle(candidates.begin(), candidates.end(), rng);  // 随机打乱
            for (int i = 0; i < n_remove && i < (int)candidates.size(); ++i)
                removed_set.insert(candidates[i]);
        }

    vector<int> removed(removed_set.begin(), removed_set.end());

    // 从方案中删除
    for (int c = 0; c < 2; ++c)
        for (int ti = (int)cars[c].trip_plans.size() - 1; ti >= 0; --ti)
        {
            auto& t = cars[c].trip_plans[ti];
            vector<int> kept;
            for (int pid : t)
                if (!removed_set.count(pid)) kept.push_back(pid);
            if (kept.empty())
            {
                cars[c].trip_plans.erase(cars[c].trip_plans.begin() + ti);
                cars[c].dest_orders.erase(cars[c].dest_orders.begin() + ti);
            }
            else if ((int)kept.size() != (int)t.size())
            {
                t = kept;
                cars[c].dest_orders[ti] = nearest_first_order(unique_dests(t), ap);
            }
        }

    return removed;
}

// ====================================================================
// ALNS 修复算子: Greedy-cheapest (代理版)
// 对每个待插入包裹, 遍历两车所有趟的所有可能插入位置
// 代理指标 (避免完整 evaluate): 局部边权增量
//   Δ = dist[prev][dest_i] + dist[dest_i][next] - dist[prev][next]
//   prev/next: 插入位置前后的目的地节点 (0 表示驿站)
// 选择 Δ 最小的位置; 若容量不足则新增一趟
// 每次插入后更新趟结构, 再处理下一个包裹
// pool: 待插入的包裹id列表 (输入输出, 被消费后清空)
// cars[2]: 两车的 CarPlan, 原地修改
// ====================================================================
void Task5::repair_greedy(vector<int>& pool, CarPlan (&cars)[2])
{
    const double cap[2] = {c1.capacity, c2.capacity};  // 两车容量

    while (!pool.empty())
    {
        double best_delta = 1e18;     // 当前最佳代理增量
        int best_pool_idx = -1;       // 最佳包裹在 pool 中的索引
        int best_car = -1, best_trip = -1, best_pos = -1;  // 最佳插入: 车, 趟(-1=新增), 位置

        // 计算两车当前总重, 用于负载均衡惩罚
        double total_w[2] = {0.0, 0.0};
        for (int c = 0; c < 2; ++c)
            for (const auto& t : cars[c].trip_plans)
                total_w[c] += trip_weight(t);
        double balance_penalty = 10.0; // 负载均衡惩罚系数: 避免把包裹全塞给一车

        for (size_t pi = 0; pi < pool.size(); ++pi)
        {
            int pid = pool[pi];
            int dest = pkgs[pid].dest;  // 包裹目的地节点
            double w = pkgs[pid].weight;

            for (int c = 0; c < 2; ++c)
            {
                auto& plan = cars[c];
                int n_trips = (int)plan.trip_plans.size();
                // 负载惩罚: 往总重更重的车插入要多付出代价, 鼓励使用轻车
                double load_bias = (total_w[c] - (c == 0 ? total_w[1] : total_w[0])) * w * balance_penalty;

                // 情况1: 插入到已有的某趟中
                for (int ti = 0; ti < n_trips; ++ti)
                {
                    if (trip_weight(plan.trip_plans[ti]) + w > cap[c] + 1e-9)
                        continue;  // 容量不足

                    const auto& d_order = plan.dest_orders[ti];  // 该趟目的地顺序
                    int nd = (int)d_order.size();                 // 目的地数量

                    bool dest_exists = false;  // 该目的地是否已在趟中
                    for (int d : d_order)
                        if (d == dest) { dest_exists = true; break; }

                    if (dest_exists)
                    {
                        double delta = load_bias; // 同目的地, 不增加路程, 仅负载惩罚
                        if (delta < best_delta)
                        {
                            best_delta = delta;
                            best_pool_idx = (int)pi;
                            best_car = c;
                            best_trip = ti;
                            best_pos = 0;  // 同目的地不关心插入位置
                        }
                        continue;
                    }

                    // 遍历插入位置: 在 prev 和 next 之间插入 dest
                    for (int pos = 0; pos <= nd; ++pos)
                    {
                        int prev_node = (pos == 0) ? 0 : d_order[pos - 1];      // 前驱节点 (0=驿站)
                        int next_node = (pos == nd) ? 0 : d_order[pos];          // 后继节点 (0=驿站)
                        double delta = ap.dist[prev_node][dest] + ap.dist[dest][next_node] - ap.dist[prev_node][next_node] + load_bias;
                        if (delta < best_delta)
                        {
                            best_delta = delta;
                            best_pool_idx = (int)pi;
                            best_car = c;
                            best_trip = ti;
                            best_pos = pos;
                        }
                    }
                }

                // 情况2: 新增一趟 (从驿站出发, 送完该包裹后返回)
                {
                    double delta = ap.dist[0][dest] + ap.dist[dest][0] + load_bias;
                    // 如果一辆车当前完全为空 (0趟), 新增第一趟时给额外奖励, 鼓励激活第二辆车
                    if (plan.trip_plans.empty())
                    {
                        delta -= 200.0;  // "免死金牌", 大幅降低新增成本
                    }

                    if (delta < best_delta)
                    {
                        best_delta = delta;
                        best_pool_idx = (int)pi;
                        best_car = c;
                        best_trip = -1;  // -1 表示新增一趟
                        best_pos = -1;
                    }
                }
            }
        }

        if (best_pool_idx == -1) { pool.clear(); return; }  // 无处可插, 放弃

        int pid = pool[best_pool_idx];
        int dest = pkgs[pid].dest;

        // 执行插入
        if (best_trip == -1)
        {
            // 新增一趟
            cars[best_car].trip_plans.push_back({pid});
            cars[best_car].dest_orders.push_back({dest});
        }
        else
        {
            auto& plan = cars[best_car];
            auto& t = plan.trip_plans[best_trip];
            t.push_back(pid);

            auto& d_order = plan.dest_orders[best_trip];
            bool exists = false;
            for (int d : d_order)
                if (d == dest) { exists = true; break; }
            if (!exists && best_pos >= 0 && best_pos <= (int)d_order.size())
                d_order.insert(d_order.begin() + best_pos, dest);  // 在指定位置插入新目的地
        }

        pool.erase(pool.begin() + best_pool_idx);  // 从待插池中移除
    }
}

// ====================================================================
// ALNS 修复算子: Regret-2
// 对每个待插入包裹计算 regret = 次佳Δ - 最佳Δ
// regret 越大说明"如果不趁现在插入, 将来会更糟", 应优先插入
// 与 Greedy-cheapest 相比, Regret-2 避免了"先来先占"的短视问题
// ====================================================================
void Task5::repair_regret2(vector<int>& pool, CarPlan (&cars)[2])
{
    const double cap[2] = {c1.capacity, c2.capacity};  // 两车容量

    while (!pool.empty())
    {
        // 对 pool 中每个包裹计算最佳和次佳 Δ
        struct InsertInfo
        {
            double best_delta;    // 最佳位置的成本增量
            double second_delta;  // 次佳位置的成本增量
            int best_car, best_trip, best_pos; // 最佳位置的坐标
        };
        vector<InsertInfo> infos(pool.size());

        for (size_t pi = 0; pi < pool.size(); ++pi)
        {
            int pid = pool[pi];
            int dest = pkgs[pid].dest;

            double d1 = 1e18, d2 = 1e18;  // best, second (越小越好)
            int bc = -1, bt = -1, bp = -1;  // 最佳位置的 (车, 趟, 位置)

            for (int c = 0; c < 2; ++c)
            {
                auto& plan = cars[c];
                int n_trips = (int)plan.trip_plans.size();

                // 已有趟的插入位置
                for (int ti = 0; ti < n_trips; ++ti)
                {
                    if (trip_weight(plan.trip_plans[ti]) + pkgs[pid].weight > cap[c] + 1e-9)
                        continue;

                    const auto& d_order = plan.dest_orders[ti];
                    int nd = (int)d_order.size();

                    // 先查该目的地是否已在趟中
                    bool dest_exists = false;
                    for (int d : d_order)
                        if (d == dest) { dest_exists = true; break; }

                    if (dest_exists)
                    {
                        double delta = 0.0;  // 同目的地, 边权增量为0
                        if (delta < d1)        { d2 = d1; d1 = delta; bc = c; bt = ti; bp = 0; }
                        else if (delta < d2)   { d2 = delta; }
                        continue;
                    }

                    // 遍历所有可能插入位置
                    for (int pos = 0; pos <= nd; ++pos)
                    {
                        int prev_node = (pos == 0) ? 0 : d_order[pos - 1];
                        int next_node = (pos == nd) ? 0 : d_order[pos];
                        double delta = ap.dist[prev_node][dest] + ap.dist[dest][next_node] - ap.dist[prev_node][next_node];

                        if (delta < d1)        { d2 = d1; d1 = delta; bc = c; bt = ti; bp = pos; }
                        else if (delta < d2)   { d2 = delta; }
                    }
                }

                // 新增一趟
                {
                    double delta = ap.dist[0][dest] + ap.dist[dest][0];
                    if (delta < d1)        { d2 = d1; d1 = delta; bc = c; bt = -1; bp = -1; }
                    else if (delta < d2)   { d2 = delta; }
                }
            }

            if (d2 >= 1e18) d2 = d1 + 1000.0; // 只有一个可行位置时给一个大 regret, 优先插入
            infos[pi] = {d1, d2, bc, bt, bp};
        }

        // 找 regret 最大的包裹 (最需要优先安排)
        double max_regret = -1.0;
        int chosen = -1;  // pool 中被选中插入的包裹索引
        for (size_t pi = 0; pi < pool.size(); ++pi)
        {
            double regret = infos[pi].second_delta - infos[pi].best_delta;
            if (regret > max_regret)
            {
                max_regret = regret;
                chosen = (int)pi;
            }
        }

        if (chosen == -1) { pool.clear(); return; } // 所有包裹无处可插

        int pid = pool[chosen];
        int dest = pkgs[pid].dest;
        auto& info = infos[chosen];

        // 执行插入
        if (info.best_trip == -1)
        {
            // 新增一趟
            cars[info.best_car].trip_plans.push_back({pid});
            cars[info.best_car].dest_orders.push_back({dest});
        }
        else
        {
            auto& plan = cars[info.best_car];
            auto& t = plan.trip_plans[info.best_trip];
            t.push_back(pid);

            auto& d_order = plan.dest_orders[info.best_trip];
            bool exists = false;
            for (int d : d_order)
                if (d == dest) { exists = true; break; }
            if (!exists && info.best_pos >= 0 && info.best_pos <= (int)d_order.size())
                d_order.insert(d_order.begin() + info.best_pos, dest);
        }

        pool.erase(pool.begin() + chosen);
    }
}

// ====================================================================
// SA 优化主循环 (Simulated Annealing)
// 评分函数: score = total_cost + overtime_penalty * overtime_count
// 温控: Lundy-Mees 恒温退火 T_{k+1} = T_k / (1 + β·T_k)
//   - β = (T_init - T_min) / (N_iter · T_init · T_min)
//   - 温度从 T_init 缓慢降到 T_min
// 算子选择: 轮盘赌, 权重自适应 (表现好的算子增加被选中概率)
// cars[2]: 两车方案, 原地优化
// ====================================================================
void Task5::sa_optimize(CarPlan (&cars)[2])
{
    // 当前解评估
    vector<double> cur_deliv0, cur_deliv1;  // 当前解各包裹送达时间
    auto [cur_cost, cur_ot] = evaluate_full(cars[0], cars[1], cur_deliv0, cur_deliv1);
    double cur_score = weighted_score(cur_cost, cur_ot, overtime_penalty);
    double best_cost = cur_cost;
    int best_ot = cur_ot;
    double best_score = cur_score;
    CarPlan best_cars[2] = {cars[0], cars[1]};  // 全局最优方案备份

    // ---- 自适应初始温度 ----
    // 对初解做 100 次随机扰动, 用上坡 Δcost 的中位数反推 T_init (初始接受率 ≈ 0.8)
    vector<double> deltas;  // 收集所有正 Δ (恶化量)
    for (int k = 0; k < 100; ++k)
    {
        CarPlan tmp[2] = {cars[0], cars[1]};

        // 随机选一个破坏算子
        int destroy_op = std::uniform_int_distribution<int>(0, 3)(rng);
        vector<int> pool;  // 被移除的包裹id列表 (插入池)
        switch (destroy_op)
        {
            case 0: pool = destroy_worst(tmp);  break;
            case 1: pool = destroy_shaw(tmp);   break;
            case 2: pool = destroy_trip(tmp);   break;
            case 3: pool = destroy_time_window(tmp); break;
        }

        if (pool.empty()) continue;

        // 随机选一个修复算子
        int repair_op = std::uniform_int_distribution<int>(0, 1)(rng);
        if (repair_op == 0) repair_greedy(pool, tmp);
        else                repair_regret2(pool, tmp);

        vector<double> dd0, dd1;
        auto [new_cost, new_ot] = evaluate_full(tmp[0], tmp[1], dd0, dd1);
        double d = weighted_score(new_cost, new_ot, overtime_penalty) - cur_score;
        if (d > 0) deltas.push_back(d);  // 只收集恶化的 Δ
    }

    double T = 1.0;  // 初始温度
    if (!deltas.empty())
    {
        std::sort(deltas.begin(), deltas.end());
        double delta_med = deltas[deltas.size() / 2];  // 恶化量的中位数
        T = delta_med / 0.223; // exp(-Δ/T) = 0.8 → T = Δ / ln(1/0.8) ≈ Δ / 0.223
        if (T < 0.1) T = 0.1;  // 下界保护
    }

    // ---- Lundy-Mees 参数 ----
    const int    MAX_ITER = 20000;   // 最大迭代次数
    const double T_MIN    = 1e-3;    // 最低温度 (停止阈值)
    double beta = (T - T_MIN) / (MAX_ITER * T * T_MIN);  // Lundy-Mees 冷却系数

    // 算子自适应权重 (初始均等)
    // destroy_weights: 对应 destroy_worst, destroy_shaw, destroy_trip, destroy_time_window
    int destroy_weights[4] = {1, 1, 1, 1};
    // repair_weights: 对应 repair_greedy, repair_regret2
    int repair_weights[2]  = {1, 1};
    int last_destroy = 0, last_repair = 0;  // 记录本轮使用的算子, 用于后续权重更新

    std::uniform_real_distribution<double> uni(0.0, 1.0);  // [0,1) 均匀分布, 用于 SA 接受判定

    // ---- 主循环 ----
    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        // 保存当前方案用于回退 (拒绝新解时恢复)
        CarPlan saved[2] = {cars[0], cars[1]};

        // ---- 轮盘赌选破坏算子 ----
        int dw_sum = destroy_weights[0] + destroy_weights[1] + destroy_weights[2] + destroy_weights[3];
        std::uniform_int_distribution<int> dw_die(0, dw_sum - 1);
        int r_d = dw_die(rng);  // 随机掷点
        int destroy_op;
        {
            int acc = 0;  // 累积权重
            destroy_op = 0;
            for (int i = 0; i < 4; ++i)
            {
                if (r_d < acc + destroy_weights[i]) { destroy_op = i; break; }
                acc += destroy_weights[i];
            }
        }

        // ---- 轮盘赌选修复算子 ----
        int rw_sum = repair_weights[0] + repair_weights[1];
        std::uniform_int_distribution<int> rw_die(0, rw_sum - 1);
        int r_r = rw_die(rng);
        int repair_op = (r_r < repair_weights[0]) ? 0 : 1;

        last_destroy = destroy_op;
        last_repair  = repair_op;

        // ---- 执行破坏 ----
        vector<int> pool;  // 被移除的包裹id列表
        switch (destroy_op)
        {
            case 0: pool = destroy_worst(cars);  break;
            case 1: pool = destroy_shaw(cars);   break;
            case 2: pool = destroy_trip(cars);   break;
            case 3: pool = destroy_time_window(cars); break;
        }

        if (pool.empty())
        {
            // 无效操作 (如空趟移除), 回退并降温
            cars[0] = saved[0];
            cars[1] = saved[1];
            goto COOL;  // 跳到降温步骤
        }

        // ---- 执行修复 ----
        if (repair_op == 0) repair_greedy(pool, cars);
        else                repair_regret2(pool, cars);

        // ---- 完整评估新解 ----
        {
            vector<double> nd0, nd1;  // 新解的送达时间
            auto [new_cost, new_ot] = evaluate_full(cars[0], cars[1], nd0, nd1);
            double new_score = weighted_score(new_cost, new_ot, overtime_penalty);
            double delta = new_score - cur_score;  // Δ: 正=恶化, 负=改进
            bool accept = false;

            // SA 接受准则: 改进则接受; 同分看 better_solution; 恶化按 Metropolis 准则
            if (new_score < cur_score - 1e-9)
                accept = true;
            else if (std::abs(new_score - cur_score) <= 1e-9 &&
                     better_solution(new_cost, new_ot, cur_cost, cur_ot, overtime_penalty))
                accept = true;
            else if (new_score > cur_score + 1e-9 && uni(rng) < std::exp(-delta / T))  // Metropolis: P(accept) = exp(-Δ/T)
                accept = true;

            if (accept)
            {
                // 接受新解, 更新当前状态
                cur_cost  = new_cost;
                cur_ot    = new_ot;
                cur_score = new_score;
                cur_deliv0 = nd0;
                cur_deliv1 = nd1;

                // 若新解优于全局最优, 更新最优解并奖励算子权重
                if (new_score < best_score - 1e-9 ||
                    (std::abs(new_score - best_score) <= 1e-9 &&
                     better_solution(new_cost, new_ot, best_cost, best_ot, overtime_penalty)))
                {
                    best_score = new_score;
                    best_cost = new_cost;
                    best_ot = new_ot;
                    best_cars[0] = cars[0];
                    best_cars[1] = cars[1];

                    // 自适应权重: 找到更优解时增加当前算子权重 (表现好的算子被选中概率更高)
                    ++destroy_weights[last_destroy];
                    ++repair_weights[last_repair];
                }
            }
            else
            {
                // 拒绝, 回退到保存的旧解
                cars[0] = saved[0];
                cars[1] = saved[1];
            }
        }

    COOL:
        T = T / (1.0 + beta * T);  // Lundy-Mees 降温公式
        if (T < T_MIN) break;       // 温度过低, 停止搜索
    }

    // 恢复全局最优解
    cars[0] = best_cars[0];
    cars[1] = best_cars[1];
}

// ====================================================================
// compute_cost (用于外部调用, 返回当前 trips 总成本)
// ====================================================================
double Task5::compute_cost()
{
    double total = 0.0;
    for (const auto& t : trips1) total += trip_cost(t, g, pkgs, c1, ap);
    for (const auto& t : trips2) total += trip_cost(t, g, pkgs, c2, ap);
    return total;
}

// ====================================================================
// solve (主入口)
// 流程: 1. K-means 分区 → 2. 贪心初解 → 3. SA+ALNS 优化 → 4. 构造输出
// 进行 RESTARTS 次独立求解 (不同随机种子), 取最优
// ====================================================================
Task5Result Task5::solve()
{
    const int RESTARTS = 6;                      // 独立求解次数
    const unsigned int BASE_SEED = 20260719u;     // 基础随机种子, 每次 +1

    CarPlan best_final[2];       // 所有 restart 中最优的两车方案
    double best_total_cost = 0.0;
    int best_total_ot = 0;
    bool have_best = false;      // 是否已记录至少一个有效解

    for (int attempt = 0; attempt < RESTARTS; ++attempt)
    {
        rng.seed(BASE_SEED + attempt);  // 固定种子保证可复现

        // ---- 1. K-means 初始分区 ----
        auto [cluster0, cluster1] = kmeans_partition();  // 结构绑定: 两个簇的包裹id列表

        // ---- 2. 贪心初解 ----
        CarPlan cars0, cars1;  // 两车初始方案
        greedy_init_one(cluster0, cars0);
        greedy_init_one(cluster1, cars1);

        CarPlan cars_arr[2] = {cars0, cars1};

        // ---- 3. SA + ALNS 优化 ----
        sa_optimize(cars_arr);

        // ---- 4. 记录本轮结果 ----
        vector<double> deliv0, deliv1;
        auto [cost, ot] = evaluate_full(cars_arr[0], cars_arr[1], deliv0, deliv1);

        if (!have_best || better_solution(cost, ot, best_total_cost, best_total_ot, overtime_penalty))
        {
            have_best = true;
            best_total_cost = cost;
            best_total_ot = ot;
            best_final[0] = cars_arr[0];
            best_final[1] = cars_arr[1];
        }
    }

    // 最终评估最优方案
    vector<double> deliv0, deliv1;
    auto [total_cost, overtime] = evaluate_full(best_final[0], best_final[1], deliv0, deliv1);

    // 构造 trips1 (车1的 Trip 列表, 用于外部输出)
    trips1.clear();
    double cur_time1 = 0.0;  // 车1当前时刻
    for (size_t t = 0; t < best_final[0].trip_plans.size(); ++t)
    {
        const auto& pkg_ids = best_final[0].trip_plans[t];
        const auto& dest_order = best_final[0].dest_orders[t];
        if (pkg_ids.empty()) continue;
        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, cur_time1, pkgs, c1, ap);  // 模拟得到 Trip 结构
        trips1.push_back(sim);
        cur_time1 = sim.end_time;
    }

    // 构造 trips2 (车2的 Trip 列表)
    trips2.clear();
    double cur_time2 = 0.0;  // 车2当前时刻
    for (size_t t = 0; t < best_final[1].trip_plans.size(); ++t)
    {
        const auto& pkg_ids = best_final[1].trip_plans[t];
        const auto& dest_order = best_final[1].dest_orders[t];
        if (pkg_ids.empty()) continue;
        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, cur_time2, pkgs, c2, ap);
        trips2.push_back(sim);
        cur_time2 = sim.end_time;
    }

    Task5Result res;
    res.trips1  = trips1;       // 车1各趟配送
    res.trips2  = trips2;       // 车2各趟配送
    res.sumCost = total_cost;   // 总运送成本
    res.exTime  = overtime;     // 超时包裹数
    return res;
}