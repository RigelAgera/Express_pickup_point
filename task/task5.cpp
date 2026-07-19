// T5: 双车协同配送 (2-CVRP + deadline)
// 算法框架: K-means 初始分区 → 贪心初解 → ALNS 产生新解 → SA 决定接受/拒绝
// 优化目标: total_cost = 车1成本 + 车2成本, 超时仅作统计
#include "task5.h"
#include "../common/dijkstra.h"
#include "../common/schedule_utils.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <unordered_set>

using std::vector;

// ====================================================================
// 构造函数
// ====================================================================
Task5::Task5(const input_data& data)
    : g(data.g), pkgs(data.packages), c1(data.c), c2(data.c),
      ap(all_pairs_dijkstra(data.g)),
      n_pkg((int)data.packages.size()),
      rng(std::random_device{}())
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

// 趟内排序: 按目的地总重量降序, 重者先送
vector<int> Task5::heaviest_dest_order(const vector<int>& pkg_ids)
{
    vector<int> dests = unique_dests(pkg_ids);
    vector<std::pair<double, int>> scored;
    for (int d : dests)
    {
        double tw = 0.0;
        for (int pid : pkg_ids)
            if (pkgs[pid].dest == d) tw += pkgs[pid].weight;
        scored.emplace_back(tw, d);
    }
    std::sort(scored.rbegin(), scored.rend());
    vector<int> order;
    for (auto& p : scored) order.push_back(p.second);
    return order;
}

// ====================================================================
// 评估: 单车方案 (成本, 超时数)
// ====================================================================
std::pair<double, int> Task5::evaluate_car(
    const vector<vector<int>>& trip_plans,
    const vector<vector<int>>& dest_orders,
    vector<double>& deliver_time,
    const car& c)
{
    deliver_time.assign(n_pkg, -1.0);
    double cur_time = 0.0;
    double total_cost = 0.0;
    int overtime = 0;

    for (size_t t = 0; t < trip_plans.size(); ++t)
    {
        const auto& pkg_ids = trip_plans[t];
        const auto& dest_order = dest_orders[t];
        if (pkg_ids.empty()) continue;

        // 构建路线 & 模拟本趟
        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, cur_time, pkgs, c, ap);

        cur_time = sim.end_time;
        total_cost += trip_cost(sim, g, pkgs, c, ap);

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
    vector<std::pair<double, double>> coords(n);
    for (int i = 0; i < n; ++i)
    {
        int dest = pkgs[i].dest;
        coords[i] = {g.get_nodes()[dest].x, g.get_nodes()[dest].y};
    }

    // K-means: 随机选两个初始质心
    std::uniform_int_distribution<int> uid(0, n - 1);
    int c0 = uid(rng);
    int c1_idx;
    do { c1_idx = uid(rng); } while (c1_idx == c0);
    vector<double> cx(2), cy(2);
    cx[0] = g.get_nodes()[pkgs[c0].dest].x;
    cy[0] = g.get_nodes()[pkgs[c0].dest].y;
    cx[1] = g.get_nodes()[pkgs[c1_idx].dest].x;
    cy[1] = g.get_nodes()[pkgs[c1_idx].dest].y;

    vector<int> labels(n, 0);
    const int MAX_ITER = 100;

    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        bool changed = false;

        // E-step: 分配每个包裹到最近的质心
        for (int i = 0; i < n; ++i)
        {
            double d0 = (coords[i].first - cx[0]) * (coords[i].first - cx[0])
                      + (coords[i].second - cy[0]) * (coords[i].second - cy[0]);
            double d1 = (coords[i].first - cx[1]) * (coords[i].first - cx[1])
                      + (coords[i].second - cy[1]) * (coords[i].second - cy[1]);
            int new_label = (d0 <= d1) ? 0 : 1;
            if (new_label != labels[i]) { labels[i] = new_label; changed = true; }
        }

        if (!changed) break;

        // M-step: 更新质心
        cx[0] = cy[0] = cx[1] = cy[1] = 0.0;
        int cnt0 = 0, cnt1 = 0;
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

    // ---- 确保可行: 若某车超重, 将溢出包裹按 deadline 升序移到另一车 ----
    auto fix_overload = [&](vector<int>& from, vector<int>& to)
    {
        // 计算理论最大容量: w_lim × ceil(n / avg_trip_capacity) 近似为 w_lim * n
        // 实际检查: 若总重超过 w_lim * ceil(from.size() / (w_lim / avg_w))
        // 简化: 按 deadline 升序排序, 累加重量, 超出的移至另一车
        // 更简单的做法: 确保每车包裹总重不超过 w_lim * 理论最少趟数
        // 理论最少趟数 = ceil(total_weight / w_lim)
        double total_w = 0.0;
        for (int pid : from) total_w += pkgs[pid].weight;
        double avg_w = total_w / from.size();
        double max_weight = c1.capacity * std::ceil((double)from.size() / std::max(1.0, c1.capacity / std::max(avg_w, 0.001)));
        // 用更宽松的方式: 直接按 w_lim * 趟数检查, 趟数 ≈ ceil(total_w / w_lim)
        int min_trips = (int)std::ceil(total_w / c1.capacity);
        if (min_trips <= 0) min_trips = 1;
        double allowed = c1.capacity * min_trips;

        if (total_w <= allowed + 1e-9) return;

        // 按 deadline 升序, 溢出部分移到另一车
        std::sort(from.begin(), from.end(),
                  [&](int a, int b) { return pkgs[a].deadline < pkgs[b].deadline; });

        vector<int> new_from;
        double cur_w = 0.0;
        for (int pid : from)
        {
            if (cur_w + pkgs[pid].weight <= allowed + 1e-9)
            {
                new_from.push_back(pid);
                cur_w += pkgs[pid].weight;
            }
            else
            {
                to.push_back(pid);
            }
        }
        from = new_from;
    };

    fix_overload(cluster0, cluster1);
    fix_overload(cluster1, cluster0);

    return {cluster0, cluster1};
}

// ====================================================================
// 贪心初解 (单车, 与 T3 完全一致)
// EDD 分趟 + heaviest-first 趟内排序
// ====================================================================
void Task5::greedy_init_one(const vector<int>& pkg_indices, CarPlan& plan)
{
    if (pkg_indices.empty()) return;

    // 按 deadline 升序 (EDD)
    vector<int> order = pkg_indices;
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return pkgs[a].deadline < pkgs[b].deadline; });

    plan.trip_plans.clear();
    plan.dest_orders.clear();

    vector<int> cur_trip;
    double cur_weight = 0.0;

    for (int pid : order)
    {
        double w = pkgs[pid].weight;
        // 容量约束: 当前包裹装不下就封趟
        if (cur_weight + w > c1.capacity + 1e-9 && !cur_trip.empty())
        {
            plan.trip_plans.push_back(cur_trip);
            plan.dest_orders.push_back(heaviest_dest_order(cur_trip));
            cur_trip.clear();
            cur_weight = 0.0;
        }
        cur_trip.push_back(pid);
        cur_weight += w;
    }

    if (!cur_trip.empty())
    {
        plan.trip_plans.push_back(cur_trip);
        plan.dest_orders.push_back(heaviest_dest_order(cur_trip));
    }
}

// ====================================================================
// ALNS 破坏算子: Worst-removal (代理版)
// 代理指标: dist[0][dest_i] * weight_i, 移除分数最高的 k 个
// k = max(1, round(n_total * ratio)), ratio ~ 0.2
// ====================================================================
vector<int> Task5::destroy_worst(CarPlan cars[2])
{
    const double ratio = 0.2;
    int total_pkgs = 0;

    // 统计总包裹数
    for (int c = 0; c < 2; ++c)
        for (const auto& t : cars[c].trip_plans)
            total_pkgs += (int)t.size();

    int k = std::max(1, (int)std::round(total_pkgs * ratio));

    // 对每个包裹打分: dist[0][dest] * weight
    vector<std::pair<double, std::pair<int, int>>> scored;
    // scored = (score, (car_idx, trip_idx, pos))
    for (int c = 0; c < 2; ++c)
        for (int ti = 0; ti < (int)cars[c].trip_plans.size(); ++ti)
            for (int pos = 0; pos < (int)cars[c].trip_plans[ti].size(); ++pos)
            {
                int pid = cars[c].trip_plans[ti][pos];
                double score = ap.dist[0][pkgs[pid].dest] * pkgs[pid].weight;
                scored.emplace_back(score, std::make_pair(c, ti * 10000 + pos));
            }

    // 按分数降序, 取前 k 个
    std::sort(scored.rbegin(), scored.rend());
    vector<int> removed;
    std::unordered_set<int> removed_set;

    for (int i = 0; i < k && i < (int)scored.size(); ++i)
    {
        int c = scored[i].second.first;
        int ti = scored[i].second.second / 10000;
        int pos = scored[i].second.second % 10000;
        int pid = cars[c].trip_plans[ti][pos];
        if (removed_set.count(pid)) continue;
        removed.push_back(pid);
        removed_set.insert(pid);

        // 从原方案中删除
        cars[c].trip_plans[ti].erase(cars[c].trip_plans[ti].begin() + pos);
        // 删除后更新 dest_orders
        if (cars[c].trip_plans[ti].empty())
        {
            cars[c].trip_plans.erase(cars[c].trip_plans.begin() + ti);
            cars[c].dest_orders.erase(cars[c].dest_orders.begin() + ti);
        }
        else
        {
            cars[c].dest_orders[ti] = heaviest_dest_order(cars[c].trip_plans[ti]);
        }
    }

    return removed;
}

// ====================================================================
// ALNS 破坏算子: Shaw-removal
// 随机选种子, 移除与其最相关的 k-1 个包裹
// 相关度 = γ1×地理距离 + γ2×deadline差异 + γ3×重量差异 (归一化)
// γ1=0.5, γ2=0.3, γ3=0.2
// ====================================================================
vector<int> Task5::destroy_shaw(CarPlan cars[2])
{
    const double ratio = 0.2;
    const double gamma1 = 0.5, gamma2 = 0.3, gamma3 = 0.2;

    // 收集所有包裹及位置信息
    struct PkgInfo { int pid; int car; int trip; int pos; };
    vector<PkgInfo> all_pkgs;
    for (int c = 0; c < 2; ++c)
        for (int ti = 0; ti < (int)cars[c].trip_plans.size(); ++ti)
            for (int pos = 0; pos < (int)cars[c].trip_plans[ti].size(); ++pos)
                all_pkgs.push_back({cars[c].trip_plans[ti][pos], c, ti, pos});

    int total = (int)all_pkgs.size();
    if (total <= 1) return {};

    int k = std::max(1, std::min((int)std::round(total * ratio), total));

    // 随机选种子
    std::uniform_int_distribution<int> uid(0, total - 1);
    int seed_idx = uid(rng);
    int seed_pid = all_pkgs[seed_idx].pid;

    // 计算归一化参数的上下界
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
    if (max_dist - min_dist < 1e-12) max_dist = min_dist + 1.0;
    if (max_dl - min_dl < 1e-12) max_dl = min_dl + 1.0;
    if (max_w - min_w < 1e-12) max_w = min_w + 1.0;

    // 计算相关度 (越小越相关)
    vector<std::pair<double, int>> related; // (relatedness, idx in all_pkgs)
    for (int i = 0; i < total; ++i)
    {
        if (i == seed_idx) continue;
        int pid = all_pkgs[i].pid;

        double dist_norm = (ap.dist[pkgs[seed_pid].dest][pkgs[pid].dest] - min_dist) / (max_dist - min_dist);
        double dl_norm   = (std::abs(pkgs[seed_pid].deadline - pkgs[pid].deadline) - min_dl) / (max_dl - min_dl);
        double w_norm    = (std::abs(pkgs[seed_pid].weight - pkgs[pid].weight) - min_w) / (max_w - min_w);

        double rel = gamma1 * dist_norm + gamma2 * dl_norm + gamma3 * w_norm;
        related.emplace_back(rel, i);
    }

    std::sort(related.begin(), related.end()); // 升序, 相关度最高的在前

    vector<int> removed;
    removed.push_back(seed_pid);

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
                cars[c].dest_orders[ti] = heaviest_dest_order(t);
            }
        }
    }

    return removed;
}

// ====================================================================
// ALNS 破坏算子: Trip-destroy
// 随机选一辆车的一整趟, 全部移除
// ====================================================================
vector<int> Task5::destroy_trip(CarPlan cars[2])
{
    // 收集所有非空趟
    vector<std::pair<int, int>> trips; // (car_idx, trip_idx)
    for (int c = 0; c < 2; ++c)
        for (int ti = 0; ti < (int)cars[c].trip_plans.size(); ++ti)
            if (!cars[c].trip_plans[ti].empty())
                trips.emplace_back(c, ti);

    if (trips.empty()) return {};

    std::uniform_int_distribution<int> uid(0, (int)trips.size() - 1);
    auto [sel_c, sel_t] = trips[uid(rng)];

    vector<int> removed = cars[sel_c].trip_plans[sel_t];

    cars[sel_c].trip_plans.erase(cars[sel_c].trip_plans.begin() + sel_t);
    cars[sel_c].dest_orders.erase(cars[sel_c].dest_orders.begin() + sel_t);

    return removed;
}

// ====================================================================
// ALNS 破坏算子: Time-window-removal
// 移除所有当前超时的包裹 + 随机若干同趟包裹
// ====================================================================
vector<int> Task5::destroy_time_window(CarPlan cars[2])
{
    // 需要先评估当前方案获取超时包裹
    vector<double> deliv0, deliv1;
    evaluate_full(cars[0], cars[1], deliv0, deliv1);

    vector<int> overtime_pkgs;
    for (int pid = 0; pid < n_pkg; ++pid)
    {
        double d_time = (deliv0[pid] > 0) ? deliv0[pid] : deliv1[pid];
        if (d_time > pkgs[pid].deadline + 1e-9)
            overtime_pkgs.push_back(pid);
    }

    if (overtime_pkgs.empty()) return {};

    std::unordered_set<int> removed_set(overtime_pkgs.begin(), overtime_pkgs.end());

    // 额外移除若干同趟包裹 (约 30% 的同趟包裹)
    vector<int> extra;
    for (int c = 0; c < 2; ++c)
        for (int ti = 0; ti < (int)cars[c].trip_plans.size(); ++ti)
        {
            bool has_ot = false;
            for (int pid : cars[c].trip_plans[ti])
                if (removed_set.count(pid)) { has_ot = true; break; }
            if (!has_ot) continue;

            // 随机移除该趟中约 30% 的包裹
            int n_in_trip = (int)cars[c].trip_plans[ti].size();
            int n_remove = std::max(1, (int)std::round(n_in_trip * 0.3));
            vector<int> candidates;
            for (int pid : cars[c].trip_plans[ti])
                if (!removed_set.count(pid)) candidates.push_back(pid);
            std::shuffle(candidates.begin(), candidates.end(), rng);
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
                cars[c].dest_orders[ti] = heaviest_dest_order(t);
            }
        }

    return removed;
}

// ====================================================================
// ALNS 修复算子: Greedy-cheapest (代理版)
// 对每个待插入包裹, 遍历两车所有趟的所有可能插入位置
// 代理指标: 局部边权增量 Δ = dist[prev][dest_i] + dist[dest_i][next] - dist[prev][next]
// 选择 Δ 最小的位置; 若容量不足则新增一趟
// 每次插入后更新趟结构, 再处理下一个包裹
// ====================================================================
void Task5::repair_greedy(vector<int>& pool, CarPlan cars[2])
{
    while (!pool.empty())
    {
        double best_delta = 1e18;
        int best_pool_idx = -1;
        int best_car = -1, best_trip = -1, best_pos = -1;

        for (size_t pi = 0; pi < pool.size(); ++pi)
        {
            int pid = pool[pi];
            int dest = pkgs[pid].dest;

            for (int c = 0; c < 2; ++c)
            {
                auto& plan = cars[c];
                int n_trips = (int)plan.trip_plans.size();

                // 情况1: 插入到已有的某趟中
                for (int ti = 0; ti < n_trips; ++ti)
                {
                    // 检查容量
                    if (trip_weight(plan.trip_plans[ti]) + pkgs[pid].weight > c1.capacity + 1e-9)
                        continue;

                    // 已存在同目的地则不重复插入 dest_orders
                    // 遍历 dest_orders 中所有可能的插入位置
                    const auto& d_order = plan.dest_orders[ti];
                    int nd = (int)d_order.size();

                    // 先查该目的地是否已在趟中
                    bool dest_exists = false;
                    for (int d : d_order)
                        if (d == dest) { dest_exists = true; break; }

                    // 尝试在每两个相邻目的地之间插入
                    for (int pos = 0; pos <= nd; ++pos)
                    {
                        if (dest_exists && pos > 0 && d_order[pos - 1] == dest) continue; // 同目的地不重复位置
                        int prev_node = (pos == 0) ? 0 : d_order[pos - 1];
                        int next_node = (pos == nd) ? 0 : d_order[pos];
                        double delta = ap.dist[prev_node][dest] + ap.dist[dest][next_node] - ap.dist[prev_node][next_node];
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

                // 情况2: 新增一趟 (只运这一个包裹)
                {
                    double delta = ap.dist[0][dest] + ap.dist[dest][0]; // 往返
                    if (delta < best_delta)
                    {
                        best_delta = delta;
                        best_pool_idx = (int)pi;
                        best_car = c;
                        best_trip = -1; // 标记为新增趟
                        best_pos = -1;
                    }
                }
            }
        }

        if (best_pool_idx == -1) { pool.clear(); return; } // 不应该发生

        int pid = pool[best_pool_idx];
        int dest = pkgs[pid].dest;

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

            // 更新 dest_orders
            auto& d_order = plan.dest_orders[best_trip];
            // 检查 dest 是否已在 d_order 中
            bool exists = false;
            for (int d : d_order)
                if (d == dest) { exists = true; break; }
            if (!exists)
                d_order.insert(d_order.begin() + best_pos, dest);
        }

        // 从 pool 中移除
        pool.erase(pool.begin() + best_pool_idx);
    }
}

// ====================================================================
// ALNS 修复算子: Regret-2
// 对每个待插入包裹计算 regret = 次佳Δ - 最佳Δ
// 优先插入 regret 最大的包裹 (避免"先来先占")
// ====================================================================
void Task5::repair_regret2(vector<int>& pool, CarPlan cars[2])
{
    while (!pool.empty())
    {
        // 对 pool 中每个包裹计算最佳和次佳 Δ
        struct InsertInfo
        {
            double best_delta;
            double second_delta;
            int best_car, best_trip, best_pos; // 最佳位置
        };
        vector<InsertInfo> infos(pool.size());

        for (size_t pi = 0; pi < pool.size(); ++pi)
        {
            int pid = pool[pi];
            int dest = pkgs[pid].dest;

            double d1 = 1e18, d2 = 1e18;
            int bc = -1, bt = -1, bp = -1;

            for (int c = 0; c < 2; ++c)
            {
                auto& plan = cars[c];
                int n_trips = (int)plan.trip_plans.size();

                // 已有趟的插入位置
                for (int ti = 0; ti < n_trips; ++ti)
                {
                    if (trip_weight(plan.trip_plans[ti]) + pkgs[pid].weight > c1.capacity + 1e-9)
                        continue;

                    const auto& d_order = plan.dest_orders[ti];
                    int nd = (int)d_order.size();

                    // 先查该目的地是否已在趟中
                    bool dest_exists = false;
                    for (int d : d_order)
                        if (d == dest) { dest_exists = true; break; }

                    for (int pos = 0; pos <= nd; ++pos)
                    {
                        if (dest_exists && pos > 0 && d_order[pos - 1] == dest) continue;
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

            if (d2 >= 1e18) d2 = d1 + 1000.0; // 只有一个可行位置时给一个大 regret
            infos[pi] = {d1, d2, bc, bt, bp};
        }

        // 找 regret 最大的包裹
        double max_regret = -1.0;
        int chosen = -1;
        for (size_t pi = 0; pi < pool.size(); ++pi)
        {
            double regret = infos[pi].second_delta - infos[pi].best_delta;
            if (regret > max_regret)
            {
                max_regret = regret;
                chosen = (int)pi;
            }
        }

        if (chosen == -1) { pool.clear(); return; } // 不应该发生

        int pid = pool[chosen];
        int dest = pkgs[pid].dest;
        auto& info = infos[chosen];

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
// SA 优化主循环
// 评分函数: score = total_cost (超时不参与)
// 温控: Lundy-Mees 恒温退火 T_{k+1} = T_k / (1 + β·T_k)
// ====================================================================
void Task5::sa_optimize(CarPlan cars[2])
{
    // 当前解评估
    vector<double> cur_deliv0, cur_deliv1;
    auto [cur_cost, cur_ot] = evaluate_full(cars[0], cars[1], cur_deliv0, cur_deliv1);
    double cur_score = cur_cost;

    double best_score = cur_score;
    CarPlan best_cars[2] = {cars[0], cars[1]};

    // ---- 自适应初始温度 ----
    // 对初解做 100 次随机扰动, 用上坡 Δcost 的中位数反推 T_init (接受率 ≈ 0.8)
    vector<double> deltas;
    for (int k = 0; k < 100; ++k)
    {
        CarPlan tmp[2] = {cars[0], cars[1]};

        // 随机选一个破坏算子
        int destroy_op = std::uniform_int_distribution<int>(0, 3)(rng);
        vector<int> pool;
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
        double d = new_cost - cur_cost;
        if (d > 0) deltas.push_back(d);
    }

    double T = 1.0;
    if (!deltas.empty())
    {
        std::sort(deltas.begin(), deltas.end());
        double delta_med = deltas[deltas.size() / 2];
        T = delta_med / 0.223; // exp(-Δ/T) = 0.8 → T = Δ / ln(1/0.8)
        if (T < 0.1) T = 0.1;
    }

    // ---- Lundy-Mees 参数 ----
    const int    MAX_ITER = 100000;
    const double T_MIN    = 1e-3;
    double beta = (T - T_MIN) / (MAX_ITER * T * T_MIN);

    // 算子自适应权重 (初始均等)
    int destroy_weights[4] = {1, 1, 1, 1};
    int repair_weights[2]  = {1, 1};
    int last_destroy = 0, last_repair = 0;

    std::uniform_real_distribution<double> uni(0.0, 1.0);

    // ---- 主循环 ----
    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        // 保存当前方案用于回退
        CarPlan saved[2] = {cars[0], cars[1]};

        // ---- 轮盘赌选破坏算子 ----
        int dw_sum = destroy_weights[0] + destroy_weights[1] + destroy_weights[2] + destroy_weights[3];
        std::uniform_int_distribution<int> dw_die(0, dw_sum - 1);
        int r_d = dw_die(rng);
        int destroy_op;
        {
            int acc = 0;
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
        vector<int> pool;
        switch (destroy_op)
        {
            case 0: pool = destroy_worst(cars);  break;
            case 1: pool = destroy_shaw(cars);   break;
            case 2: pool = destroy_trip(cars);   break;
            case 3: pool = destroy_time_window(cars); break;
        }

        if (pool.empty())
        {
            // 无效操作, 回退并降温
            cars[0] = saved[0];
            cars[1] = saved[1];
            goto COOL;
        }

        // ---- 执行修复 ----
        if (repair_op == 0) repair_greedy(pool, cars);
        else                repair_regret2(pool, cars);

        // ---- 完整评估新解 ----
        {
            vector<double> nd0, nd1;
            auto [new_cost, new_ot] = evaluate_full(cars[0], cars[1], nd0, nd1);
            double delta = new_cost - cur_cost;

            if (delta < 0 || uni(rng) < std::exp(-delta / T))
            {
                // 接受新解
                cur_cost  = new_cost;
                cur_ot    = new_ot;
                cur_deliv0 = nd0;
                cur_deliv1 = nd1;

                if (new_cost < best_score)
                {
                    best_score = new_cost;
                    best_cars[0] = cars[0];
                    best_cars[1] = cars[1];

                    // 自适应权重: 降低 best_cost 的算子权重+1
                    ++destroy_weights[last_destroy];
                    ++repair_weights[last_repair];
                }
            }
            else
            {
                // 拒绝, 回退
                cars[0] = saved[0];
                cars[1] = saved[1];
            }
        }

    COOL:
        T = T / (1.0 + beta * T);
        if (T < T_MIN) break;
    }

    // 恢复最优解
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
// 1. K-means 分区 → 2. 贪心初解 → 3. SA+ALNS 优化 → 4. 构造输出
// ====================================================================
Task5Result Task5::solve()
{
    // ---- 1. K-means 初始分区 ----
    auto [cluster0, cluster1] = kmeans_partition();

    // ---- 2. 贪心初解 (每车独立, 与 T3 一致) ----
    CarPlan cars0, cars1;
    greedy_init_one(cluster0, cars0);
    greedy_init_one(cluster1, cars1);

    CarPlan cars_arr[2] = {cars0, cars1};

    // ---- 3. SA + ALNS 优化 ----
    sa_optimize(cars_arr);

    // ---- 4. 最终评估 & 构造输出 ----
    vector<double> deliv0, deliv1;
    auto [total_cost, overtime] = evaluate_full(cars_arr[0], cars_arr[1], deliv0, deliv1);

    // 构造 trips1
    trips1.clear();
    double cur_time1 = 0.0;
    for (size_t t = 0; t < cars_arr[0].trip_plans.size(); ++t)
    {
        const auto& pkg_ids = cars_arr[0].trip_plans[t];
        const auto& dest_order = cars_arr[0].dest_orders[t];
        if (pkg_ids.empty()) continue;
        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, cur_time1, pkgs, c1, ap);
        trips1.push_back(sim);
        cur_time1 = sim.end_time;
    }

    // 构造 trips2
    trips2.clear();
    double cur_time2 = 0.0;
    for (size_t t = 0; t < cars_arr[1].trip_plans.size(); ++t)
    {
        const auto& pkg_ids = cars_arr[1].trip_plans[t];
        const auto& dest_order = cars_arr[1].dest_orders[t];
        if (pkg_ids.empty()) continue;
        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, cur_time2, pkgs, c2, ap);
        trips2.push_back(sim);
        cur_time2 = sim.end_time;
    }

    Task5Result res;
    res.trips1  = trips1;
    res.trips2  = trips2;
    res.sumCost = total_cost;
    res.exTime  = overtime;
    return res;
}