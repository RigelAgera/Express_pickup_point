// T2: 最小化不满意度之和
// 约束：不考虑超时（Ti=INF），包裹在其 Si 之后才可配送，容量 w_lim 约束
// 算法: 贪心初解 + 模拟退火(SA) 优化，480同学让AI写的，对不对未知，她目前也看不懂
// 编译: g++ -std=c++17 -O2 task/task2.cpp common/input_reader.cpp -o task2.exe
#pragma once
#include "../common/input_reader.h"
#include "../common/dijkstra.h"
#include "../common/min_heap.h"
#include "../common/circular_queue.h"
#include "../common/schedule_utils.h"
#include <iostream>
#include <iomanip>
#include <numeric>
#include <vector>
#include <random>
#include <chrono>

// ---- T2 结果 ----
struct Task2Result
{
    std::vector<Trip> trips;
    double total_dissatisfaction;
    std::vector<double> deliver_time;           // 每个包裹的送达时间 Di（按原始 id 索引）
    std::vector<double> dissatisfaction_per_pkg; // 每个包裹的不满意度 Di - Si
};

// ---- Task2 命名空间 ----
namespace Task2
{
    Task2Result solve(const input_data& data);
}

// ---- 辅助: 提取唯一目的地 ----
inline std::vector<int> unique_dests(const std::vector<int>& pkg_ids,
                                     const std::vector<package>& pkgs)
{
    std::vector<int> dests;
    for (int pid : pkg_ids)
        dests.push_back(pkgs[pid].dest);
    std::sort(dests.begin(), dests.end());
    dests.erase(std::unique(dests.begin(), dests.end()), dests.end());
    return dests;
}

// ---- 辅助: 计算一趟的总重量 ----
inline double trip_weight(const std::vector<int>& pkg_ids,
                          const std::vector<package>& pkgs)
{
    double w = 0.0;
    for (int pid : pkg_ids)
        w += pkgs[pid].weight;
    return w;
}

// ---- 核心: 评估一个完整方案的不满意度 ----
// trip_plans[t] = 该趟的包裹 id 列表（作为分组，具体送达顺序由 dest_order 决定）
// dest_orders[t] = 该趟的唯一目的地访问顺序
// 返回总不满意度，并填入 deliver_time[], dissatisfaction[]
inline double evaluate_solution(
    const std::vector<std::vector<int>>& trip_plans,
    const std::vector<std::vector<int>>& dest_orders,
    const std::vector<package>& pkgs,
    const car& c,
    const AllPairResult& ap,
    std::vector<double>& deliver_time,
    std::vector<double>& dissatisfaction)
{
    int n_pkg = (int)pkgs.size();
    deliver_time.assign(n_pkg, -1.0);
    dissatisfaction.assign(n_pkg, 0.0);

    double cur_time = 0.0;  // 当前时刻（小车回到驿站的时间）
    double total = 0.0;

    for (size_t t_idx = 0; t_idx < trip_plans.size(); ++t_idx)
    {
        const auto& pkg_ids = trip_plans[t_idx];
        const auto& dest_order = dest_orders[t_idx];
        if (pkg_ids.empty()) continue;

        // 本趟出发时间 = max(小车可用时间, 本趟所有包裹的 max(Si))
        double max_S = 0.0;
        for (int pid : pkg_ids)
            if (pkgs[pid].arrive > max_S)
                max_S = pkgs[pid].arrive;
        double depart = std::max(cur_time, max_S);

        // 构建路线
        std::vector<int> route = build_route(dest_order, ap);

        // 模拟本趟
        Trip sim = simulate_trip(pkg_ids, route, depart, pkgs, c, ap);

        // 填入送达时间
        for (size_t i = 0; i < pkg_ids.size(); ++i)
        {
            int pid = pkg_ids[i];
            deliver_time[pid] = sim.deliver_time[i];
            dissatisfaction[pid] = sim.deliver_time[i] - pkgs[pid].arrive;
            total += dissatisfaction[pid];
        }

        cur_time = sim.end_time;  // 本趟返回驿站的时间
    }
    return total;
}

// ---- 核心: 对一趟的唯一目的地做 nearest_first 排序 ----
inline std::vector<int> nearest_dest_order(const std::vector<int>& pkg_ids,
                                           const std::vector<package>& pkgs,
                                           const AllPairResult& ap)
{
    std::vector<int> dests = unique_dests(pkg_ids, pkgs);
    return nearest_first_order(dests, ap);
}

// ---- 贪心初解 ----
// 按 Si 排序 → 贪心装车（容量约束）→ 趟内 nearest_first
inline void greedy_init(const std::vector<package>& pkgs,
                        const car& c,
                        const AllPairResult& ap,
                        std::vector<std::vector<int>>& trip_plans,
                        std::vector<std::vector<int>>& dest_orders)
{
    int n = (int)pkgs.size();

    // 按 Si 排序
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return pkgs[a].arrive < pkgs[b].arrive; });

    trip_plans.clear();
    dest_orders.clear();

    std::vector<int> cur_trip;
    double cur_weight = 0.0;

    for (int pid : order)
    {
        double w = pkgs[pid].weight;
        if (cur_weight + w > c.capacity + 1e-9 && !cur_trip.empty())
        {
            // 关闭当前趟，开新趟
            trip_plans.push_back(cur_trip);
            dest_orders.push_back(nearest_dest_order(cur_trip, pkgs, ap));
            cur_trip.clear();
            cur_weight = 0.0;
        }
        cur_trip.push_back(pid);
        cur_weight += w;
    }

    if (!cur_trip.empty())
    {
        trip_plans.push_back(cur_trip);
        dest_orders.push_back(nearest_dest_order(cur_trip, pkgs, ap));
    }
}

// ---- 尝试一次随机邻域移动（返回修改后的 plans/orders），不做接受判断 ----
// 如果无法产生合法 move，返回 false（退化为原解）
inline bool try_random_move(
    std::vector<std::vector<int>>& trip_plans,
    std::vector<std::vector<int>>& dest_orders,
    const std::vector<package>& pkgs,
    const car& c,
    const AllPairResult& ap,
    std::mt19937& rng)
{
    int n_trip_cur = (int)trip_plans.size();
    int op = std::uniform_int_distribution<int>(0, 2)(rng);

    auto saved_plans = trip_plans;
    auto saved_orders = dest_orders;

    if (op == 0 && n_trip_cur >= 2)              // 跨趟搬包裹
    {
        int src = std::uniform_int_distribution<int>(0, n_trip_cur - 1)(rng);
        if (trip_plans[src].size() <= 1) return false;
        int pos = std::uniform_int_distribution<int>(0, (int)trip_plans[src].size() - 1)(rng);
        int pid = trip_plans[src][pos];
        double pw = pkgs[pid].weight;
        int dst;
        do { dst = std::uniform_int_distribution<int>(0, n_trip_cur - 1)(rng); } while (dst == src);
        if (trip_weight(trip_plans[dst], pkgs) + pw > c.capacity + 1e-9) return false;
        trip_plans[src].erase(trip_plans[src].begin() + pos);
        trip_plans[dst].push_back(pid);
        dest_orders[src] = nearest_dest_order(trip_plans[src], pkgs, ap);
        dest_orders[dst] = nearest_dest_order(trip_plans[dst], pkgs, ap);
        if (trip_plans[src].empty())
        {
            trip_plans.erase(trip_plans.begin() + src);
            dest_orders.erase(dest_orders.begin() + src);
        }
        return true;
    }
    else if (op == 1 && n_trip_cur >= 1)         // 趟内交换两个目的地
    {
        int t = std::uniform_int_distribution<int>(0, n_trip_cur - 1)(rng);
        if (dest_orders[t].size() < 2) return false;
        int i = std::uniform_int_distribution<int>(0, (int)dest_orders[t].size() - 1)(rng);
        int j = std::uniform_int_distribution<int>(0, (int)dest_orders[t].size() - 1)(rng);
        if (i == j) return false;
        std::swap(dest_orders[t][i], dest_orders[t][j]);
        return true;
    }
    else if (op == 2 && n_trip_cur >= 1)         // 趟内 2-opt
    {
        int t = std::uniform_int_distribution<int>(0, n_trip_cur - 1)(rng);
        int sz = (int)dest_orders[t].size();
        if (sz < 2) return false;
        int i = std::uniform_int_distribution<int>(0, sz - 2)(rng);
        int j = std::uniform_int_distribution<int>(i + 1, sz - 1)(rng);
        std::reverse(dest_orders[t].begin() + i, dest_orders[t].begin() + j + 1);
        return true;
    }
    return false;
}

// ---- 模拟退火（自适应参数，Lundy-Mees 恒温退火）----
inline void sa_optimize(std::vector<std::vector<int>>& trip_plans,
                        std::vector<std::vector<int>>& dest_orders,
                        const std::vector<package>& pkgs,
                        const car& c,
                        const AllPairResult& ap,
                        std::mt19937& rng)
{
    int n_pkg = (int)pkgs.size();

    // --- 1. 自适应估计 T_init ---
    // 对初解做100次随机扰动，用 Δ 的中位数反推使初始接受率≈0.8的温度
    std::vector<double> deltas;
    std::vector<double> dummy_deliv, dummy_diss;
    double cur_score = evaluate_solution(trip_plans, dest_orders, pkgs, c, ap,
                                         dummy_deliv, dummy_diss);
    for (int k = 0; k < 100; ++k)
    {
        auto p2 = trip_plans, o2 = dest_orders;
        if (!try_random_move(p2, o2, pkgs, c, ap, rng)) continue;
        double ns = evaluate_solution(p2, o2, pkgs, c, ap, dummy_deliv, dummy_diss);
        double d = ns - cur_score;
        if (d > 0) deltas.push_back(d);  // 只统计 uphill moves
    }
    double T = 1.0;
    if (!deltas.empty())
    {
        std::sort(deltas.begin(), deltas.end());
        double delta_med = deltas[deltas.size() / 2];   // 中位数
        T = delta_med / 0.223;                           // exp(−Δ/T)=0.8 → T=Δ/ln(1/0.8)
        if (T < 0.1) T = 0.1;
    }

    // --- 2. Lundy-Mees 参数 ---
    const int    MAX_ITER = 100000;
    const double T_MIN    = 1e-3;
    // β 使得经过 MAX_ITER 步后从 T 降到 T_MIN
    double beta = (T - T_MIN) / (MAX_ITER * T * T_MIN);

    // --- 3. 当前解状态 ---
    std::vector<int> pkg_trip(n_pkg, -1);
    auto rebuild_pkg_trip = [&]() {
        for (int t = 0; t < (int)trip_plans.size(); ++t)
            for (int pid : trip_plans[t])
                pkg_trip[pid] = t;
    };
    rebuild_pkg_trip();

    std::vector<double> cur_deliv(n_pkg), cur_diss(n_pkg);
    cur_score = evaluate_solution(trip_plans, dest_orders, pkgs, c, ap,
                                  cur_deliv, cur_diss);
    double best_score = cur_score;
    auto best_plans = trip_plans;
    auto best_orders = dest_orders;

    std::uniform_real_distribution<double> uni(0.0, 1.0);

    // --- 4. 主循环（每步都降温） ---
    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        auto saved_plans = trip_plans;
        auto saved_orders = dest_orders;

        if (!try_random_move(trip_plans, dest_orders, pkgs, c, ap, rng))
        {
            trip_plans = saved_plans;
            dest_orders = saved_orders;
            goto COOL;
        }

        {
            std::vector<double> nd(n_pkg), nds(n_pkg);
            double ns = evaluate_solution(trip_plans, dest_orders, pkgs, c, ap, nd, nds);
            double delta = ns - cur_score;

            if (delta < 0 || uni(rng) < std::exp(-delta / T))
            {
                cur_score = ns;
                cur_deliv = nd;
                cur_diss = nds;
                rebuild_pkg_trip();
                if (cur_score < best_score)
                {
                    best_score = cur_score;
                    best_plans = trip_plans;
                    best_orders = dest_orders;
                }
            }
            else
            {
                trip_plans = saved_plans;
                dest_orders = saved_orders;
            }
        }

    COOL:
        T = T / (1.0 + beta * T);
        if (T < T_MIN) break;
    }

    trip_plans = best_plans;
    dest_orders = best_orders;
}

// ---- Task2::solve 主入口 ----
inline Task2Result Task2::solve(const input_data& data)
{
    // 1. 确保所有 deadline = 1e9（T2 约束）
    std::vector<package> pkgs = data.packages;
    for (auto& p : pkgs)
        p.deadline = 1e9;

    car c = data.c;

    // 2. 全源最短路
    AllPairResult ap = all_pairs_dijkstra(data.g);

    // 3. 贪心初解
    std::vector<std::vector<int>> trip_plans, dest_orders;
    greedy_init(pkgs, c, ap, trip_plans, dest_orders);

    // 4. 模拟退火优化
    std::mt19937 rng((unsigned)std::chrono::steady_clock::now().time_since_epoch().count());
    sa_optimize(trip_plans, dest_orders, pkgs, c, ap, rng);

    // 5. 最终评估，构造输出
    int n_pkg = (int)pkgs.size();
    std::vector<double> deliver_time(n_pkg), dissatisfaction(n_pkg);
    double total = evaluate_solution(trip_plans, dest_orders, pkgs, c, ap,
                                     deliver_time, dissatisfaction);

    // 6. 构造 Trip 列表（含完整路线）
    std::vector<Trip> trips;
    double cur_time = 0.0;
    for (size_t t_idx = 0; t_idx < trip_plans.size(); ++t_idx)
    {
        const auto& pkg_ids = trip_plans[t_idx];
        const auto& dest_order = dest_orders[t_idx];
        if (pkg_ids.empty()) continue;

        double max_S = 0.0;
        for (int pid : pkg_ids)
            if (pkgs[pid].arrive > max_S)
                max_S = pkgs[pid].arrive;
        double depart = std::max(cur_time, max_S);

        std::vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, depart, pkgs, c, ap);

        trips.push_back(sim);
        cur_time = sim.end_time;
    }

    return { trips, total, deliver_time, dissatisfaction };
}