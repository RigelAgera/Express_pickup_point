// T2: 最小化不满意度之和
// 编译: g++ -std=c++17 -O2 task/task2.cpp common/input_reader.cpp -o task2.exe
#include "task2.h"
#include "../common/dijkstra.h"
#include "../common/min_heap.h"
#include "../common/circular_queue.h"
#include "../common/schedule_utils.h"
#include <iostream>
#include <iomanip>
#include <numeric>
#include <algorithm>
#include <chrono>

using std::vector;

// ============ 构造 ============

Task2::Task2(const input_data& data)
    : pkgs(data.packages), c(data.c),
      ap(all_pairs_dijkstra(data.g)),
      n_pkg((int)pkgs.size()),
      rng((unsigned)std::chrono::steady_clock::now().time_since_epoch().count())
{
    // T2 约束：所有 deadline = INF
    for (auto& p : pkgs)
        p.deadline = 1e9;
}

// ============ 辅助 ============

vector<int> Task2::unique_dests(const vector<int>& pkg_ids)
{
    vector<int> dests;
    for (int pid : pkg_ids)
        dests.push_back(pkgs[pid].dest);
    std::sort(dests.begin(), dests.end());
    dests.erase(std::unique(dests.begin(), dests.end()), dests.end());
    return dests;
}

double Task2::trip_weight(const vector<int>& pkg_ids)
{
    double w = 0.0;
    for (int pid : pkg_ids)
        w += pkgs[pid].weight;
    return w;
}

// ============ 核心 ============

vector<int> Task2::nearest_dest_order(const vector<int>& pkg_ids)
{
    vector<int> dests = unique_dests(pkg_ids);
    return nearest_first_order(dests, ap);
}

double Task2::evaluate_solution(
    const vector<vector<int>>& trip_plans,
    const vector<vector<int>>& dest_orders,
    vector<double>& deliver_time,
    vector<double>& dissatisfaction)
{
    deliver_time.assign(n_pkg, -1.0);
    dissatisfaction.assign(n_pkg, 0.0);

    double cur_time = 0.0;
    double total = 0.0;

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

        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, depart, pkgs, c, ap);

        for (size_t i = 0; i < pkg_ids.size(); ++i)
        {
            int pid = pkg_ids[i];
            deliver_time[pid] = sim.deliver_time[i];
            dissatisfaction[pid] = sim.deliver_time[i] - pkgs[pid].arrive;
            total += dissatisfaction[pid];
        }

        cur_time = sim.end_time;
    }
    return total;
}

// ============ 贪心初解 ============

void Task2::greedy_init(vector<vector<int>>& trip_plans,
                         vector<vector<int>>& dest_orders)
{
    vector<int> order(n_pkg);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return pkgs[a].arrive < pkgs[b].arrive; });

    trip_plans.clear();
    dest_orders.clear();

    vector<int> cur_trip;
    double cur_weight = 0.0;

    for (int pid : order)
    {
        double w = pkgs[pid].weight;
        if (cur_weight + w > c.capacity + 1e-9 && !cur_trip.empty())
        {
            trip_plans.push_back(cur_trip);
            dest_orders.push_back(nearest_dest_order(cur_trip));
            cur_trip.clear();
            cur_weight = 0.0;
        }
        cur_trip.push_back(pid);
        cur_weight += w;
    }

    if (!cur_trip.empty())
    {
        trip_plans.push_back(cur_trip);
        dest_orders.push_back(nearest_dest_order(cur_trip));
    }
}

// ============ 邻域移动 ============

bool Task2::try_random_move(vector<vector<int>>& trip_plans,
                             vector<vector<int>>& dest_orders)
{
    int n_trip_cur = (int)trip_plans.size();
    int op = std::uniform_int_distribution<int>(0, 2)(rng);

    auto saved_plans = trip_plans;
    auto saved_orders = dest_orders;

    if (op == 0 && n_trip_cur >= 2)
    {
        int src = std::uniform_int_distribution<int>(0, n_trip_cur - 1)(rng);
        if (trip_plans[src].size() <= 1) return false;
        int pos = std::uniform_int_distribution<int>(0, (int)trip_plans[src].size() - 1)(rng);
        int pid = trip_plans[src][pos];
        double pw = pkgs[pid].weight;
        int dst;
        do { dst = std::uniform_int_distribution<int>(0, n_trip_cur - 1)(rng); } while (dst == src);
        if (trip_weight(trip_plans[dst]) + pw > c.capacity + 1e-9) return false;
        trip_plans[src].erase(trip_plans[src].begin() + pos);
        trip_plans[dst].push_back(pid);
        dest_orders[src] = nearest_dest_order(trip_plans[src]);
        dest_orders[dst] = nearest_dest_order(trip_plans[dst]);
        if (trip_plans[src].empty())
        {
            trip_plans.erase(trip_plans.begin() + src);
            dest_orders.erase(dest_orders.begin() + src);
        }
        return true;
    }
    else if (op == 1 && n_trip_cur >= 1)
    {
        int t = std::uniform_int_distribution<int>(0, n_trip_cur - 1)(rng);
        if (dest_orders[t].size() < 2) return false;
        int i = std::uniform_int_distribution<int>(0, (int)dest_orders[t].size() - 1)(rng);
        int j = std::uniform_int_distribution<int>(0, (int)dest_orders[t].size() - 1)(rng);
        if (i == j) return false;
        std::swap(dest_orders[t][i], dest_orders[t][j]);
        return true;
    }
    else if (op == 2 && n_trip_cur >= 1)
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

// ============ 模拟退火 ============

void Task2::sa_optimize(vector<vector<int>>& trip_plans,
                         vector<vector<int>>& dest_orders)
{
    vector<double> dummy_deliv, dummy_diss;
    double cur_score = evaluate_solution(trip_plans, dest_orders, dummy_deliv, dummy_diss);

    // 自适应 T_init
    vector<double> deltas;
    for (int k = 0; k < 100; ++k)
    {
        auto p2 = trip_plans, o2 = dest_orders;
        if (!try_random_move(p2, o2)) continue;
        double ns = evaluate_solution(p2, o2, dummy_deliv, dummy_diss);
        double d = ns - cur_score;
        if (d > 0) deltas.push_back(d);
    }
    double T = 1.0;
    if (!deltas.empty())
    {
        std::sort(deltas.begin(), deltas.end());
        double delta_med = deltas[deltas.size() / 2];
        T = delta_med / 0.223;
        if (T < 0.1) T = 0.1;
    }

    // Lundy-Mees
    const int    MAX_ITER = 100000;
    const double T_MIN    = 1e-3;
    double beta = (T - T_MIN) / (MAX_ITER * T * T_MIN);

    // pkg → trip 映射
    vector<int> pkg_trip(n_pkg, -1);
    auto rebuild_pkg_trip = [&]() {
        for (int t = 0; t < (int)trip_plans.size(); ++t)
            for (int pid : trip_plans[t])
                pkg_trip[pid] = t;
    };
    rebuild_pkg_trip();

    vector<double> cur_deliv(n_pkg), cur_diss(n_pkg);
    cur_score = evaluate_solution(trip_plans, dest_orders, cur_deliv, cur_diss);
    double best_score = cur_score;
    auto best_plans = trip_plans;
    auto best_orders = dest_orders;

    std::uniform_real_distribution<double> uni(0.0, 1.0);

    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        auto saved_plans = trip_plans;
        auto saved_orders = dest_orders;

        if (!try_random_move(trip_plans, dest_orders))
        {
            trip_plans = saved_plans;
            dest_orders = saved_orders;
            goto COOL;
        }

        {
            vector<double> nd(n_pkg), nds(n_pkg);
            double ns = evaluate_solution(trip_plans, dest_orders, nd, nds);
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

// ============ 主入口 ============

Task2Result Task2::solve()
{
    // 1. 贪心初解
    vector<vector<int>> trip_plans, dest_orders;
    greedy_init(trip_plans, dest_orders);

    // 2. 模拟退火优化
    sa_optimize(trip_plans, dest_orders);

    // 3. 最终评估
    vector<double> deliver_time(n_pkg), dissatisfaction(n_pkg);
    double total = evaluate_solution(trip_plans, dest_orders, deliver_time, dissatisfaction);

    // 4. 构造 Trip 列表
    vector<Trip> trips;
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

        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, depart, pkgs, c, ap);

        trips.push_back(sim);
        cur_time = sim.end_time;
    }

    return { trips, total, deliver_time, dissatisfaction };
}