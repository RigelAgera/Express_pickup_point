// T5简化版: K-means分区 + EDD贪心分趟 + nearest-first趟内排序, 无SA/ALNS
#include "task5_simple.h"
#include "../common/dijkstra.h"
#include "../common/schedule_utils.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

using std::vector;

Task5Simple::Task5Simple(const input_data& data)
    : g(data.g), pkgs(data.packages), c1(data.c), c2(data.c),
      ap(all_pairs_dijkstra(data.g)),
      n_pkg((int)data.packages.size()),
      rng(std::random_device{}())
{
}

// ============ 辅助函数 ============

vector<int> Task5Simple::unique_dests(const vector<int>& pkg_ids)
{
    vector<int> dests;
    for (int pid : pkg_ids)
        dests.push_back(pkgs[pid].dest);
    std::sort(dests.begin(), dests.end());
    dests.erase(std::unique(dests.begin(), dests.end()), dests.end());
    return dests;
}

double Task5Simple::trip_weight(const vector<int>& pkg_ids)
{
    double w = 0.0;
    for (int pid : pkg_ids)
        w += pkgs[pid].weight;
    return w;
}

vector<int> Task5Simple::nearest_dest_order(const vector<int>& pkg_ids)
{
    vector<int> dests = unique_dests(pkg_ids);
    return nearest_first_order(dests, ap);
}

// ============ K-means 分区 (k=2, 按目的地坐标) ============

std::pair<vector<int>, vector<int>> Task5Simple::kmeans_partition()
{
    if (n_pkg == 0) return {{}, {}};
    if (n_pkg == 1) return {{0}, {}};

    // 提取每个包裹目的地坐标
    vector<std::pair<double, double>> coords(n_pkg);
    for (int i = 0; i < n_pkg; ++i)
    {
        int dest = pkgs[i].dest;
        coords[i] = {g.get_nodes()[dest].x, g.get_nodes()[dest].y};
    }

    // 随机选两个初始质心
    std::uniform_int_distribution<int> uid(0, n_pkg - 1);
    int idx0 = uid(rng);
    int idx1;
    do { idx1 = uid(rng); } while (idx1 == idx0);
    double cx[2], cy[2];
    cx[0] = g.get_nodes()[pkgs[idx0].dest].x;
    cy[0] = g.get_nodes()[pkgs[idx0].dest].y;
    cx[1] = g.get_nodes()[pkgs[idx1].dest].x;
    cy[1] = g.get_nodes()[pkgs[idx1].dest].y;

    vector<int> labels(n_pkg, 0);
    const int MAX_ITER = 100;

    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        bool changed = false;
        for (int i = 0; i < n_pkg; ++i)
        {
            double d0 = (coords[i].first - cx[0]) * (coords[i].first - cx[0])
                      + (coords[i].second - cy[0]) * (coords[i].second - cy[0]);
            double d1 = (coords[i].first - cx[1]) * (coords[i].first - cx[1])
                      + (coords[i].second - cy[1]) * (coords[i].second - cy[1]);
            int new_label = (d0 <= d1) ? 0 : 1;
            if (new_label != labels[i]) { labels[i] = new_label; changed = true; }
        }
        if (!changed) break;

        cx[0] = cy[0] = cx[1] = cy[1] = 0.0;
        int cnt0 = 0, cnt1 = 0;
        for (int i = 0; i < n_pkg; ++i)
        {
            if (labels[i] == 0) { cx[0] += coords[i].first; cy[0] += coords[i].second; ++cnt0; }
            else               { cx[1] += coords[i].first; cy[1] += coords[i].second; ++cnt1; }
        }
        if (cnt0 > 0) { cx[0] /= cnt0; cy[0] /= cnt0; }
        if (cnt1 > 0) { cx[1] /= cnt1; cy[1] /= cnt1; }
    }

    vector<int> cluster0, cluster1;
    for (int i = 0; i < n_pkg; ++i)
    {
        if (labels[i] == 0) cluster0.push_back(i);
        else                cluster1.push_back(i);
    }

    // 空簇处理: 按EDD交替分配
    if (cluster0.empty() || cluster1.empty())
    {
        vector<int> all;
        for (int pid : cluster0) all.push_back(pid);
        for (int pid : cluster1) all.push_back(pid);
        std::sort(all.begin(), all.end(),
                  [&](int a, int b) { return pkgs[a].deadline < pkgs[b].deadline; });
        cluster0.clear();
        cluster1.clear();
        double w0 = 0.0, w1 = 0.0;
        for (int pid : all)
        {
            if (w0 <= w1) { cluster0.push_back(pid); w0 += pkgs[pid].weight; }
            else          { cluster1.push_back(pid); w1 += pkgs[pid].weight; }
        }
    }

    return {cluster0, cluster1};
}

// ============ 单车贪心初解 (EDD + nearest-first) ============

void Task5Simple::greedy_init_one(const vector<int>& pkg_indices, CarPlan& plan, const car& c)
{
    if (pkg_indices.empty()) return;

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
        if (cur_weight + w > c.capacity + 1e-9 && !cur_trip.empty())
        {
            plan.trip_plans.push_back(cur_trip);
            plan.dest_orders.push_back(nearest_dest_order(cur_trip));
            cur_trip.clear();
            cur_weight = 0.0;
        }
        cur_trip.push_back(pid);
        cur_weight += w;
    }

    if (!cur_trip.empty())
    {
        plan.trip_plans.push_back(cur_trip);
        plan.dest_orders.push_back(nearest_dest_order(cur_trip));
    }
}

// ============ 单车评估: (成本, 超时数) ============

std::pair<double, int> Task5Simple::evaluate_car(
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

        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, cur_time, pkgs, c, ap);

        cur_time = sim.end_time;
        total_cost += trip_cost(sim, g, pkgs, c, ap);

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

// ============ solve (主入口) ============

Task5SimpleResult Task5Simple::solve()
{
    // 1. K-means 分区
    auto [cluster0, cluster1] = kmeans_partition();

    // 2. 每车独立贪心
    CarPlan plan0, plan1;
    greedy_init_one(cluster0, plan0, c1);
    greedy_init_one(cluster1, plan1, c2);

    // 3. 评估
    vector<double> deliv0, deliv1;
    auto [cost0, ot0] = evaluate_car(plan0.trip_plans, plan0.dest_orders, deliv0, c1);
    auto [cost1, ot1] = evaluate_car(plan1.trip_plans, plan1.dest_orders, deliv1, c2);
    double total_cost = cost0 + cost1;
    int overtime = ot0 + ot1;

    // 4. 构造输出 trips1
    vector<Trip> trips1, trips2;
    double cur_time = 0.0;
    for (size_t t = 0; t < plan0.trip_plans.size(); ++t)
    {
        const auto& pkg_ids = plan0.trip_plans[t];
        const auto& dest_order = plan0.dest_orders[t];
        if (pkg_ids.empty()) continue;
        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, cur_time, pkgs, c1, ap);
        trips1.push_back(sim);
        cur_time = sim.end_time;
    }

    cur_time = 0.0;
    for (size_t t = 0; t < plan1.trip_plans.size(); ++t)
    {
        const auto& pkg_ids = plan1.trip_plans[t];
        const auto& dest_order = plan1.dest_orders[t];
        if (pkg_ids.empty()) continue;
        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, cur_time, pkgs, c2, ap);
        trips2.push_back(sim);
        cur_time = sim.end_time;
    }

    return {trips1, trips2, total_cost, overtime};
}