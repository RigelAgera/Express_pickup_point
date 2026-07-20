// T3简化版: 纯EDD贪心 + nearest-first趟内排序, 无SA优化
#include "task3_simple.h"
#include "../common/dijkstra.h"
#include "../common/schedule_utils.h"
#include <algorithm>
#include <numeric>
#include <iostream>

using std::vector;

Task3Simple::Task3Simple(const input_data& data)
    : g(data.g), pkgs(data.packages), c(data.c),
      ap(all_pairs_dijkstra(data.g)),
      n_pkg((int)data.packages.size())
{
    // T3要求 Si=0
    for (auto& p : pkgs)
        p.arrive = 0.0;
}

vector<int> Task3Simple::unique_dests(const vector<int>& pkg_ids)
{
    vector<int> dests;
    for (int pid : pkg_ids)
        dests.push_back(pkgs[pid].dest);
    std::sort(dests.begin(), dests.end());
    dests.erase(std::unique(dests.begin(), dests.end()), dests.end());
    return dests;
}

double Task3Simple::trip_weight(const vector<int>& pkg_ids)
{
    double w = 0.0;
    for (int pid : pkg_ids)
        w += pkgs[pid].weight;
    return w;
}

vector<int> Task3Simple::nearest_dest_order(const vector<int>& pkg_ids)
{
    vector<int> dests = unique_dests(pkg_ids);
    return nearest_first_order(dests, ap);
}

std::pair<int, double> Task3Simple::evaluate_solution(
    const vector<vector<int>>& trip_plans,
    const vector<vector<int>>& dest_orders,
    vector<double>& deliver_time)
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
            double d_i = sim.deliver_time[i];
            deliver_time[pid] = d_i;
            if (d_i > pkgs[pid].deadline + 1e-9)
                ++overtime;
        }
    }

    return {overtime, total_cost};
}

void Task3Simple::greedy_init(
    vector<vector<int>>& trip_plans,
    vector<vector<int>>& dest_orders)
{
    // EDD: 按deadline升序
    vector<int> order(n_pkg);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return pkgs[a].deadline < pkgs[b].deadline; });

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

Task3SimpleResult Task3Simple::solve()
{
    vector<vector<int>> trip_plans, dest_orders;
    greedy_init(trip_plans, dest_orders);

    vector<double> deliver_time(n_pkg);
    auto [overtime, total_cost] = evaluate_solution(
        trip_plans, dest_orders, deliver_time);

    // 构造 Trip 输出
    vector<Trip> result_trips;
    double cur_time = 0.0;
    for (size_t t = 0; t < trip_plans.size(); ++t)
    {
        const auto& pkg_ids = trip_plans[t];
        const auto& dest_order = dest_orders[t];
        if (pkg_ids.empty()) continue;

        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, cur_time, pkgs, c, ap);
        result_trips.push_back(sim);
        cur_time = sim.end_time;
    }

    return {result_trips, total_cost, overtime};
}