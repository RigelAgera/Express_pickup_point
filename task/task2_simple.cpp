// T2简化版: 纯贪心 + nearest-first趟内排序, 无SA优化
#include "task2_simple.h"
#include "../common/dijkstra.h"
#include "../common/schedule_utils.h"
#include <algorithm>
#include <numeric>

using std::vector;

Task2Simple::Task2Simple(const input_data& data)
    : pkgs(data.packages), c(data.c),
      ap(all_pairs_dijkstra(data.g)),
      n_pkg((int)pkgs.size())
{
    // T2: deadline=INF
    for (auto& p : pkgs)
        p.deadline = 1e9;
}

vector<int> Task2Simple::unique_dests(const vector<int>& pkg_ids)
{
    vector<int> dests;
    for (int pid : pkg_ids)
        dests.push_back(pkgs[pid].dest);
    std::sort(dests.begin(), dests.end());
    dests.erase(std::unique(dests.begin(), dests.end()), dests.end());
    return dests;
}

double Task2Simple::trip_weight(const vector<int>& pkg_ids)
{
    double w = 0.0;
    for (int pid : pkg_ids)
        w += pkgs[pid].weight;
    return w;
}

vector<int> Task2Simple::nearest_dest_order(const vector<int>& pkg_ids)
{
    vector<int> dests = unique_dests(pkg_ids);
    return nearest_first_order(dests, ap);
}

double Task2Simple::evaluate_solution(
    const vector<vector<int>>& trip_plans,
    const vector<vector<int>>& dest_orders,
    vector<double>& deliver_time,
    vector<double>& dissatisfaction)
{
    deliver_time.assign(n_pkg, -1.0);
    dissatisfaction.assign(n_pkg, 0.0);

    double cur_time = 0.0;
    double total = 0.0;

    for (size_t t = 0; t < trip_plans.size(); ++t)
    {
        const auto& pkg_ids = trip_plans[t];
        const auto& dest_order = dest_orders[t];
        if (pkg_ids.empty()) continue;

        // 出发时间 = max(cur_time, max(S_i))
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

void Task2Simple::greedy_init(
    vector<vector<int>>& trip_plans,
    vector<vector<int>>& dest_orders)
{
    // 按包裹到达时间 Si 升序
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

Task2SimpleResult Task2Simple::solve()
{
    vector<vector<int>> trip_plans, dest_orders;
    greedy_init(trip_plans, dest_orders);

    vector<double> deliver_time(n_pkg);
    vector<double> dissatisfaction(n_pkg);
    double total = evaluate_solution(trip_plans, dest_orders,
                                      deliver_time, dissatisfaction);

    // 构造 Trip 输出
    vector<Trip> result_trips;
    double cur_time = 0.0;
    for (size_t t = 0; t < trip_plans.size(); ++t)
    {
        const auto& pkg_ids = trip_plans[t];
        const auto& dest_order = dest_orders[t];
        if (pkg_ids.empty()) continue;

        double max_S = 0.0;
        for (int pid : pkg_ids)
            if (pkgs[pid].arrive > max_S)
                max_S = pkgs[pid].arrive;
        double depart = std::max(cur_time, max_S);

        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, depart, pkgs, c, ap);
        result_trips.push_back(sim);
        cur_time = sim.end_time;
    }

    return {result_trips, total, deliver_time, dissatisfaction};
}