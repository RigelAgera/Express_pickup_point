#include "schedule_utils.h"

AllPairResult all_pairs_dijkstra(const graph& g)
{
    int n = g.get_n();
    AllPairResult r;
    r.dist.assign(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
    r.path.assign(n, std::vector<std::vector<int>>(n));

    for (int src = 0; src < n; ++src)
    {
        auto res = dijkstra(g, src);
        r.dist[src] = res.dist;
        r.dist[src][src] = 0.0;
        for (int dst = 0; dst < n; ++dst)
            r.path[src][dst] = get_path(res.prev, src, dst);
    }
    return r;
}

double trip_cost(const Trip& trip, const graph& g,
                 const std::vector<package>& pkgs,
                 const car& c,
                 const AllPairResult& ap)
{
    double cost = 0.0;
    double cur_weight = c.car_weight;
    for (int pid : trip.pkg_ids)
        cur_weight += pkgs[pid].weight;

    for (size_t seg = 0; seg + 1 < trip.route.size(); ++seg)
    {
        int from = trip.route[seg];
        int to   = trip.route[seg + 1];
        double seg_len = ap.dist[from][to];

        double weight_before = cur_weight;
        for (int pid : trip.pkg_ids)
        {
            if (pkgs[pid].dest == to)
                cur_weight -= pkgs[pid].weight;
        }

        cost += seg_len * weight_before;
    }
    return cost;
}

Trip simulate_trip(const std::vector<int>& pkg_ids,
                   const std::vector<int>& route,
                   double depart_time,
                   const std::vector<package>& pkgs,
                   const car& c,
                   const AllPairResult& ap)
{
    Trip t;
    t.pkg_ids = pkg_ids;
    t.route = route;
    t.depart_time = depart_time;
    t.deliver_time.resize(pkg_ids.size(), 0.0);

    double cur_time = depart_time;

    std::vector<bool> delivered(pkg_ids.size(), false);

    for (size_t seg = 0; seg + 1 < route.size(); ++seg)
    {
        int from = route[seg];
        int to   = route[seg + 1];
        double seg_len = ap.dist[from][to];
        cur_time += seg_len / c.speed;

        for (size_t i = 0; i < pkg_ids.size(); ++i)
        {
            if (!delivered[i] && pkgs[pkg_ids[i]].dest == to)
            {
                t.deliver_time[i] = cur_time;
                delivered[i] = true;
            }
        }
    }

    t.end_time = cur_time;
    return t;
}

std::vector<int> build_route(const std::vector<int>& dests,
                             const AllPairResult& ap)
{
    std::vector<int> route;
    route.push_back(0);
    for (int d : dests)
    {
        const auto& p = ap.path[route.back()][d];
        for (size_t i = 1; i < p.size(); ++i)
            route.push_back(p[i]);
    }
    const auto& p = ap.path[route.back()][0];
    for (size_t i = 1; i < p.size(); ++i)
        route.push_back(p[i]);
    if (route.back() != 0)
        route.push_back(0);
    return route;
}

std::vector<int> nearest_first_order(const std::vector<int>& dests,
                                     const AllPairResult& ap)
{
    std::vector<int> order = dests;
    std::vector<int> result;
    std::vector<bool> used(order.size(), false);
    int cur = 0;
    for (size_t k = 0; k < order.size(); ++k)
    {
        double best = std::numeric_limits<double>::infinity();
        int best_idx = -1;
        for (size_t i = 0; i < order.size(); ++i)
        {
            if (used[i]) continue;
            double d = ap.dist[cur][order[i]];
            if (d < best) { best = d; best_idx = (int)i; }
        }
        cur = order[best_idx];
        result.push_back(cur);
        used[best_idx] = true;
    }
    return result;
}

std::vector<int> heaviest_first_order(const std::vector<int>& pkg_indices,
                                      const std::vector<package>& pkgs)
{
    std::vector<int> order;
    std::vector<std::pair<double, int>> tmp;
    for (int idx : pkg_indices)
        tmp.emplace_back(pkgs[idx].weight, idx);
    std::sort(tmp.rbegin(), tmp.rend());
    for (auto& p : tmp)
        order.push_back(p.second);
    return order;
}

std::vector<int> lightest_first_order(const std::vector<int>& pkg_indices,
                                      const std::vector<package>& pkgs)
{
    std::vector<int> order;
    std::vector<std::pair<double, int>> tmp;
    for (int idx : pkg_indices)
        tmp.emplace_back(pkgs[idx].weight, idx);
    std::sort(tmp.begin(), tmp.end());
    for (auto& p : tmp)
        order.push_back(p.second);
    return order;
}