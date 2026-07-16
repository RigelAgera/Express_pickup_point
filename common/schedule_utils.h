#pragma once
#include "graph.h"
#include "dijkstra.h"
#include "models.h"
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>

// 全源最短路：dist[i][j] = i→j 最短距离，path[i][j] = 途经节点序列
struct AllPairResult
{
    std::vector<std::vector<double>> dist;
    std::vector<std::vector<std::vector<int>>> path;
};

inline AllPairResult all_pairs_dijkstra(const graph& g)
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

// 一趟配送的描述
struct Trip
{
    std::vector<int> pkg_ids;        // 本趟携带的包裹 id
    std::vector<int> route;          // 行驶路线（节点序列，首尾都是 0）
    std::vector<double> segment_len; // 每段路的长度
    double depart_time;              // 出发时刻
    double end_time;                 // 回到驿站的时刻
    std::vector<double> deliver_time;// 每个包裹的送达时刻
};

// 一趟配送的成本（T3 定义）
inline double trip_cost(const Trip& trip, const graph& g,
                        const std::vector<package>& pkgs,
                        const car& c,
                        const AllPairResult& ap)
{
    // 建立包裹 id 到目的地和重量的映射
    double cost = 0.0;
    double cur_weight = c.car_weight;
    for (int pid : trip.pkg_ids)
        cur_weight += pkgs[pid].weight;

    double cur_time = trip.depart_time;
    for (size_t seg = 0; seg + 1 < trip.route.size(); ++seg)
    {
        int from = trip.route[seg];
        int to   = trip.route[seg + 1];
        double seg_len = ap.dist[from][to];

        // 到达 to 时卸下目的地为 to 的包裹
        double weight_before = cur_weight;
        for (int pid : trip.pkg_ids)
        {
            if (pkgs[pid].dest == to)
                cur_weight -= pkgs[pid].weight;
        }

        cost += seg_len * weight_before;
        cur_time += seg_len / c.speed;
    }
    return cost;
}

// 模拟一趟：给定包裹列表和送达顺序（节点序列），计算各送达时刻
// route 形如 [0, dest_a, dest_b, ..., 0]
inline Trip simulate_trip(const std::vector<int>& pkg_ids,
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

    // 记录每个包裹的送达时刻
    std::vector<bool> delivered(pkg_ids.size(), false);

    for (size_t seg = 0; seg + 1 < route.size(); ++seg)
    {
        int from = route[seg];
        int to   = route[seg + 1];
        double seg_len = ap.dist[from][to];
        cur_time += seg_len / c.speed;

        // 在 to 卸下目的地为 to 的包裹
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

// 构造 route：从 0 出发，依次到达 dests 中的节点，再回到 0
// 每段走最短路
inline std::vector<int> build_route(const std::vector<int>& dests,
                                    const AllPairResult& ap)
{
    std::vector<int> route;
    route.push_back(0);
    for (int d : dests)
    {
        const auto& p = ap.path[route.back()][d];
        // 拼接路径，跳过首节点（避免重复）
        for (size_t i = 1; i < p.size(); ++i)
            route.push_back(p[i]);
    }
    // 回到 0
    const auto& p = ap.path[route.back()][0];
    for (size_t i = 1; i < p.size(); ++i)
        route.push_back(p[i]);
    if (route.back() != 0)
        route.push_back(0);
    return route;
}

// 对一组包裹的 dest 排序：最近优先（贪心）
inline std::vector<int> nearest_first_order(const std::vector<int>& dests,
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

// 按重量降序排序（重先送）
inline std::vector<int> heaviest_first_order(const std::vector<int>& pkg_indices,
                                             const std::vector<package>& pkgs)
{
    std::vector<int> order;
    std::vector<std::pair<double, int>> tmp;
    for (int idx : pkg_indices)
        tmp.emplace_back(pkgs[idx].weight, idx);
    std::sort(tmp.rbegin(), tmp.rend()); // 降序
    for (auto& p : tmp)
        order.push_back(p.second);
    return order;
}

inline std::vector<int> lightest_first_order(const std::vector<int>& pkg_indices,
                                             const std::vector<package>& pkgs)
{
    std::vector<int> order;
    std::vector<std::pair<double, int>> tmp;
    for (int idx : pkg_indices)
        tmp.emplace_back(pkgs[idx].weight, idx);
    std::sort(tmp.begin(), tmp.end()); // 升序
    for (auto& p : tmp)
        order.push_back(p.second);
    return order;
}