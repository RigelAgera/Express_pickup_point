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

AllPairResult all_pairs_dijkstra(const graph& g);

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
double trip_cost(const Trip& trip, const graph& g,
                 const std::vector<package>& pkgs,
                 const car& c,
                 const AllPairResult& ap);

// 模拟一趟：给定包裹列表和送达顺序（节点序列），计算各送达时刻
// route 形如 [0, dest_a, dest_b, ..., 0]
Trip simulate_trip(const std::vector<int>& pkg_ids,
                   const std::vector<int>& route,
                   double depart_time,
                   const std::vector<package>& pkgs,
                   const car& c,
                   const AllPairResult& ap);

// 构造 route：从 0 出发，依次到达 dests 中的节点，再回到 0
// 每段走最短路
std::vector<int> build_route(const std::vector<int>& dests,
                             const AllPairResult& ap);

// 对一组包裹的 dest 排序：最近优先（贪心）
std::vector<int> nearest_first_order(const std::vector<int>& dests,
                                     const AllPairResult& ap);

// 按重量降序排序（重先送）
std::vector<int> heaviest_first_order(const std::vector<int>& pkg_indices,
                                      const std::vector<package>& pkgs);

// 按重量升序排序（轻先送）
std::vector<int> lightest_first_order(const std::vector<int>& pkg_indices,
                                      const std::vector<package>& pkgs);