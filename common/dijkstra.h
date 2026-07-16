#pragma once
#include "graph.h"
#include "min_heap.h"
#include <vector>
#include <limits>

// Dijkstra 结果
struct DijkResult
{
    std::vector<double> dist;       // dist[v] = 从源点到 v 的最短距离
    std::vector<int> prev;          // prev[v] = 最短路径上 v 的前驱节点，源点或不可达为 -1，可通过前驱回溯路径，满足T1要求
};

// 单源最短路：dijkstra(g, src) 返回 src 到所有节点的距离和前驱
// 时间复杂度 O((E + V) log V)

// 编译器会自行决定是否真正内联展开，仅用于满足ODR
inline DijkResult dijkstra(const graph& g, int src)
{
    int n = g.get_n();  // 获取节点数量
    const auto& adj = g.get_adj();  // 获取邻接表，是vector<vector<adj_edge>>类型

    std::vector<double> dist(n, std::numeric_limits<double>::infinity());   // 初始值为double类型的正无穷大
    std::vector<int> prev(n, -1);

    MinHeap heap(n);
    dist[src] = 0.0;
    heap.push(src, 0.0);

    while (!heap.empty())
    {
        auto top = heap.top();
        heap.pop();
        int u = top.id;
        if (top.dist != dist[u])
            continue; // 过时记录，跳过

        for (const auto& e : adj[u])
        {
            int v = e.v;
            double nd = dist[u] + e.w;
            if (nd < dist[v] - 1e-12)
            {
                dist[v] = nd;
                prev[v] = u;
                if (heap.contains(v))
                    heap.decrease_key(v, nd);
                else
                    heap.push(v, nd);
            }
        }
    }

    return { dist, prev };
}

// 从 prev 数组重建 src→dst 的完整路径（含两端），不可达则返回空
inline std::vector<int> get_path(const std::vector<int>& prev, int src, int dst)
{
    std::vector<int> path;
    if (prev[dst] == -1 && src != dst)
        return path; // 不可达

    for (int cur = dst; cur != -1; cur = prev[cur])
        path.push_back(cur);

    std::reverse(path.begin(), path.end());
    if (path.front() != src)
        return {};
    return path;
}