// T2: 最小化不满意度之和
// 约束：不考虑超时（Ti=INF），包裹在其 Si 之后才可配送，容量 w_lim 约束
// 编译: g++ -std=c++17 -O2 task/task2.cpp common/input_reader.cpp -o task2.exe

#include "../common/input_reader.h"
#include "../common/dijkstra.h"
#include "../common/min_heap.h"
#include "../common/circular_queue.h"
#include "../common/schedule_utils.h"
#include <iostream>
#include <iomanip>
#include <numeric>

int main(int argc, char* argv[])
{
    std::string folder = (argc >= 2) ? argv[1] : "测试数据/示例";

    auto data = read_input(folder);
    auto& pkgs = data.packages;
    auto& c = data.c;
    int K = (int)pkgs.size();

    auto ap = all_pairs_dijkstra(data.g);

    // 按到站时间排序的包裹索引
    std::vector<int> idx_by_arrive(K);
    std::iota(idx_by_arrive.begin(), idx_by_arrive.end(), 0);
    std::sort(idx_by_arrive.begin(), idx_by_arrive.end(),
              [&](int a, int b) { return pkgs[a].arrive < pkgs[b].arrive; });

    // 贪心策略：按到站时间顺序处理，每次尽可能多装当前可派的包裹
    // 装车时按"最近优先"排序送达顺序，重先装以省成本
    std::vector<bool> assigned(K, false);
    std::vector<Trip> trips;
    double total_unhappiness = 0.0;

    int pkg_ptr = 0; // 指向下一个可能变为可送的包裹

    while (true)
    {
        // 找到最早可以出发的时间
        double earliest_ready = std::numeric_limits<double>::infinity();
        bool any_available = false;
        for (int i = 0; i < K; ++i)
        {
            if (!assigned[i] && pkgs[i].arrive <= earliest_ready)
            {
                earliest_ready = pkgs[i].arrive;
                any_available = true;
            }
        }
        if (!any_available) break;

        // 收集当前可派送的包裹（arrive <= earliest_ready）
        std::vector<int> candidates;
        for (int i = 0; i < K; ++i)
        {
            if (!assigned[i] && pkgs[i].arrive <= earliest_ready + 1e-9)
                candidates.push_back(i);
        }

        // 按重量排序装车：容量限制下尽量多装
        // 策略：按重量降序装入尽可能多的包裹
        auto sorted = heaviest_first_order(candidates, pkgs);

        std::vector<int> trip_pkg_ids;
        double loaded = 0.0;
        for (int idx : sorted)
        {
            if (loaded + pkgs[idx].weight <= c.capacity + 1e-9)
            {
                trip_pkg_ids.push_back(idx);
                loaded += pkgs[idx].weight;
                assigned[idx] = true;
            }
        }

        if (trip_pkg_ids.empty())
        {
            // 没有能装车的包裹（单件超过容量？不太可能但容错）
            break;
        }

        // 决定出发时间：本趟最早出发时间 = max( earliest_ready, 0 )
        // 实际上就是 earliest_ready
        // 如果之前的车还没回来，需要等
        double prev_return = trips.empty() ? 0.0 : trips.back().end_time;
        double depart = std::max(earliest_ready, prev_return);

        // 排序送达顺序：最近优先
        std::vector<int> dests;
        for (int idx : trip_pkg_ids)
            dests.push_back(pkgs[idx].dest);

        // 按最近优先重新排列 dests（对 trip_pkg_ids 同步排序）
        std::vector<int> perm(trip_pkg_ids.size());
        std::iota(perm.begin(), perm.end(), 0);
        std::vector<int> cur_dest = dests;
        std::vector<int> sorted_dests;
        std::vector<int> sorted_indices;
        int cur_node = 0;
        std::vector<bool> used(perm.size(), false);
        for (size_t k = 0; k < perm.size(); ++k)
        {
            double best = std::numeric_limits<double>::infinity();
            int best_j = -1;
            for (size_t j = 0; j < perm.size(); ++j)
            {
                if (used[j]) continue;
                double d = ap.dist[cur_node][dests[j]];
                if (d < best) { best = d; best_j = (int)j; }
            }
            cur_node = dests[best_j];
            sorted_dests.push_back(dests[best_j]);
            sorted_indices.push_back(best_j);
            used[best_j] = true;
        }

        // 重建 trip_pkg_ids 按 sorted_indices 顺序
        std::vector<int> sorted_pkg_ids;
        for (int j : sorted_indices)
            sorted_pkg_ids.push_back(trip_pkg_ids[j]);

        // 构建路线
        auto route = build_route(sorted_dests, ap);
        auto trip = simulate_trip(sorted_pkg_ids, route, depart, pkgs, c, ap);

        // 计算不满意度
        for (size_t i = 0; i < trip.pkg_ids.size(); ++i)
        {
            int pid = trip.pkg_ids[i];
            double di = trip.deliver_time[i];
            double si = pkgs[pid].arrive;
            total_unhappiness += (di - si);
        }

        trips.push_back(trip);
    }

    // === 输出 ===
    std::cout << "=== T2 最小化不满意度之和 ===\n";
    std::cout << "数据: " << folder << "\n\n";
    std::cout << "共 " << trips.size() << " 趟配送:\n\n";

    for (size_t t = 0; t < trips.size(); ++t)
    {
        auto& tr = trips[t];
        std::cout << "第 " << (t + 1) << " 趟:\n";
        std::cout << "  出发时刻: " << tr.depart_time << "\n";
        std::cout << "  路线: ";
        for (size_t i = 0; i < tr.route.size(); ++i)
        {
            if (i > 0) std::cout << " -> ";
            std::cout << tr.route[i];
        }
        std::cout << "\n";
        std::cout << "  包裹详情:\n";
        double trip_unhap = 0.0;
        for (size_t i = 0; i < tr.pkg_ids.size(); ++i)
        {
            int pid = tr.pkg_ids[i];
            double di = tr.deliver_time[i];
            double si = pkgs[pid].arrive;
            trip_unhap += (di - si);
            std::cout << "    包裹 " << pid << ": 目的地=" << pkgs[pid].dest
                      << "  重量=" << pkgs[pid].weight
                      << "  到站=" << si
                      << "  送达=" << di
                      << "  不满意度=" << (di - si) << "\n";
        }
        std::cout << "  本趟不满意度小计: " << trip_unhap << "\n";
        std::cout << "  回到驿站时刻: " << tr.end_time << "\n\n";
    }

    std::cout << "================================\n";
    std::cout << "不满意度之和: " << total_unhappiness << "\n";

    return 0;
}