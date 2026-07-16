// T3: 带容量的运送成本
// 约束：Si=0（所有包裹随时可派），容量 w_lim，每个包裹有 Ti（最晚送达时间）
// 成本 = Σ 段长 × (w_car + 该段行驶时车上包裹总重)
// 编译: g++ -std=c++17 -O2 task/task3.cpp common/input_reader.cpp -o task3.exe

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

    // T3 策略：Si=0，所以出发时间为 0
    // 贪心：先按重量降序装车（重先送以减少后期成本），一趟内按最近优先排序送达
    std::vector<bool> assigned(K, false);
    std::vector<Trip> trips;
    int overtime_count = 0;
    double total_cost = 0.0;

    // 按重量降序排列的包裹索引（未分配的）
    auto get_candidates = [&]() -> std::vector<int> {
        std::vector<int> cand;
        for (int i = 0; i < K; ++i)
            if (!assigned[i])
                cand.push_back(i);
        // 按重量降序
        std::sort(cand.begin(), cand.end(),
                  [&](int a, int b) { return pkgs[a].weight > pkgs[b].weight; });
        return cand;
    };

    double prev_return = 0.0;

    while (true)
    {
        auto candidates = get_candidates();
        if (candidates.empty()) break;

        // 容量约束：挑最多的包裹装车（先挑重的）
        std::vector<int> trip_pkg_ids;
        double loaded = 0.0;
        for (int idx : candidates)
        {
            if (loaded + pkgs[idx].weight <= c.capacity + 1e-9)
            {
                trip_pkg_ids.push_back(idx);
                loaded += pkgs[idx].weight;
                assigned[idx] = true;
            }
        }

        if (trip_pkg_ids.empty()) break;

        double depart = prev_return; // Si=0，可直接出发

        // 排序送达顺序：最近优先（贪心）
        std::vector<int> dests;
        for (int idx : trip_pkg_ids)
            dests.push_back(pkgs[idx].dest);

        // 最近优先重排
        std::vector<int> sorted_dests;
        std::vector<int> sorted_indices;
        std::vector<bool> used(trip_pkg_ids.size(), false);
        int cur_node = 0;
        for (size_t k = 0; k < trip_pkg_ids.size(); ++k)
        {
            double best = std::numeric_limits<double>::infinity();
            int best_j = -1;
            for (size_t j = 0; j < trip_pkg_ids.size(); ++j)
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

        std::vector<int> sorted_pkg_ids;
        for (int j : sorted_indices)
            sorted_pkg_ids.push_back(trip_pkg_ids[j]);

        auto route = build_route(sorted_dests, ap);
        auto trip = simulate_trip(sorted_pkg_ids, route, depart, pkgs, c, ap);

        // 计算成本（逐段计算）
        double cost_trip = trip_cost(trip, data.g, pkgs, c, ap);
        total_cost += cost_trip;

        // 检查超时
        for (size_t i = 0; i < trip.pkg_ids.size(); ++i)
        {
            int pid = trip.pkg_ids[i];
            if (trip.deliver_time[i] > pkgs[pid].deadline + 1e-9)
                ++overtime_count;
        }

        trips.push_back(trip);
        prev_return = trip.end_time;
    }

    // === 输出 ===
    std::cout << "=== T3 带容量的运送成本 ===\n";
    std::cout << "数据: " << folder << "\n";
    std::cout << "小车: 速度=" << c.speed << "  自重=" << c.car_weight
              << "  容量=" << c.capacity << "\n\n";
    std::cout << "共 " << trips.size() << " 趟配送:\n\n";

    for (size_t t_idx = 0; t_idx < trips.size(); ++t_idx)
    {
        auto& tr = trips[t_idx];
        std::cout << "第 " << (t_idx + 1) << " 趟:\n";
        std::cout << "  出发时刻: " << tr.depart_time << "\n";
        std::cout << "  路线: ";
        for (size_t i = 0; i < tr.route.size(); ++i)
        {
            if (i > 0) std::cout << " -> ";
            std::cout << tr.route[i];
        }
        std::cout << "\n";

        // 逐段详列成本
        double cur_w = c.car_weight;
        for (int pid : tr.pkg_ids) cur_w += pkgs[pid].weight;
        double seg_cost_sum = 0.0;

        std::cout << "  段成本明细:\n";
        for (size_t seg = 0; seg + 1 < tr.route.size(); ++seg)
        {
            int from = tr.route[seg];
            int to   = tr.route[seg + 1];
            double seg_len = ap.dist[from][to];

            double w_before = cur_w;
            for (int pid : tr.pkg_ids)
                if (pkgs[pid].dest == to)
                    cur_w -= pkgs[pid].weight;

            double seg_cost = seg_len * w_before;
            seg_cost_sum += seg_cost;

            std::cout << "    " << from << "->" << to
                      << "  段长=" << seg_len
                      << "  车载重=" << w_before
                      << "  段成本=" << seg_cost << "\n";
        }
        std::cout << "  本趟成本小计: " << seg_cost_sum << "\n";

        std::cout << "  包裹详情:\n";
        for (size_t i = 0; i < tr.pkg_ids.size(); ++i)
        {
            int pid = tr.pkg_ids[i];
            bool overtime = tr.deliver_time[i] > pkgs[pid].deadline + 1e-9;
            std::cout << "    包裹 " << pid
                      << ": 目的地=" << pkgs[pid].dest
                      << "  重量=" << pkgs[pid].weight
                      << "  送达=" << tr.deliver_time[i]
                      << "  截止=" << pkgs[pid].deadline
                      << (overtime ? "  [超时]" : "") << "\n";
        }
        std::cout << "  回到驿站时刻: " << tr.end_time << "\n\n";
    }

    std::cout << "================================\n";
    std::cout << "运送总成本: " << total_cost << "\n";
    std::cout << "超时包裹数: " << overtime_count << " / " << K << "\n";

    return 0;
}