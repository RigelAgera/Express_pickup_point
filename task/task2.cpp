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
    : pkgs(data.packages),                     // 所有包裹（副本）
      c(data.c),                               // 车辆信息（容量等）
      ap(all_pairs_dijkstra(data.g)),           // 预计算所有节点对的最短路径（距离+下一跳）
      n_pkg((int)pkgs.size()),                 // 包裹总数
      rng((unsigned)std::chrono::steady_clock::now().time_since_epoch().count())  // 随机数生成器（用当前时间播种）
{
    // T2 约束：所有 deadline = INF（T2 不考虑超时）
    for (auto& p : pkgs)
        p.deadline = 1e9;
}

// ============ 辅助 ============

// 返回 pkg_ids 中所有包裹的目的地（去重排序）
vector<int> Task2::unique_dests(const vector<int>& pkg_ids)
{
    vector<int> dests;
    for (int pid : pkg_ids)
        dests.push_back(pkgs[pid].dest);
    std::sort(dests.begin(), dests.end());
    dests.erase(std::unique(dests.begin(), dests.end()), dests.end());
    return dests;
}

// 计算给定包裹集合的总重量
double Task2::trip_weight(const vector<int>& pkg_ids)
{
    double w = 0.0;
    for (int pid : pkg_ids)
        w += pkgs[pid].weight;
    return w;
}

// ============ 核心 ============

// 为给定包裹列表生成"最近邻"目的地访问顺序
vector<int> Task2::nearest_dest_order(const vector<int>& pkg_ids)
{
    vector<int> dests = unique_dests(pkg_ids);
    return nearest_first_order(dests, ap);
}

// 评估完整解（所有趟次）的总不满意度
// trip_plans: 每趟携带的包裹 id 列表
// dest_orders: 每趟的目的地访问顺序
double Task2::evaluate_solution(
    const vector<vector<int>>& trip_plans,
    const vector<vector<int>>& dest_orders,
    vector<double>& deliver_time,      // 输出：每个包裹的送达时间
    vector<double>& dissatisfaction)   // 输出：每个包裹的不满意度 = 送达时间 - 到达时间
{
    deliver_time.assign(n_pkg, -1.0);
    dissatisfaction.assign(n_pkg, 0.0);

    double cur_time = 0.0;  // 车辆当前可用的时间（从仓库出发的最早时间）
    double total = 0.0;     // 总不满意度累加

    for (size_t t_idx = 0; t_idx < trip_plans.size(); ++t_idx)
    {
        const auto& pkg_ids = trip_plans[t_idx];     // 当前趟的包裹 id 列表
        const auto& dest_order = dest_orders[t_idx]; // 当前趟的目的地顺序
        if (pkg_ids.empty()) continue;

        // max_S: 该趟所有包裹中最晚的到达时间（即所有包裹都到齐的时刻，最早可出发时间）
        double max_S = 0.0;
        for (int pid : pkg_ids)
            if (pkgs[pid].arrive > max_S)
                max_S = pkgs[pid].arrive;
        double depart = std::max(cur_time, max_S);  // 实际出发时间

        vector<int> route = build_route(dest_order, ap);  // 根据目的地顺序生成完整行驶路线
        Trip sim = simulate_trip(pkg_ids, route, depart, pkgs, c, ap);  // 模拟该趟行程

        for (size_t i = 0; i < pkg_ids.size(); ++i)
        {
            int pid = pkg_ids[i];
            deliver_time[pid] = sim.deliver_time[i];                        // 包裹送达时间
            dissatisfaction[pid] = sim.deliver_time[i] - pkgs[pid].arrive;  // 不满意度 = 送达 - 到达
            total += dissatisfaction[pid];
        }

        cur_time = sim.end_time;  // 车辆完成本趟后回到仓库的时间
    }
    return total;
}

// ============ 贪心初解 ============

// 贪心构造初始解：按包裹到达时间排序，依次装箱，装满即出发
void Task2::greedy_init(vector<vector<int>>& trip_plans,
                         vector<vector<int>>& dest_orders)
{
    // order: 按到达时间升序排列的包裹 id 列表
    vector<int> order(n_pkg);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return pkgs[a].arrive < pkgs[b].arrive; });

    trip_plans.clear();
    dest_orders.clear();

    vector<int> cur_trip;     // 当前趟的包裹 id 列表
    double cur_weight = 0.0;  // 当前趟已装载总重量

    for (int pid : order)
    {
        double w = pkgs[pid].weight;
        // 若加入当前包裹会超重，则封当前趟，开新趟
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

    // 处理最后一趟（可能不满载）
    if (!cur_trip.empty())
    {
        trip_plans.push_back(cur_trip);
        dest_orders.push_back(nearest_dest_order(cur_trip));
    }
}

// ============ 邻域移动 ============

// 尝试一次随机扰动，返回是否成功（不评估好坏，只做结构修改）
// 三种操作：
//   op=0: 将一个包裹从一趟移到另一趟
//   op=1: 随机交换某趟的两个目的地顺序
//   op=2: 随机反转某趟的一段目的地子序列
bool Task2::try_random_move(vector<vector<int>>& trip_plans,
                             vector<vector<int>>& dest_orders)
{
    int n_trip_cur = (int)trip_plans.size();  // 当前趟数
    int op = std::uniform_int_distribution<int>(0, 2)(rng);  // 随机选择操作类型

    auto saved_plans = trip_plans;    // 备份当前趟分配
    auto saved_orders = dest_orders;  // 备份当前目的地顺序

    if (op == 0 && n_trip_cur >= 2)
    {
        // 操作 0：随机选一个包裹，从 src 趟移到 dst 趟（不超重为前提）
        int src = std::uniform_int_distribution<int>(0, n_trip_cur - 1)(rng);
        if (trip_plans[src].size() <= 1) return false;  // 源趟至少留一个包裹
        int pos = std::uniform_int_distribution<int>(0, (int)trip_plans[src].size() - 1)(rng);
        int pid = trip_plans[src][pos];  // 被移动的包裹 id
        double pw = pkgs[pid].weight;
        int dst;
        do { dst = std::uniform_int_distribution<int>(0, n_trip_cur - 1)(rng); } while (dst == src);
        if (trip_weight(trip_plans[dst]) + pw > c.capacity + 1e-9) return false;  // 目标趟超重则放弃
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
        // 操作 1：随机选一趟，交换其两个目的地的顺序
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
        // 操作 2：随机选一趟，反转其某段目的地子序列（2-opt 变体）
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

// 对贪心初解进行模拟退火优化
void Task2::sa_optimize(vector<vector<int>>& trip_plans,
                         vector<vector<int>>& dest_orders)
{
    vector<double> dummy_deliv, dummy_diss;
    double cur_score = evaluate_solution(trip_plans, dest_orders, dummy_deliv, dummy_diss);

    // ---- 自适应计算初始温度 T_init ----
    // 采样若干次随机扰动，收集不满意度增量（恶化量），取中位数反推初始温度
    vector<double> deltas;  // 收集的正增量样本（坏解的目标函数上升量）
    for (int k = 0; k < 100; ++k)
    {
        auto p2 = trip_plans, o2 = dest_orders;
        if (!try_random_move(p2, o2)) continue;
        double ns = evaluate_solution(p2, o2, dummy_deliv, dummy_diss);
        double d = ns - cur_score;  // 目标函数增量（>0 表示变差）
        if (d > 0) deltas.push_back(d);
    }
    double T = 1.0;  // 初始温度
    if (!deltas.empty())
    {
        std::sort(deltas.begin(), deltas.end());
        double delta_med = deltas[deltas.size() / 2];  // 增量中位数
        // 用接受概率 p = exp(-delta_med / T_init) 反推，设期望接受率约为 0.8
        // exp(-x) = 0.8 ⇒ x ≈ 0.223
        T = delta_med / 0.223;
        if (T < 0.1) T = 0.1;  // 温度下界保护
    }

    // ---- Lundy-Mees 降温策略 ----
    // T_{k+1} = T_k / (1 + beta * T_k)
    const int    MAX_ITER = 100000;  // 最大迭代次数
    const double T_MIN    = 1e-3;    // 最低温度（低于此温度停止）
    double beta = (T - T_MIN) / (MAX_ITER * T * T_MIN);  // 降温系数

    // ---- pkg_trip: 包裹到其所在趟次的映射（加速查找） ----
    vector<int> pkg_trip(n_pkg, -1);
    auto rebuild_pkg_trip = [&]() {
        for (int t = 0; t < (int)trip_plans.size(); ++t)
            for (int pid : trip_plans[t])
                pkg_trip[pid] = t;
    };
    rebuild_pkg_trip();

    // ---- 当前解与最优解 ----
    vector<double> cur_deliv(n_pkg), cur_diss(n_pkg);  // 当前解的送达时间和不满意度
    cur_score = evaluate_solution(trip_plans, dest_orders, cur_deliv, cur_diss);
    double best_score = cur_score;    // 最优分数
    auto best_plans = trip_plans;     // 最优趟分配
    auto best_orders = dest_orders;   // 最优目的地顺序

    std::uniform_real_distribution<double> uni(0.0, 1.0);  // [0,1) 均匀分布，用于接受概率判定

    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        auto saved_plans = trip_plans;    // 备份当前趟分配（以便回退）
        auto saved_orders = dest_orders;  // 备份当前目的地顺序

        if (!try_random_move(trip_plans, dest_orders))
        {
            // 扰动失败（如无合法移动），回退
            trip_plans = saved_plans;
            dest_orders = saved_orders;
            goto COOL;
        }

        {
            vector<double> nd(n_pkg), nds(n_pkg);  // nd: 新解的送达时间, nds: 新解的不满意度
            double ns = evaluate_solution(trip_plans, dest_orders, nd, nds);  // ns: 新解总不满意度
            double delta = ns - cur_score;  // 分数差（<0 表示改进）

            // Metropolis 准则：变好一定接受，变差以概率 exp(-delta/T) 接受
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
                // 拒绝新解，回退
                trip_plans = saved_plans;
                dest_orders = saved_orders;
            }
        }

    COOL:
        // Lundy-Mees 降温
        T = T / (1.0 + beta * T);
        if (T < T_MIN) break;
    }

    // 返回搜索过程中找到的最优解
    trip_plans = best_plans;
    dest_orders = best_orders;
}

// ============ 主入口 ============

Task2Result Task2::solve()
{
    // 1. 贪心初解
    // trip_plans[t]: 第 t 趟携带的包裹 id 列表
    // dest_orders[t]: 第 t 趟的目的地访问顺序
    vector<vector<int>> trip_plans, dest_orders;
    greedy_init(trip_plans, dest_orders);

    // 2. 模拟退火优化
    sa_optimize(trip_plans, dest_orders);

    // 3. 最终评估
    vector<double> deliver_time(n_pkg), dissatisfaction(n_pkg);
    double total = evaluate_solution(trip_plans, dest_orders, deliver_time, dissatisfaction);

    // 4. 构造 Trip 列表（输出格式）
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