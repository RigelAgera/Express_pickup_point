// T3: 带容量运送成本，逻辑基本同 T2, 但 S_i = 0, 有 deadline T_i
// 约束: S_i = 0, 容量 w_lim, 有 deadline T_i
// 算法: EDD 贪心分趟 + heaviest-first 趟内排序 + SA 优化
// 评估: 加权评分 score = M * overtime + total_cost, M 足够大使超时主导
#include "task3.h"
#include "../common/dijkstra.h"
#include "../common/min_heap.h"
#include "../common/circular_queue.h"
#include "../common/schedule_utils.h"
#include <algorithm>
#include <numeric>
#include <iostream>
#include <chrono>
#include <cmath>

using std::vector;

// ========== 构造函数 ==========
Task3::Task3(const input_data& data)
    : g(data.g), pkgs(data.packages), c(data.c),
      ap(all_pairs_dijkstra(data.g)),
      n_pkg((int)data.packages.size()),
      rng((unsigned)std::chrono::steady_clock::now().time_since_epoch().count())
{
    // T3 要求 S_i = 0, 强制清零
    for (auto& p : pkgs)
    {
        if (p.arrive != 0.0)
        {
            p.arrive = 0.0;
            // std::cout << "Warning: package " << p.id
            //           << " arrive time set to 0 for T3." << std::endl;
        }
    }
}

// ========== unique_dests ==========
// 返回 pkg_ids 中去重后的目的地节点列表
vector<int> Task3::unique_dests(const vector<int>& pkg_ids)
{
    vector<int> dests;
    for (int pid : pkg_ids)
        dests.push_back(pkgs[pid].dest);
    std::sort(dests.begin(), dests.end());
    dests.erase(std::unique(dests.begin(), dests.end()), dests.end());
    return dests;
}

// ========== trip_weight ==========
// 返回一趟中所有包裹的总重量
// $$W(\text{trip}) = \sum_{i \in \text{trip}} w_i$$
double Task3::trip_weight(const vector<int>& pkg_ids)
{
    double w = 0.0;
    for (int pid : pkg_ids)
        w += pkgs[pid].weight;
    return w;
}

// ========== heaviest_dest_order ==========
// 趟内排序: 按目的地总重量降序, 重者先送以降低路段成本
// $$\text{order} = \operatorname{sort\_desc}\left(
//    \text{dests},\; \text{key}(u) = \sum_{i \in \text{trip},\; d_i = u} w_i\right)$$
vector<int> Task3::heaviest_dest_order(const vector<int>& pkg_ids)
{
    vector<int> dests = unique_dests(pkg_ids);
    // 计算每个目的地的包裹总重
    vector<std::pair<double, int>> scored;
    for (int d : dests)
    {
        double tw = 0.0;
        for (int pid : pkg_ids)
            if (pkgs[pid].dest == d)
                tw += pkgs[pid].weight;
        scored.emplace_back(tw, d);
    }
    // 按重量降序
    std::sort(scored.rbegin(), scored.rend());
    vector<int> order;
    for (auto& p : scored)
        order.push_back(p.second);
    return order;
}

// ========== evaluate_solution ==========
// 模拟完整方案, 返回 (超时包裹数, 总成本), 填入各包裹送达时间 D_i
//
// 单趟成本 (逐段累加):
// $$\text{cost}(\text{trip}) = \sum_{k=1}^{|\text{route}|-1}
//    \text{dist}[u_{k-1}][u_k] \cdot
//    \bigl(w_{\text{car}} + \sum_{i\in \text{remain}} w_i\bigr)$$
//
// 超时判定:
// $$\text{overtime\_count} = \bigl|\{\,i \mid D_i > T_i\,\}\bigr|$$
std::pair<int, double> Task3::evaluate_solution(
    const vector<vector<int>>& trip_plans,
    const vector<vector<int>>& dest_orders,
    vector<double>& deliver_time)
{
    deliver_time.assign(n_pkg, -1.0);
    double cur_time = 0.0;   // 小车当前可用时刻 (回到驿站的时间)
    double total_cost = 0.0;
    int overtime = 0;

    for (size_t t = 0; t < trip_plans.size(); ++t)
    {
        const auto& pkg_ids = trip_plans[t];
        const auto& dest_order = dest_orders[t];
        if (pkg_ids.empty()) continue;

        // 构建路线 & 模拟本趟
        // depart_time = cur_time (S_i=0, 无需等待)
        vector<int> route = build_route(dest_order, ap);
        Trip sim = simulate_trip(pkg_ids, route, cur_time, pkgs, c, ap);

        // 累计时间与成本
        cur_time = sim.end_time;
        total_cost += trip_cost(sim, g, pkgs, c, ap);

        // 记录送达时间 D_i, 统计超时
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

// ========== greedy_init ==========
// EDD (Earliest Deadline First) 贪心分趟:
//   1. 所有包裹按 T_i 升序排序
//   2. 依次贪心装车 (容量约束), 装满即封趟
//   3. 每趟内部用 heaviest_dest_order 排目的地
//
// $$\text{order} = \operatorname{sort\_asc}(\text{pkgs},\; \text{key} = T_i)$$
void Task3::greedy_init(
    vector<vector<int>>& trip_plans,
    vector<vector<int>>& dest_orders)
{
    // 按 deadline 升序 (EDD)
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
        // 容量约束: 装不下就封趟
        if (cur_weight + w > c.capacity + 1e-9 && !cur_trip.empty())
        {
            trip_plans.push_back(cur_trip);
            dest_orders.push_back(heaviest_dest_order(cur_trip));
            cur_trip.clear();
            cur_weight = 0.0;
        }
        cur_trip.push_back(pid);
        cur_weight += w;
    }

    if (!cur_trip.empty())
    {
        trip_plans.push_back(cur_trip);
        dest_orders.push_back(heaviest_dest_order(cur_trip));
    }
}

// ========== try_random_move ==========
// 随机尝试一种邻域移动, 修改 trip_plans / dest_orders
// 成功返回 true, 无法产生合法移动返回 false
//
// 三种操作 (等概率):
//   op=0: 跨趟搬包裹 (move)
//   op=1: 趟内交换两个目的地 (swap)
//   op=2: 趟内 2-opt (reverse sub-path)
bool Task3::try_random_move(
    vector<vector<int>>& trip_plans,
    vector<vector<int>>& dest_orders)
{
    int n_trip = (int)trip_plans.size();
    int op = std::uniform_int_distribution<int>(0, 2)(rng);

    if (op == 0 && n_trip >= 2)
    {
        // 跨趟搬包裹: 从 src 趟随机取一个包裹移到 dst 趟
        int src = std::uniform_int_distribution<int>(0, n_trip - 1)(rng);
        if (trip_plans[src].size() <= 1) return false;
        int pos = std::uniform_int_distribution<int>(
            0, (int)trip_plans[src].size() - 1)(rng);
        int pid = trip_plans[src][pos];
        double pw = pkgs[pid].weight;

        int dst;
        do { dst = std::uniform_int_distribution<int>(0, n_trip - 1)(rng); }
        while (dst == src);

        if (trip_weight(trip_plans[dst]) + pw > c.capacity + 1e-9)
            return false;

        trip_plans[src].erase(trip_plans[src].begin() + pos);
        trip_plans[dst].push_back(pid);
        dest_orders[src] = heaviest_dest_order(trip_plans[src]);
        dest_orders[dst] = heaviest_dest_order(trip_plans[dst]);
        if (trip_plans[src].empty())
        {
            trip_plans.erase(trip_plans.begin() + src);
            dest_orders.erase(dest_orders.begin() + src);
        }
        return true;
    }
    else if (op == 1 && n_trip >= 1)
    {
        // 趟内交换两个目的地
        int t = std::uniform_int_distribution<int>(0, n_trip - 1)(rng);
        if (dest_orders[t].size() < 2) return false;
        int i = std::uniform_int_distribution<int>(
            0, (int)dest_orders[t].size() - 1)(rng);
        int j = std::uniform_int_distribution<int>(
            0, (int)dest_orders[t].size() - 1)(rng);
        if (i == j) return false;
        std::swap(dest_orders[t][i], dest_orders[t][j]);
        return true;
    }
    else if (op == 2 && n_trip >= 1)
    {
        // 趟内 2-opt: 反转某段子路径
        int t = std::uniform_int_distribution<int>(0, n_trip - 1)(rng);
        int sz = (int)dest_orders[t].size();
        if (sz < 2) return false;
        int i = std::uniform_int_distribution<int>(0, sz - 2)(rng);
        int j = std::uniform_int_distribution<int>(i + 1, sz - 1)(rng);
        std::reverse(dest_orders[t].begin() + i,
                     dest_orders[t].begin() + j + 1);
        return true;
    }
    return false;
}

// ========== sa_optimize ==========
// 模拟退火优化, 使用加权评分:
// $$\text{score} = M \cdot \text{overtime\_count} + \text{total\_cost}$$
// M 足够大使得超时数成为主导项 (优先压超时), 同超时下优化成本
//
// 接受准则:
// $$P(\text{accept}) = \begin{cases}
//    1, & \Delta < 0 \\
//    \exp(-\Delta / T), & \Delta \ge 0
// \end{cases}$$
//
// 温控: Lundy-Mees 恒温退火
// $$T_{k+1} = \frac{T_k}{1 + \beta \cdot T_k}$$
void Task3::sa_optimize(
    vector<vector<int>>& trip_plans,
    vector<vector<int>>& dest_orders)
{
    // M: 超时惩罚系数, 取足够大使其主导评分
    // 参考: 示例数据一趟成本约 850, 大规模数据预计在 10^5 量级
    const double M = 1e8;

    auto score = [&](int ot, double cost) -> double {
        return M * static_cast<double>(ot) + cost;
    };

    // 当前解评估
    vector<double> cur_deliv(n_pkg);
    auto [cur_ot, cur_cost] = evaluate_solution(
        trip_plans, dest_orders, cur_deliv);
    double cur_score = score(cur_ot, cur_cost);

    double best_score = cur_score;
    auto best_plans = trip_plans;
    auto best_orders = dest_orders;

    // ---- 自适应初始温度 ----
    // 对初解做 100 次随机扰动, 用上坡 Δ 的中位数反推 T_init (接受率 ≈ 0.8)
    vector<double> deltas;
    for (int k = 0; k < 100; ++k)
    {
        auto p2 = trip_plans, o2 = dest_orders;
        if (!try_random_move(p2, o2)) continue;
        vector<double> dd(n_pkg);
        auto [ot2, cost2] = evaluate_solution(p2, o2, dd);
        double ns = score(ot2, cost2);
        double d = ns - cur_score;
        if (d > 0) deltas.push_back(d);
    }
    double T = 1.0;
    if (!deltas.empty())
    {
        std::sort(deltas.begin(), deltas.end());
        double delta_med = deltas[deltas.size() / 2];    // 中位数
        T = delta_med / 0.223;   // exp(-Δ/T) = 0.8  ⟹  T = Δ / ln(1/0.8) ≈ Δ / 0.223
        if (T < 0.1) T = 0.1;
    }

    // ---- Lundy-Mees 参数 ----
    const int    MAX_ITER = 100000;
    const double T_MIN    = 1e-3;
    double beta = (T - T_MIN) / (MAX_ITER * T * T_MIN);

    std::uniform_real_distribution<double> uni(0.0, 1.0);

    // ---- 主循环 ----
    for (int iter = 0; iter < MAX_ITER; ++iter)
    {
        auto saved_plans = trip_plans;
        auto saved_orders = dest_orders;

        if (!try_random_move(trip_plans, dest_orders))
        {
            trip_plans = saved_plans;
            dest_orders = saved_orders;
            goto COOL;
        }

        {
            vector<double> nd(n_pkg);
            auto [ot_new, cost_new] = evaluate_solution(
                trip_plans, dest_orders, nd);
            double new_score = score(ot_new, cost_new);
            double delta = new_score - cur_score;

            if (delta < 0 || uni(rng) < std::exp(-delta / T))
            {
                cur_score = new_score;
                cur_ot    = ot_new;
                cur_cost  = cost_new;
                cur_deliv = nd;
                if (cur_score < best_score)
                {
                    best_score = cur_score;
                    best_plans = trip_plans;
                    best_orders = dest_orders;
                }
            }
            else
            {
                trip_plans = saved_plans;
                dest_orders = saved_orders;
            }
        }

    COOL:
        T = T / (1.0 + beta * T);
        if (T < T_MIN) break;
    }

    trip_plans = best_plans;
    dest_orders = best_orders;
}

// ========== solve (主入口) ==========
Task3Result Task3::solve()
{
    // 1. EDD 贪心初解
    vector<vector<int>> trip_plans, dest_orders;
    greedy_init(trip_plans, dest_orders);

    // 2. SA 优化
    sa_optimize(trip_plans, dest_orders);

    // 3. 最终评估
    vector<double> deliver_time(n_pkg);
    auto [overtime, total_cost] = evaluate_solution(
        trip_plans, dest_orders, deliver_time);

    // 4. 构造 Trip 输出列表
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