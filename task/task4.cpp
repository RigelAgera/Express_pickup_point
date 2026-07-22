#include "task4.h"
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <numeric>

Task4::Task4(const input_data& data)
    : g(data.g), v(data.c.speed), ap(all_pairs_dijkstra(data.g)) {}

// 计算给定访问顺序对应的回路总路径长度
// 路径：仓库0 → order[0] → order[1] → ... → order.back() → 仓库0
double Task4::tour_length(const vector<int>& order)
{
    if (order.empty()) return 0.0;
    double total = ap.dist[0][order[0]];                       // 从仓库到第一个节点
    for (size_t i = 0; i + 1 < order.size(); ++i)
        total += ap.dist[order[i]][order[i + 1]];              // 节点间顺序移动
    total += ap.dist[order.back()][0];                         // 从最后一个节点返回仓库
    return total;
}

// 最近邻贪心构造初始回路：每次从剩余节点中选择距离当前节点最近的一个
vector<int> Task4::nearest_neighbor_order(const vector<int>& nodes)
{
    if (nodes.empty()) return {};
    vector<int> remaining = nodes;   // 尚未访问的节点集合
    vector<int> result;              // 访问顺序结果
    int cur = 0;                     // 当前所在位置，初始为仓库0
    while (!remaining.empty()) {
        int best_idx = 0;
        double best_dist = 1e18;
        for (size_t i = 0; i < remaining.size(); ++i) {
            double d = ap.dist[cur][remaining[i]];
            if (d < best_dist) {
                best_dist = d;
                best_idx = (int)i;
            }
        }
        cur = remaining[best_idx];
        result.push_back(cur);
        remaining.erase(remaining.begin() + best_idx);
    }
    return result;
}

// OX (Order Crossover)：顺序交叉算子，用于两个父代排列生成子代
// 随机选择一段区间从父代a复制，其余位置按父代b的顺序填充未使用的节点
vector<int> Task4::ox_crossover(const vector<int>& a, const vector<int>& b, std::mt19937& rng)
{
    int n = (int)a.size();
    if (n <= 1) return a;
    std::uniform_int_distribution<int> dist(0, n - 1);
    int p1 = dist(rng), p2 = dist(rng);    // 随机选择交叉区间的两个端点
    if (p1 > p2) std::swap(p1, p2);        // 保证 p1 <= p2

    vector<int> child(n, -1);              // 子代，-1 表示尚未填充的位置
    // 复制 a[p1..p2] 到 child
    for (int i = p1; i <= p2; ++i)
        child[i] = a[i];

    // 从 b 中按顺序填充未占用的位置（从 p2+1 位置开始循环取元素）
    int fill = 0;                          // 下一个可填充位置
    for (int i = 0; i < n; ++i) {
        int val = b[(p2 + 1 + i) % n];     // 从 b 中循环取元素，保证不重复
        bool used = false;
        for (int j = 0; j < n; ++j) {
            if (child[j] == val) { used = true; break; }
        }
        if (!used) {
            while (child[fill % n] != -1) ++fill;  // 找到下一个空位
            child[fill % n] = val;
        }
    }
    return child;
}

// 复合变异算子：以一定概率对个体执行 swap（交换）、inversion（反转）或 scramble（打乱）操作
void Task4::mutate(vector<int>& order, std::mt19937& rng)
{
    int n = (int)order.size();
    if (n <= 1) return;

    std::uniform_real_distribution<double> prob(0.0, 1.0);  // [0,1) 均匀分布，用于概率判断
    std::uniform_int_distribution<int> idx(0, n - 1);        // 随机索引

    // swap 变异：随机交换两个位置 (概率 0.3)
    if (prob(rng) < 0.3) {
        int i = idx(rng), j = idx(rng);
        std::swap(order[i], order[j]);
    }
    // inversion 变异：随机反转一段子序列 (概率 0.2)
    if (prob(rng) < 0.2) {
        int i = idx(rng), j = idx(rng);
        if (i > j) std::swap(i, j);
        std::reverse(order.begin() + i, order.begin() + j + 1);
    }
    // scramble 变异：随机打乱一段子序列 (概率 0.1)
    if (prob(rng) < 0.1) {
        int i = idx(rng), j = idx(rng);
        if (i > j) std::swap(i, j);
        if (i < j)
            std::shuffle(order.begin() + i, order.begin() + j + 1, rng);
    }
}

// 初始化种群：20% 用最近邻贪心构造，其余随机生成
// pop_size: 种群规模
vector<vector<int>> Task4::init_population(const vector<int>& nodes, int pop_size, std::mt19937& rng)
{
    vector<vector<int>> pop;                        // 种群（pop_size 个个体的排列）
    int greedy_count = std::max(1, pop_size / 5);   // 贪心个体数量，至少 1 个

    // 贪心个体
    for (int k = 0; k < greedy_count; ++k) {
        vector<int> ind = nearest_neighbor_order(nodes);
        pop.push_back(ind);
    }
    // 剩余随机生成
    while ((int)pop.size() < pop_size) {
        vector<int> ind = nodes;
        std::shuffle(ind.begin(), ind.end(), rng);
        pop.push_back(ind);
    }
    return pop;
}

// 锦标赛选择：随机选 t_size 个个体，返回其中适应度最高者的索引
int Task4::tournament_select(const vector<double>& fitness, std::mt19937& rng, int t_size)
{
    int pop_size = (int)fitness.size();
    std::uniform_int_distribution<int> dist(0, pop_size - 1);  // 均匀随机索引
    int best = dist(rng);
    for (int i = 1; i < t_size; ++i) {
        int cand = dist(rng);
        if (fitness[cand] > fitness[best])    // 适应度越大越好（= 1/路径长度）
            best = cand;
    }
    return best;
}

// 遗传算法主求解流程
Task4Result Task4::solve(const vector<int>& return_nodes)
{
    Task4Result result;             // 求解结果：最优访问顺序
    if (return_nodes.empty()) {
        result.destination = {};
        return result;
    }

    const int n = (int)return_nodes.size();

    // ── 遗传算法参数 ──
    const int pop_size = std::min(200, std::max(20, 4 * n));  // 种群规模：随问题规模自适应，范围 [20, 200]
    const double pc = 0.85;                                    // 交叉概率
    const double pm = 0.30;                                    // 变异概率
    const int elite_count = 2;                                 // 精英保留数量
    const int max_gen = std::max(500, pop_size * 10);          // 最大迭代代数
    const int stagnation_limit = 200;                          // 停滞代数上限：连续多少代无改进则提前终止

    std::mt19937 rng(42);  // Mersenne Twister 随机数生成器，固定种子 42 保证可复现

    // 初始种群
    vector<vector<int>> population = init_population(return_nodes, pop_size, rng);

    // 适应度向量：fitness[i] = 1 / tour_length(population[i])，适应度越高越优
    vector<double> fitness(pop_size);
    double best_fitness = 0.0;                       // 历史最优适应度
    vector<int> best_individual = population[0];     // 历史最优个体（最优访问顺序）
    int stagnant = 0;                                // 连续未改进代数计数器

    for (int gen = 0; gen < max_gen; ++gen) {
        // 评估种群中每个个体的适应度
        for (int i = 0; i < pop_size; ++i) {
            double len = tour_length(population[i]);
            fitness[i] = 1.0 / (len + 1e-9);         // +1e-9 防止除零
            if (fitness[i] > best_fitness) {         // 发现更优个体
                best_fitness = fitness[i];
                best_individual = population[i];
                stagnant = 0;                        // 重置停滞计数
            }
        }

        if (++stagnant >= stagnation_limit)          // 达到停滞上限，提前终止进化
            break;

        // 锦标赛选择：从种群中随机选3个个体，取适应度最高者
        vector<vector<int>> next_gen;                // 下一代种群

        // 精英保留：将当前种群中适应度最高的 elite_count 个个体直接复制到下一代
        vector<int> sorted_idx(pop_size);            // 按适应度降序排列的索引
        std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
        std::sort(sorted_idx.begin(), sorted_idx.end(),
            [&](int i, int j) { return fitness[i] > fitness[j]; });
        for (int e = 0; e < elite_count && e < pop_size; ++e)
            next_gen.push_back(population[sorted_idx[e]]);   // 精英个体直接进入下一代

        // 生成剩余个体：选择-交叉-变异
        while ((int)next_gen.size() < pop_size) {
            // 锦标赛选择两个父代
            int p1 = tournament_select(fitness, rng, 3);
            int p2 = tournament_select(fitness, rng, 3);

            vector<int> child;
            std::uniform_real_distribution<double> prob(0.0, 1.0);
            // 以概率 pc 进行 OX 交叉，否则直接复制父代 p1
            if (prob(rng) < pc)
                child = ox_crossover(population[p1], population[p2], rng);
            else
                child = population[p1];

            // 以概率 pm 对子代进行变异
            if (prob(rng) < pm)
                mutate(child, rng);

            next_gen.push_back(child);
        }

        population = std::move(next_gen);  // 替换为新一代种群
        fitness.resize(pop_size);          // 适应度向量调整大小
    }

    result.destination = best_individual;  // 输出找到的最优访问顺序
    return result;
}