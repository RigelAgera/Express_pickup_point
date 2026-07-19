#include "task4.h"
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <numeric>

Task4::Task4(const input_data& data)
    : g(data.g), v(data.c.speed), ap(all_pairs_dijkstra(data.g)) {}

double Task4::tour_length(const vector<int>& order)
{
    if (order.empty()) return 0.0;
    double total = ap.dist[0][order[0]];
    for (size_t i = 0; i + 1 < order.size(); ++i)
        total += ap.dist[order[i]][order[i + 1]];
    total += ap.dist[order.back()][0];
    return total;
}

// 最近邻贪心（与 schedule_utils 中的类似但独立实现以确保兼容）
vector<int> Task4::nearest_neighbor_order(const vector<int>& nodes)
{
    if (nodes.empty()) return {};
    vector<int> remaining = nodes;
    vector<int> result;
    int cur = 0;
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

// OX (Order Crossover)
vector<int> Task4::ox_crossover(const vector<int>& a, const vector<int>& b, std::mt19937& rng)
{
    int n = (int)a.size();
    if (n <= 1) return a;
    std::uniform_int_distribution<int> dist(0, n - 1);
    int p1 = dist(rng), p2 = dist(rng);
    if (p1 > p2) std::swap(p1, p2);

    vector<int> child(n, -1);
    // 复制 a[p1..p2] 到 child
    for (int i = p1; i <= p2; ++i)
        child[i] = a[i];

    // 从 b 中按顺序填充未占用的位置
    int fill = 0;
    for (int i = 0; i < n; ++i) {
        // 从 b 中取元素
        int val = b[(p2 + 1 + i) % n];
        // 检查 val 是否已在 child 中
        bool used = false;
        for (int j = 0; j < n; ++j) {
            if (child[j] == val) { used = true; break; }
        }
        if (!used) {
            while (child[fill % n] != -1) ++fill;
            child[fill % n] = val;
        }
    }
    return child;
}

void Task4::mutate(vector<int>& order, std::mt19937& rng)
{
    int n = (int)order.size();
    if (n <= 1) return;

    std::uniform_real_distribution<double> prob(0.0, 1.0);
    std::uniform_int_distribution<int> idx(0, n - 1);

    // swap 变异 (0.3)
    if (prob(rng) < 0.3) {
        int i = idx(rng), j = idx(rng);
        std::swap(order[i], order[j]);
    }
    // inversion 变异 (0.2)
    if (prob(rng) < 0.2) {
        int i = idx(rng), j = idx(rng);
        if (i > j) std::swap(i, j);
        std::reverse(order.begin() + i, order.begin() + j + 1);
    }
    // scramble 变异 (0.1)
    if (prob(rng) < 0.1) {
        int i = idx(rng), j = idx(rng);
        if (i > j) std::swap(i, j);
        if (i < j)
            std::shuffle(order.begin() + i, order.begin() + j + 1, rng);
    }
}

vector<vector<int>> Task4::init_population(const vector<int>& nodes, int pop_size, std::mt19937& rng)
{
    vector<vector<int>> pop;
    int greedy_count = std::max(1, pop_size / 5);  // 20% 贪心

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

int Task4::tournament_select(const vector<double>& fitness, std::mt19937& rng, int t_size)
{
    int pop_size = (int)fitness.size();
    std::uniform_int_distribution<int> dist(0, pop_size - 1);
    int best = dist(rng);
    for (int i = 1; i < t_size; ++i) {
        int cand = dist(rng);
        if (fitness[cand] > fitness[best])
            best = cand;
    }
    return best;
}

Task4Result Task4::solve(const vector<int>& return_nodes)
{
    Task4Result result;
    if (return_nodes.empty()) {
        result.destination = {};
        return result;
    }

    const int n = (int)return_nodes.size();
    const int pop_size = std::min(200, std::max(20, 4 * n));
    const double pc = 0.85;   // 交叉概率
    const double pm = 0.30;   // 变异概率
    const int elite_count = 2;
    const int max_gen = std::max(500, pop_size * 10);
    const int stagnation_limit = 200;

    std::mt19937 rng(42);  // 固定种子，可复现

    // 初始种群
    vector<vector<int>> population = init_population(return_nodes, pop_size, rng);

    // 计算适应度 (1 / tour_length)
    vector<double> fitness(pop_size);
    double best_fitness = 0.0;
    vector<int> best_individual = population[0];
    int stagnant = 0;

    for (int gen = 0; gen < max_gen; ++gen) {
        // 评估
        for (int i = 0; i < pop_size; ++i) {
            double len = tour_length(population[i]);
            fitness[i] = 1.0 / (len + 1e-9);
            if (fitness[i] > best_fitness) {
                best_fitness = fitness[i];
                best_individual = population[i];
                stagnant = 0;
            }
        }

        if (++stagnant >= stagnation_limit)
            break;

        // 下一代
        vector<vector<int>> next_gen;

        // 精英保留：复制最优个体
        // 找出精英索引
        vector<int> sorted_idx(pop_size);
        std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
        std::sort(sorted_idx.begin(), sorted_idx.end(),
            [&](int i, int j) { return fitness[i] > fitness[j]; });
        for (int e = 0; e < elite_count && e < pop_size; ++e)
            next_gen.push_back(population[sorted_idx[e]]);

        // 生成剩余个体
        while ((int)next_gen.size() < pop_size) {
            int p1 = tournament_select(fitness, rng, 3);
            int p2 = tournament_select(fitness, rng, 3);

            vector<int> child;
            std::uniform_real_distribution<double> prob(0.0, 1.0);
            if (prob(rng) < pc)
                child = ox_crossover(population[p1], population[p2], rng);
            else
                child = population[p1];

            if (prob(rng) < pm)
                mutate(child, rng);

            next_gen.push_back(child);
        }

        population = std::move(next_gen);
        fitness.resize(pop_size);
    }

    result.destination = best_individual;
    return result;
}