// T4 vs T4Simple 对比测试
// 用法: main_t4_compare [数据文件夹路径，默认"测试数据/示例"]
// return_nodes 取所有包裹目的地去重后的集合
#include "common/input_reader.h"
#include "task/task4.h"
#include "task/task4_simple.h"
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>
#include <algorithm>

int main(int argc, char* argv[])
{
    std::vector<std::string> folders;
    for (int i = 1; i < argc; ++i)
        folders.push_back(argv[i]);
    if (folders.empty())
        folders.push_back("测试数据/示例");

    for (const auto& folder : folders)
    {
        std::cout << "========================================\n";
        std::cout << "[" << folder << "]\n";

        input_data data = read_input(folder);

        // 构造 return_nodes: 所有包裹目的地去重
        std::vector<int> return_nodes;
        for (const auto& p : data.packages)
            return_nodes.push_back(p.dest);
        std::sort(return_nodes.begin(), return_nodes.end());
        return_nodes.erase(std::unique(return_nodes.begin(), return_nodes.end()), return_nodes.end());

        // ---- T4 完整版 (GA) ----
        auto t0 = std::chrono::steady_clock::now();
        Task4 solver_ga(data);
        auto r_ga = solver_ga.solve(return_nodes);
        auto t1 = std::chrono::steady_clock::now();
        double ms_ga = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // ---- T4 简化版 (最近邻贪心) ----
        t0 = std::chrono::steady_clock::now();
        Task4Simple solver_simple(data);
        auto r_simple = solver_simple.solve(return_nodes);
        t1 = std::chrono::steady_clock::now();
        double ms_simple = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // 计算回路长度
        Task4Simple tmp(data);  // 复用其 tour_length
        double len_ga = tmp.tour_length(r_ga.destination);
        double len_simple = tmp.tour_length(r_simple.destination);

        // ---- 输出对比 ----
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  节点数: " << return_nodes.size() << "\n";
        std::cout << "                  T4(GA)      T4Simple\n";
        std::cout << "  回路长度:  " << std::setw(12) << len_ga
                  << "   " << std::setw(12) << len_simple << "\n";
        std::cout << "  耗时(ms):  " << std::setw(12) << ms_ga
                  << "   " << std::setw(12) << ms_simple << "\n";

        double diff = len_simple - len_ga;
        std::cout << "\n  长度差 (Simple-GA): " << diff
                  << "  (" << (std::abs(len_ga) > 1e-9 ? 100.0 * diff / len_ga : 0.0) << "%)\n";

        // 输出路径对比
        std::cout << "  GA路径:     ";
        for (int nd : r_ga.destination) std::cout << nd << " ";
        std::cout << "\n  Simple路径: ";
        for (int nd : r_simple.destination) std::cout << nd << " ";
        std::cout << "\n" << std::endl;
    }

    return 0;
}