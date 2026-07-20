// T5 vs T5Simple 对比测试
// 用法: main_t5_compare [数据文件夹路径，默认"测试数据/示例"]
#include "common/input_reader.h"
#include "task/task5.h"
#include "task/task5_simple.h"
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>

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

        input_data data1 = read_input(folder);
        input_data data2 = read_input(folder);

        // ---- T5 完整版 (SA+ALNS) ----
        auto t0 = std::chrono::steady_clock::now();
        Task5 solver_full(data1);
        auto r_full = solver_full.solve();
        auto t1 = std::chrono::steady_clock::now();
        double ms_full = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // ---- T5 简化版 (K-means + 贪心) ----
        t0 = std::chrono::steady_clock::now();
        Task5Simple solver_simple(data2);
        auto r_simple = solver_simple.solve();
        t1 = std::chrono::steady_clock::now();
        double ms_simple = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // ---- 输出对比 ----
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "                  T5(ALNS)      T5Simple\n";
        std::cout << "  总成本:     " << std::setw(12) << r_full.sumCost
                  << "   " << std::setw(12) << r_simple.sumCost << "\n";
        std::cout << "  超时数:     " << std::setw(12) << r_full.exTime
                  << "   " << std::setw(12) << r_simple.exTime << "\n";
        std::cout << "  车1趟数:    " << std::setw(12) << r_full.trips1.size()
                  << "   " << std::setw(12) << r_simple.trips1.size() << "\n";
        std::cout << "  车2趟数:    " << std::setw(12) << r_full.trips2.size()
                  << "   " << std::setw(12) << r_simple.trips2.size() << "\n";
        std::cout << "  耗时(ms):   " << std::setw(12) << ms_full
                  << "   " << std::setw(12) << ms_simple << "\n";

        double cost_diff = r_simple.sumCost - r_full.sumCost;
        int ot_diff = r_simple.exTime - r_full.exTime;
        std::cout << "\n  成本差 (Simple-ALNS): " << cost_diff
                  << "  (" << (std::abs(r_full.sumCost) > 1e-9 ? 100.0 * cost_diff / r_full.sumCost : 0.0) << "%)\n";
        std::cout << "  超时差 (Simple-ALNS): " << ot_diff << "\n";
        std::cout << std::endl;
    }

    return 0;
}