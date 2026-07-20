// T3 vs T3Simple 对比测试
// 用法: main_t3_compare [数据文件夹路径，默认"测试数据/示例"]
// 输出: 两个版本的结果对比
#include "common/input_reader.h"
#include "task/task3.h"
#include "task/task3_simple.h"
#include <iostream>
#include <fstream>
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

        // ---- T3 完整版 (SA) ----
        auto t0 = std::chrono::steady_clock::now();
        Task3 solver_sa(data1);
        auto r_sa = solver_sa.solve();
        auto t1 = std::chrono::steady_clock::now();
        double ms_sa = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // ---- T3 简化版 (纯贪心) ----
        t0 = std::chrono::steady_clock::now();
        Task3Simple solver_simple(data2);
        auto r_simple = solver_simple.solve();
        t1 = std::chrono::steady_clock::now();
        double ms_simple = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // ---- 输出对比 ----
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "                 T3(SA)       T3Simple\n";
        std::cout << "  总成本:   " << std::setw(12) << r_sa.sumCost
                  << "   " << std::setw(12) << r_simple.sumCost << "\n";
        std::cout << "  超时数:   " << std::setw(12) << r_sa.exTime
                  << "   " << std::setw(12) << r_simple.exTime << "\n";
        std::cout << "  总趟数:   " << std::setw(12) << r_sa.trips.size()
                  << "   " << std::setw(12) << r_simple.trips.size() << "\n";
        std::cout << "  耗时(ms): " << std::setw(12) << ms_sa
                  << "   " << std::setw(12) << ms_simple << "\n";

        // 差值
        double cost_diff = r_simple.sumCost - r_sa.sumCost;
        int ot_diff = r_simple.exTime - r_sa.exTime;
        std::cout << "\n  成本差 (Simple-SA): " << cost_diff
                  << "  (" << (std::abs(r_sa.sumCost) > 1e-9 ? 100.0 * cost_diff / r_sa.sumCost : 0.0) << "%)\n";
        std::cout << "  超时差 (Simple-SA): " << ot_diff << "\n";
        std::cout << std::endl;
    }

    return 0;
}