// T2 vs T2Simple 对比测试
// 用法: main_t2_compare [数据文件夹路径，默认"测试数据/示例"]
#include "common/input_reader.h"
#include "task/task2.h"
#include "task/task2_simple.h"
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

        // ---- T2 完整版 (SA) ----
        auto t0 = std::chrono::steady_clock::now();
        Task2 solver_sa(data1);
        auto r_sa = solver_sa.solve();
        auto t1 = std::chrono::steady_clock::now();
        double ms_sa = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // ---- T2 简化版 (纯贪心) ----
        t0 = std::chrono::steady_clock::now();
        Task2Simple solver_simple(data2);
        auto r_simple = solver_simple.solve();
        t1 = std::chrono::steady_clock::now();
        double ms_simple = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // ---- 输出对比 ----
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "                  T2(SA)       T2Simple\n";
        std::cout << "  总不满意度: " << std::setw(14) << r_sa.total_dissatisfaction
                  << "   " << std::setw(14) << r_simple.total_dissatisfaction << "\n";
        std::cout << "  总趟数:     " << std::setw(14) << r_sa.trips.size()
                  << "   " << std::setw(14) << r_simple.trips.size() << "\n";
        std::cout << "  耗时(ms):   " << std::setw(14) << ms_sa
                  << "   " << std::setw(14) << ms_simple << "\n";

        double diff = r_simple.total_dissatisfaction - r_sa.total_dissatisfaction;
        std::cout << "\n  不满意度差 (Simple-SA): " << diff
                  << "  (" << (std::abs(r_sa.total_dissatisfaction) > 1e-9
                                 ? 100.0 * diff / r_sa.total_dissatisfaction : 0.0)
                  << "%)\n";
        std::cout << std::endl;
    }

    return 0;
}