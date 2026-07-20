// T2 独立测试主函数
// 编译: g++ -std=c++17 -O2 task/task2_test.cpp task/task2.cpp common/input_reader.cpp common/output_writer.cpp common/schedule_utils.cpp -o task2_test.exe
#include "../common/input_reader.h"
#include "../common/output_writer.h"
#include "task2.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

int main(int argc, char* argv[])
{
    std::vector<std::string> folders;
    for (int i = 1; i < argc; ++i)
        folders.push_back(argv[i]);

    if (folders.empty())
        folders.push_back("测试数据/示例");

    for (const auto& folder : folders)
    {
        input_data data = read_input(folder);

        // T2: 忽略超时约束
        for (auto& p : data.packages)
            p.deadline = 1e9;

        Task2 solver(data);
        Task2Result r = solver.solve();

        std::ofstream out(to_path(folder + "/result_t2.md"));
        if (!out)
        {
            std::cerr << "无法创建 " << folder << "/result_t2.md\n";
            continue;
        }

        output::format_md(folder, out);
        output::task2(r, data, out);
        out.close();

        std::cout << "[完成] " << folder << " -> result_t2.md"
                  << "  总不满意度 = " << r.total_dissatisfaction << "\n";
    }

    return 0;
}