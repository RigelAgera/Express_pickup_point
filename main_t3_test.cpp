// T3 单独测试主程序
// 用法: main_t3_test [数据文件夹路径，默认"测试数据/示例"]
#include "common/input_reader.h"
#include "common/output_writer.h"
#include "task/task3.h"
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

        // T3 构造时已强制 S_i = 0，无需外部清零
        Task3 solver(data);
        auto r = solver.solve();

        // 输出到 result.md
        std::ofstream out(to_path(folder + "/result.md"));
        if (out)
        {
            output::format_md(folder, out);
            output::task3(r, data, out);
            out.close();
        }

        // 屏幕摘要
        std::cout << "[" << folder << "]\n";
        std::cout << "  总成本: " << r.sumCost << "\n";
        std::cout << "  超时数: " << r.exTime << "\n";
        std::cout << "  总趟数: " << r.trips.size() << "\n";
        std::cout << std::endl;
    }

    return 0;
}