// 主程序（必做）：T1 ~ T3
#include "common/input_reader.h"
#include "common/output_writer.h"
#include "task/task1.h"
// #include "task/task2.h"
// #include "task/task3.h"
#include <iostream>
#include <fstream>

int main(int argc, char* argv[])
{
    std::string folder = (argc >= 2) ? argv[1] : "测试数据/示例";

    input_data data = read_input(folder);

    std::ofstream out(folder + "/result.md");
    if (!out)
    {
        std::cerr << "无法创建 result.md\n";
        return 1;
    }

    output::format_md(folder, out);

    // T1
    {
        Task1Result r = Task1::solve(data);
        output::shortestPath(r, out);
    }

    out.close();

    // T2（待 task2.h/task2.cpp 改造后取消注释）
    // {
    //     auto r = Task2::solve(data);
    //     std::cout << Task2::format(r, folder) << "\n";
    // }

    // T3（待 task3.h/task3.cpp 改造后取消注释）
    // {
    //     auto r = Task3::solve(data);
    //     std::cout << Task3::format(r, folder) << "\n";
    // }

    return 0;
}
