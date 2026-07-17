// 主程序（必做）：T1 ~ T3
#include "common/input_reader.h"
#include "common/output_writer.h"
#include "task/task1.h"
#include "task/task2.h"
// #include "task/task3.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <set>

// 解析 -t 选项，返回要运行的 task 编号集合
std::set<int> parse_tasks(const std::string& arg)
{
    // arg 形如 "-t1" / "-t1,2" / "-t1,2,3"
    std::set<int> tasks;
    std::string s = arg.substr(2);           // 去掉 "-t"
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        int t = std::stoi(token);
        if (t >= 1 && t <= 3) tasks.insert(t);
    }
    return tasks;
}

int main(int argc, char* argv[])
{
    // 默认值
    std::set<int> tasks = { 1, 2, 3 };
    std::vector<std::string> folders;

    // 解析命令行参数
    int i = 1;
    while (i < argc)
    {
        std::string arg = argv[i];
        if (arg.rfind("-t", 0) == 0)        // 以 "-t" 开头
        {
            tasks = parse_tasks(arg);
        }
        else
        {
            folders.push_back(arg);
        }
        ++i;
    }

    if (folders.empty())
        folders.push_back("测试数据/示例");

    // 对每个数据文件夹执行选定 task
    for (const auto& folder : folders)
    {
        input_data data = read_input(folder);

        std::ofstream out(folder + "/result.md");
        if (!out)
        {
            std::cerr << "无法创建 " << folder << "/result.md\n";
            continue;
        }

        output::format_md(folder, out);

        // T1 —— 原始数据
        if (tasks.count(1))
        {
#ifdef TASK1_ENABLED
            Task1Result r = Task1::solve(data); 
            output::shortestPath(r, out);
#else
            std::cerr << "[跳过] T1: task1.h 未包含\n";
#endif
        }

        // T2 —— deadline 设为 1e9
        if (tasks.count(2))
        {
            input_data d2 = data;
            for (auto& p : d2.packages) p.deadline = 1e9;
            auto r = Task2::solve(d2);
            output::task2(r, d2, out);
        }

        // T3 —— arrive 设为 0
        if (tasks.count(3))
        {
#ifdef TASK3_ENABLED
            input_data d3 = data;
            for (auto& p : d3.packages) p.arrive = 0.0;
            auto r = Task3::solve(d3);
            // TODO: output::task3(r, out);
            std::cerr << "[提示] T3 输出函数待实现\n";
#else
            std::cerr << "[跳过] T3: task3.h 未包含\n";
#endif
        }

        out.close();
    }

    return 0;
}