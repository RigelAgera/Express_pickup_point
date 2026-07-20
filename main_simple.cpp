// 用于 T2 ~ T5 简化版（纯贪心/启发式）的测试
// 参考 main_other.cpp
// 主程序（简化版）：T1 ~ T5（T1 最短路无简化版，按原逻辑处理；T2 ~ T5 使用对应的 Simple 简化算法类）
#include "common/input_reader.h"
#include "common/output_writer.h"
#include "task/task1.h"
#include "task/task2_simple.h"
#include "task/task3_simple.h"
#include "task/task4_simple.h"
#include "task/task5_simple.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <set>
#include <algorithm>

// 解析 -t 选项，返回要运行的 task 编号集合
std::set<int> parse_tasks(const std::string& arg)
{
    // arg 形如 "-t2" / "-t2,3" / "-t1,2,3,4,5"
    std::set<int> tasks;
    std::string s = arg.substr(2);           // 去掉 "-t"
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        try
        {
            int t = std::stoi(token);
            if (t >= 1 && t <= 5) tasks.insert(t);
        }
        catch (...)
        {
            // 非法 token 直接忽略，避免整次运行因参数格式问题中断
        }
    }

    // 如果解析不到合法任务号，退回默认任务集合
    if (tasks.empty()) tasks = { 1, 2, 3, 4, 5 };
    return tasks;
}

int main(int argc, char* argv[])
{
    // 默认值：执行1~5（2~5为简化版）
    std::set<int> tasks = { 1, 2, 3, 4, 5 };
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

        // 输出为 result_simple.md 以便和正常完整版本的 result.md 进行对照
        std::string out_path = folder + "/result_simple.md";
        std::ofstream out(to_path(out_path));
        if (!out)
        {
            std::cerr << "无法创建 " << out_path << "\n";
            continue;
        }

        output::format_md(folder, out);

        // T1 —— 原始数据最短路（T1无简化算法，按标准最短路输出）
        if (tasks.count(1))
        {
            Task1Result r = Task1::solve(data); 
            output::shortestPath(r, out);
        }

        // T2简化版 —— 纯贪心(按到达时间分批装车) + nearest-first趟内排序
        if (tasks.count(2))
        {
            input_data d2 = data;
            for (auto& p : d2.packages) p.deadline = 1e9;
            Task2Simple solver(d2);
            auto r = solver.solve();
            output::task2_simple(r, d2, out);
        }

        // T3简化版 —— EDD贪心 + nearest-first趟内排序（arrive 设为 0）
        if (tasks.count(3))
        {
            input_data d3 = data;
            for (auto& p : d3.packages) p.arrive = 0.0;
            Task3Simple solver(d3);
            auto r = solver.solve();
            output::task3_simple(r, d3, out);
        }

        // T4简化版 —— 最近邻贪心求解 TSP
        if (tasks.count(4))
        {
            std::vector<int> return_nodes;
            return_nodes.reserve(data.packages.size());
            for (const auto& p : data.packages)
                return_nodes.push_back(p.dest);
            std::sort(return_nodes.begin(), return_nodes.end());
            return_nodes.erase(std::unique(return_nodes.begin(), return_nodes.end()), return_nodes.end());

            Task4Simple solver(data);
            auto r = solver.solve(return_nodes);
            output::task4_simple(r, data, out);
        }

        // T5简化版 —— K-means分区 + EDD贪心分趟 + nearest-first趟内排序（arrive 清零）
        if (tasks.count(5))
        {
            input_data d5 = data;
            for (auto& p : d5.packages) p.arrive = 0.0;
            Task5Simple solver(d5);
            auto r = solver.solve();
            output::task5_simple(r, d5, out);
        }

        out.close();
        std::cout << "[完成] " << folder << " -> result_simple.md\n";
    }

    return 0;
}
