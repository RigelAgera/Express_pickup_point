// 用于1-5的测试
// 从哪里来点数据，2-5用的算法都是大规模有优势，但给的样例数据量小
// 主程序（扩展）：T1 ~ T5
#include "common/input_reader.h"
#include "common/output_writer.h"
#include "task/task1.h"
#include "task/task2.h"
#include "task/task3.h"
#include "task/task4.h"
#include "task/task5.h"
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
    // arg 形如 "-t1" / "-t1,2" / "-t1,2,3,4,5"
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
    // 默认值
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

        std::ofstream out(folder + "/result.md");
        if (!out)
        {
            std::cerr << "无法创建 " << folder << "/result.md\n";
            continue;
        }

        output::format_md(folder, out);

        // T1 —— 原始数据最短路
        if (tasks.count(1))
        {
            Task1Result r = Task1::solve(data); 
            output::shortestPath(r, out);
        }

        // T2 —— deadline 设为 1e9（忽略超时约束）
        if (tasks.count(2))
        {
            input_data d2 = data;
            for (auto& p : d2.packages) p.deadline = 1e9;
            Task2 solver(d2);
            auto r = solver.solve();
            output::task2(r, d2, out);
        }

        // T3 —— arrive 设为 0（任何包裹可立即发车）
        if (tasks.count(3))
        {
            input_data d3 = data;
            for (auto& p : d3.packages) p.arrive = 0.0;
            Task3 solver(d3);
            auto r = solver.solve();

            // output_writer 暂无 task3 专用接口，这里直接输出核心指标
            out << "\n## 三、T3 带容量运送成本\n\n";
            out << "- 总运送成本：**" << r.sumCost << "**\n";
            out << "- 超时包裹数：**" << r.exTime << "**\n";
            out << "- 趟数：**" << r.trips.size() << "**\n\n";
        }

        // T4 —— 退货回收（默认使用包裹目的地去重集合作为回收点）
        if (tasks.count(4))
        {
            std::vector<int> return_nodes;
            return_nodes.reserve(data.packages.size());
            for (const auto& p : data.packages)
                return_nodes.push_back(p.dest);
            std::sort(return_nodes.begin(), return_nodes.end());
            return_nodes.erase(std::unique(return_nodes.begin(), return_nodes.end()), return_nodes.end());

            Task4 solver(data);
            auto r = solver.solve(return_nodes);

            out << "\n## 四、T4 退货回收（GA 近似）\n\n";
            out << "- 回收点数量：**" << return_nodes.size() << "**\n";
            out << "- 访问顺序（不含 0）：";
            if (r.destination.empty())
            {
                out << "无\n\n";
            }
            else
            {
                for (size_t i = 0; i < r.destination.size(); ++i)
                {
                    if (i) out << " -> ";
                    out << r.destination[i];
                }
                out << "\n\n";
            }
        }

        // T5 —— 双车协同配送
        if (tasks.count(5))
        {
            Task5 solver(data);
            auto r = solver.solve();

            out << "\n## 五、T5 双车协同配送\n\n";
            out << "- 总运送成本：**" << r.sumCost << "**\n";
            out << "- 超时包裹数：**" << r.exTime << "**\n";
            out << "- 车1趟数：**" << r.trips1.size() << "**\n";
            out << "- 车2趟数：**" << r.trips2.size() << "**\n\n";
        }

        out.close();
        std::cout << "[完成] " << folder << " -> result.md\n";
    }

    return 0;
}