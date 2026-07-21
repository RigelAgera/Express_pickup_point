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
#include "common/dijkstra.h"
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
    bool export_viz = false;
    double t5_overtime_penalty = 0.0;

    // 解析命令行参数
    int i = 1;
    while (i < argc)
    {
        std::string arg = argv[i];
        if (arg == "--viz" || arg == "-viz")
        {
            export_viz = true;
        }
        else if (arg == "--no-viz")
        {
            export_viz = false;
        }
        else if (arg.rfind("--t5-penalty=", 0) == 0)
        {
            std::string v = arg.substr(std::string("--t5-penalty=").size());
            try
            {
                t5_overtime_penalty = std::stod(v);
            }
            catch (...)
            {
                std::cerr << "[警告] 无法解析 --t5-penalty 参数，使用默认值 0.0\n";
                t5_overtime_penalty = 0.0;
            }
        }
        else if (arg == "--t5-penalty")
        {
            if (i + 1 < argc)
            {
                try
                {
                    t5_overtime_penalty = std::stod(argv[++i]);
                }
                catch (...)
                {
                    std::cerr << "[警告] 无法解析 --t5-penalty 参数，使用默认值 0.0\n";
                    t5_overtime_penalty = 0.0;
                }
            }
            else
            {
                std::cerr << "[警告] --t5-penalty 缺少数值，使用默认值 0.0\n";
                t5_overtime_penalty = 0.0;
            }
        }
        if (arg.rfind("-t", 0) == 0)        // 以 "-t" 开头
        {
            tasks = parse_tasks(arg);
        }
        else if (arg != "--viz" && arg != "-viz" && arg != "--no-viz" &&
                 arg != "--t5-penalty" && arg.rfind("--t5-penalty=", 0) != 0)
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
        std::vector<output::VisualizationRoute> viz_routes;

        std::ofstream out(to_path(folder + "/result.md"));
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

            if (export_viz)
            {
                for (int dst = 1; dst < r.n; ++dst)
                {
                    auto path = get_path(r.prev, 0, dst);
                    if (path.empty())
                        continue;

                    output::VisualizationRoute route;
                    route.nodes = std::move(path);
                    route.label = "T1 path to " + std::to_string(dst);
                    route.color = "#1f77b4";
                    viz_routes.push_back(std::move(route));
                }
            }
        }

        // T2 —— deadline 设为 1e9（忽略超时约束）
        if (tasks.count(2))
        {
            input_data d2 = data;
            for (auto& p : d2.packages) p.deadline = 1e9;
            Task2 solver(d2);
            auto r = solver.solve();
            output::task2(r, d2, out);

            if (export_viz)
            {
                for (size_t ti = 0; ti < r.trips.size(); ++ti)
                {
                    const auto& trip = r.trips[ti];
                    output::VisualizationRoute route;
                    route.nodes = trip.route;
                    route.label = "T2 trip " + std::to_string(ti + 1);
                    route.pkg_ids = trip.pkg_ids;
                    route.depart_time = trip.depart_time;
                    route.end_time = trip.end_time;
                    viz_routes.push_back(std::move(route));
                }
            }
        }

        // T3 —— arrive 设为 0（任何包裹可立即发车）
        if (tasks.count(3))
        {
            input_data d3 = data;
            for (auto& p : d3.packages) p.arrive = 0.0;
            Task3 solver(d3);
            auto r = solver.solve();
            output::task3(r, d3, out);

            if (export_viz)
            {
                for (size_t ti = 0; ti < r.trips.size(); ++ti)
                {
                    const auto& trip = r.trips[ti];
                    output::VisualizationRoute route;
                    route.nodes = trip.route;
                    route.label = "T3 trip " + std::to_string(ti + 1);
                    route.pkg_ids = trip.pkg_ids;
                    route.depart_time = trip.depart_time;
                    route.end_time = trip.end_time;
                    viz_routes.push_back(std::move(route));
                }
            }
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
            output::task4(r, data, out);

            if (export_viz && !r.destination.empty())
            {
                output::VisualizationRoute route;
                route.label = "T4 tour";
                route.color = "#d62728";
                route.nodes.push_back(0);
                route.nodes.insert(route.nodes.end(), r.destination.begin(), r.destination.end());
                route.nodes.push_back(0);
                viz_routes.push_back(std::move(route));
            }
        }

        // T5 —— 双车协同配送（arrive 清零，包裹即时可用）
        if (tasks.count(5))
        {
            input_data d5 = data;
            for (auto& p : d5.packages) p.arrive = 0.0;
            Task5 solver(d5, t5_overtime_penalty);
            auto r = solver.solve();
            output::task5(r, d5, out);

            if (export_viz)
            {
                for (size_t ti = 0; ti < r.trips1.size(); ++ti)
                {
                    const auto& trip = r.trips1[ti];
                    output::VisualizationRoute route;
                    route.nodes = trip.route;
                    route.label = "T5 car1 trip " + std::to_string(ti + 1);
                    route.pkg_ids = trip.pkg_ids;
                    route.depart_time = trip.depart_time;
                    route.end_time = trip.end_time;
                    route.color = "#2ca02c";
                    viz_routes.push_back(std::move(route));
                }
                for (size_t ti = 0; ti < r.trips2.size(); ++ti)
                {
                    const auto& trip = r.trips2[ti];
                    output::VisualizationRoute route;
                    route.nodes = trip.route;
                    route.label = "T5 car2 trip " + std::to_string(ti + 1);
                    route.pkg_ids = trip.pkg_ids;
                    route.depart_time = trip.depart_time;
                    route.end_time = trip.end_time;
                    route.color = "#ff7f0e";
                    viz_routes.push_back(std::move(route));
                }
            }
        }

        out.close();
        if (export_viz)
        {
            output::export_visualization_json(data,
                                              viz_routes,
                                              folder + "/result_viz.json",
                                              folder);
        }
        std::cout << "[完成] " << folder << " -> result.md\n";
        if (export_viz)
            std::cout << "[完成] " << folder << " -> result_viz.json\n";
    }

    return 0;
}