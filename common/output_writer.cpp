#include "output_writer.h"
#include "dijkstra.h"
#include <fstream>
#include <iomanip>

namespace output {

void format_md(const std::string& folder, std::ofstream& out)
{
    out << "# " << folder << " 最短路结果\n\n";
}

void shortestPath(const Task1Result& r, std::ofstream& out)
{
    out << "## 一、最短路（Dijkstra）\n\n";
    out << "从驿站（节点 0）到各节点的 **最短距离**：\n\n";

    // 表格头
    out << "| 目标节点 |";
    for (int i = 0; i < r.n; ++i)
        out << " " << i << " |";
    out << "\n";

    out << "|";
    for (int i = 0; i <= r.n; ++i)
        out << "---|";
    out << "\n";

    // 距离行
    out << "| 最短距离 |";
    for (int i = 0; i < r.n; ++i)
        out << " " << r.dist[i] << " |";
    out << "\n\n";

    // 每条最短路径回溯
    out << "各节点最短路径：\n\n";
    for (int dst = 1; dst < r.n; ++dst)
    {
        std::vector<int> path = get_path(r.prev, 0, dst);
        if (path.empty())
        {
            out << "- `0 → " << dst << "`：不可达\n";
            continue;
        }
        out << "- `0 → " << dst << "`：";
        for (size_t i = 0; i < path.size(); ++i)
        {
            if (i > 0) out << " → ";
            out << path[i];
        }
        out << "，距离 **" << r.dist[dst] << "**\n";
    }
}

void task2(const Task2Result& r, const input_data& data, std::ofstream& out)
{
    out << "## 二、T2 最小化不满意度之和（模拟退火）\n\n";

    out << "**总不满意度之和**：**" << std::fixed << std::setprecision(4)
        << r.total_dissatisfaction << "**\n\n";

    out << "共 **" << r.trips.size() << "** 趟：\n\n";

    for (size_t t_idx = 0; t_idx < r.trips.size(); ++t_idx)
    {
        const auto& trip = r.trips[t_idx];
        out << "### 第 " << (t_idx + 1) << " 趟\n\n";
        out << "- 出发时刻：**" << std::fixed << std::setprecision(4) << trip.depart_time << "**\n";
        out << "- 返回时刻：**" << std::fixed << std::setprecision(4) << trip.end_time << "**\n";
        out << "- 路线：";
        for (size_t s = 0; s < trip.route.size(); ++s)
        {
            if (s > 0) out << " → ";
            out << trip.route[s];
        }
        out << "\n";
        out << "- 配送包裹：\n\n";

        out << "| 包裹ID | 目的地 | 重量 | Si | Di | 不满意度 |\n";
        out << "|--------|--------|------|----|----|----------|\n";
        for (size_t pi = 0; pi < trip.pkg_ids.size(); ++pi)
        {
            int pid = trip.pkg_ids[pi];
            const auto& p = data.packages[pid];
            out << "| " << pid + 1
                << " | " << p.dest
                << " | " << std::fixed << std::setprecision(1) << p.weight
                << " | " << std::fixed << std::setprecision(4) << p.arrive
                << " | " << std::fixed << std::setprecision(4) << trip.deliver_time[pi]
                << " | " << std::fixed << std::setprecision(4) << (trip.deliver_time[pi] - p.arrive)
                << " |\n";
        }
        out << "\n";
    }
}

} // namespace output
