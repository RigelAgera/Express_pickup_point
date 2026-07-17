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

} // namespace output