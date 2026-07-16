// T1: Dijkstra 最短路查询
// 编译: g++ -std=c++17 -O2 task/task1.cpp common/input_reader.cpp -o task1.exe

#include "../common/input_reader.h"
#include "../common/dijkstra.h"
#include <iostream>
#include <iomanip>

int main(int argc, char* argv[])
{
    std::string folder = (argc >= 2) ? argv[1] : "测试数据/示例";

    auto data = read_input(folder);
    int n = data.g.get_n();

    // 从驿站 0 跑 Dijkstra
    auto res = dijkstra(data.g, 0);

    std::cout << "=== T1 最短路查询 ===\n";
    std::cout << "数据: " << folder << "\n\n";

    // 输出 0 → 各节点最短距离
    std::cout << "从驿站(0)到各节点的最短距离:\n";
    std::cout << "--------------------------------\n";
    std::cout << std::left << std::setw(10) << "目标节点" << "最短距离" << "\n";
    std::cout << "--------------------------------\n";
    for (int v = 0; v < n; ++v)
    {
        std::cout << std::left << std::setw(10) << v;
        if (res.dist[v] >= 1e18)
            std::cout << "不可达";
        else
            std::cout << res.dist[v];
        std::cout << "\n";
    }

    // 输出 0 → 各节点的具体路径
    std::cout << "\n从驿站(0)到各节点的最短路径:\n";
    std::cout << "--------------------------------\n";
    for (int v = 1; v < n; ++v)
    {
        auto path = get_path(res.prev, 0, v);
        std::cout << "0 -> " << v << ": ";
        if (path.empty())
        {
            std::cout << "不可达";
        }
        else
        {
            for (size_t i = 0; i < path.size(); ++i)
            {
                if (i > 0) std::cout << " -> ";
                std::cout << path[i];
            }
            std::cout << "   (距离 " << res.dist[v] << ")";
        }
        std::cout << "\n";
    }

    return 0;
}