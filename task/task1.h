#pragma once
#include "../common/input_reader.h"
#include "../common/dijkstra.h"
#include <string>
#include <vector>

// T1 结果
struct Task1Result
{
    std::vector<double> dist;   // 从驿站 0 到各节点的最短距离
    std::vector<int> prev;      // 最短路径前驱
    int n;                      // 节点数
};

class Task1
{
public:
    // 执行 Dijkstra，返回结果数据
    static Task1Result solve(const input_data& data);

};