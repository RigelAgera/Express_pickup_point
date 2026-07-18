// 标准TSP
// 退货点少（≤20 左右）用状态压缩 DP 求精确解，多了用最近邻 + 2-opt 等启发式。
#pragma once
#include "models.h"
#include "graph.h"
#include "schedule_utils.h"
#include "../common/input_reader.h"
#include <vector>

using std::vector;

struct Task4Result
{
    vector<int> destination; // 访问的地点的顺序
};

class Task4
{
    private:
        graph g;
        // vector<package> pkgs;   // 好像真的没有用
        double v;  // 只有最后算时间的时候速度有用
        AllPairResult ap;

    public:
        // 构造函数
        Task4(const input_data& data) : g(data.g), v(data.c.speed), ap(all_pairs_dijkstra(data.g)){}

};
