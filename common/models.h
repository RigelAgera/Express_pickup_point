// 小车和包裹两个对象的定义
// 后续可能需要在 `Package` 里加 `delivered_time`（送达时间 `D_i`）等运行时字段，可以到时候再扩展。
#pragma once
#include <vector>


struct package
{
    int id, dest;
    //数据格式说明里提到是实数，何意味，暂时用double吧
    double weight, arrive, deadline;
};

struct car
{
    // 单个包裹不会超过capacity
    double speed, car_weight, capacity;

};

// 一趟配送的描述
struct Trip
{
    std::vector<int> pkg_ids;        // 本趟携带的包裹 id
    std::vector<int> route;          // 行驶路线（节点序列，首尾都是 0）
    std::vector<double> segment_len; // 每段路的长度
    double depart_time;              // 出发时刻
    double end_time;                 // 回到驿站的时刻
    std::vector<double> deliver_time;// 每个包裹的送达时刻
};

// 全源最短路：dist[i][j] = i→j 最短距离，path[i][j] = 途经节点序列
struct AllPairResult
{
    std::vector<std::vector<double>> dist;
    std::vector<std::vector<std::vector<int>>> path;
};