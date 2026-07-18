// 最简单的写法应该是直接复用T3逻辑。只在时间计算上调整
// 其实直觉上感觉可以二分图，一辆送一边的，似乎会更快？一方如果提前送完就去另一边帮忙
// 还是要去查一下怎么写
#pragma once
#include "models.h"
#include "graph.h"
#include "schedule_utils.h"
#include "../common/input_reader.h"
#include <vector>

using std::vector;

struct Task5Result  // 用于传答案，最后构造就行
{
    vector<Trip> trips1;
    vector<Trip> trips2;
    int sumCost; // 总运送成本，和T3不同，要求总成本尽量小
    int exTime; // 超时包裹数，仅作统计目的
};

class Task5
{
    private:
        graph g;
        vector<package> pkgs;
        car c1, c2;
        vector<Trip> trips1;
        vector<Trip> trips2;
        AllPairResult ap;

    public:
        // 构造函数
        Task5(const input_data& data) : 
        g(data.g),pkgs(data.packages), c1(data.c),c2(data.c), ap(all_pairs_dijkstra(data.g)){}

        Task5Result solve();

        // 计算两辆车的总成本
        double compute_cost();

};