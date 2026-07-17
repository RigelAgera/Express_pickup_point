// T1: Dijkstra 最短路查询

#include "../common/input_reader.h"
#include "../common/dijkstra.h"
#include "task1.h"
#include <sstream>
#include <iomanip>

Task1Result Task1::solve(const input_data& data)
{
    int n = data.g.get_n(); // 节点数量
    DijkResult res = dijkstra(data.g, 0); 
    return { res.dist, res.prev, n };
}
