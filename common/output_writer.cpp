#include "output_writer.h"
#include "dijkstra.h"
#include <fstream>
#include <iomanip>

namespace output {

namespace {

template <typename TResult>
void write_task2_impl(const TResult& r, const input_data& data, std::ofstream& out)
{
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

template <typename TResult>
void write_task3_impl(const TResult& r, const input_data& data, std::ofstream& out)
{
    out << "- 总运送成本：**" << std::fixed << std::setprecision(4) << r.sumCost << "**\n";
    out << "- 超时包裹数：**" << r.exTime << "**\n";
    out << "- 趟数：**" << r.trips.size() << "**\n\n";

    for (size_t t_idx = 0; t_idx < r.trips.size(); ++t_idx)
    {
        const auto& trip = r.trips[t_idx];
        out << "### 第 " << (t_idx + 1) << " 趟\n\n";
        out << "- 出发时刻：**" << std::fixed << std::setprecision(4) << trip.depart_time << "**\n";
        out << "- 返回时刻：**" << std::fixed << std::setprecision(4) << trip.end_time << "**\n";
        out << "- 路线：";
        for (size_t i = 0; i < trip.route.size(); ++i)
        {
            if (i > 0) out << " → ";
            out << trip.route[i];
        }
        out << "\n";
        out << "- 配送包裹：\n\n";
        out << "| 包裹ID | 目的地 | 重量 | Deadline | 送达时刻 | 是否超时 |\n";
        out << "|--------|--------|------|----------|----------|----------|\n";

        for (size_t pi = 0; pi < trip.pkg_ids.size(); ++pi)
        {
            int pid = trip.pkg_ids[pi];
            const auto& p = data.packages[pid];
            double deliver = (pi < trip.deliver_time.size()) ? trip.deliver_time[pi] : -1.0;
            bool late = deliver > p.deadline + 1e-9;
            out << "| " << pid + 1
                << " | " << p.dest
                << " | " << std::fixed << std::setprecision(1) << p.weight
                << " | " << std::fixed << std::setprecision(4) << p.deadline
                << " | " << std::fixed << std::setprecision(4) << deliver
                << " | " << (late ? "是" : "否")
                << " |\n";
        }
        out << "\n";
    }
}

template <typename TResult>
void write_task4_impl(const TResult& r, const input_data& data, std::ofstream& out)
{
    if (r.destination.empty())
    {
        out << "- 回收点数量：**0**\n";
        out << "- 访问顺序（不含 0）：无\n\n";
        return;
    }

    AllPairResult ap = all_pairs_dijkstra(data.g);
    double total_len = ap.dist[0][r.destination[0]];
    for (size_t i = 0; i + 1 < r.destination.size(); ++i)
        total_len += ap.dist[r.destination[i]][r.destination[i + 1]];
    total_len += ap.dist[r.destination.back()][0];

    out << "- 回收点数量：**" << r.destination.size() << "**\n";
    out << "- 访问顺序（不含 0）：";
    for (size_t i = 0; i < r.destination.size(); ++i)
    {
        if (i > 0) out << " → ";
        out << r.destination[i];
    }
    out << "\n";
    out << "- 回路总距离（0 出发并回到 0）：**" << std::fixed << std::setprecision(4)
        << total_len << "**\n\n";
}

template <typename TResult>
void write_task5_impl(const TResult& r, const input_data& data, std::ofstream& out)
{
    out << "- 总运送成本：**" << std::fixed << std::setprecision(4) << r.sumCost << "**\n";
    out << "- 超时包裹数：**" << r.exTime << "**\n";
    out << "- 车1趟数：**" << r.trips1.size() << "**\n";
    out << "- 车2趟数：**" << r.trips2.size() << "**\n\n";

    auto write_car = [&](const std::vector<Trip>& trips, int car_idx)
    {
        out << "### 车" << car_idx << " 配送详情\n\n";
        if (trips.empty())
        {
            out << "- 无配送趟次\n\n";
            return;
        }

        for (size_t t_idx = 0; t_idx < trips.size(); ++t_idx)
        {
            const auto& trip = trips[t_idx];
            out << "#### 第 " << (t_idx + 1) << " 趟\n\n";
            out << "- 出发时刻：**" << std::fixed << std::setprecision(4) << trip.depart_time << "**\n";
            out << "- 返回时刻：**" << std::fixed << std::setprecision(4) << trip.end_time << "**\n";
            out << "- 路线：";
            for (size_t i = 0; i < trip.route.size(); ++i)
            {
                if (i > 0) out << " → ";
                out << trip.route[i];
            }
            out << "\n";
            out << "- 配送包裹：\n\n";
            out << "| 包裹ID | 目的地 | 重量 | Deadline | 送达时刻 | 是否超时 |\n";
            out << "|--------|--------|------|----------|----------|----------|\n";

            for (size_t pi = 0; pi < trip.pkg_ids.size(); ++pi)
            {
                int pid = trip.pkg_ids[pi];
                const auto& p = data.packages[pid];
                double deliver = (pi < trip.deliver_time.size()) ? trip.deliver_time[pi] : -1.0;
                bool late = deliver > p.deadline + 1e-9;
                out << "| " << pid + 1
                    << " | " << p.dest
                    << " | " << std::fixed << std::setprecision(1) << p.weight
                    << " | " << std::fixed << std::setprecision(4) << p.deadline
                    << " | " << std::fixed << std::setprecision(4) << deliver
                    << " | " << (late ? "是" : "否")
                    << " |\n";
            }
            out << "\n";
        }
    };

    write_car(r.trips1, 1);
    write_car(r.trips2, 2);
}

} // namespace

void format_md(const std::string& folder, std::ofstream& out)
{
    out << "# " << folder << " 测试结果\n\n";   // 这里好像有乱码问题
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
    write_task2_impl(r, data, out);
}

void task3(const Task3Result& r, const input_data& data, std::ofstream& out)
{
    out << "\n## 三、T3 带容量运送成本\n\n";
    write_task3_impl(r, data, out);
}

void task4(const Task4Result& r, const input_data& data, std::ofstream& out)
{
    out << "\n## 四、T4 退货回收（GA 近似）\n\n";
    write_task4_impl(r, data, out);
}

void task5(const Task5Result& r, const input_data& data, std::ofstream& out)
{
    out << "\n## 五、T5 双车协同配送\n\n";
    write_task5_impl(r, data, out);
}

void task2_simple(const Task2SimpleResult& r, const input_data& data, std::ofstream& out)
{
    out << "## 二、T2 最小化不满意度之和（纯贪心简化版）\n\n";
    write_task2_impl(r, data, out);
}

void task3_simple(const Task3SimpleResult& r, const input_data& data, std::ofstream& out)
{
    out << "\n## 三、T3 带容量运送成本（EDD纯贪心简化版）\n\n";
    write_task3_impl(r, data, out);
}

void task4_simple(const Task4SimpleResult& r, const input_data& data, std::ofstream& out)
{
    out << "\n## 四、T4 退货回收（最近邻贪心简化版）\n\n";
    write_task4_impl(r, data, out);
}

void task5_simple(const Task5SimpleResult& r, const input_data& data, std::ofstream& out)
{
    out << "\n## 五、T5 双车协同配送（K-means+贪心简化版）\n\n";
    write_task5_impl(r, data, out);
}

} // namespace output
