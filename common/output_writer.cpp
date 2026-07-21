#include "output_writer.h"
#include "dijkstra.h"
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

namespace {

std::string json_escape(const std::string& s)
{
    std::ostringstream oss;
    for (char ch : s)
    {
        unsigned char uch = static_cast<unsigned char>(ch);
        switch (ch)
        {
        case '\\': oss << "\\\\"; break;
        case '"': oss << "\\\""; break;
        case '\b': oss << "\\b"; break;
        case '\f': oss << "\\f"; break;
        case '\n': oss << "\\n"; break;
        case '\r': oss << "\\r"; break;
        case '\t': oss << "\\t"; break;
        default:
            if (uch < 0x20 || uch > 0x7e)
            {
                oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                    << static_cast<int>(uch) << std::dec << std::setfill(' ');
            }
            else
            {
                oss << ch;
            }
            break;
        }
    }
    return oss.str();
}

const char* pick_color(size_t idx)
{
    static const char* palette[] = {
        "#e6194b", "#3cb44b", "#ffe119", "#4363d8", "#f58231",
        "#911eb4", "#46f0f0", "#f032e6", "#bcf60c", "#fabebe",
        "#008080", "#e6beff", "#9a6324", "#fffac8", "#800000",
        "#aaffc3", "#808000", "#ffd8b1", "#000075", "#808080"
    };
    return palette[idx % (sizeof(palette) / sizeof(palette[0]))];
}

template <typename T>
void write_json_number(std::ofstream& out, T value)
{
    out << std::fixed << std::setprecision(6) << value;
}

} // namespace

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

void export_visualization_json(const input_data& data,
                               const std::vector<VisualizationRoute>& routes,
                               const std::string& output_file,
                               const std::string& title)
{
    std::ofstream out(to_path(output_file));
    if (!out)
        return;

    out << "{\n";
    out << "  \"meta\": {\n";
    out << "    \"title\": \"" << json_escape(title) << "\",\n";
    out << "    \"node_count\": " << data.g.get_n() << ",\n";
    out << "    \"edge_count\": ";

    std::set<std::pair<int, int>> unique_edges;
    for (const auto& adj_list : data.g.get_adj())
    {
        for (const auto& e : adj_list)
        {
            int u = std::min(e.u, e.v);
            int v = std::max(e.u, e.v);
            unique_edges.insert({u, v});
        }
    }
    out << unique_edges.size() << ",\n";
    out << "    \"route_count\": " << routes.size() << "\n";
    out << "  },\n";

    out << "  \"nodes\": [\n";
    const auto& nodes = data.g.get_nodes();
    for (size_t i = 0; i < nodes.size(); ++i)
    {
        const auto& n = nodes[i];
        out << "    {\"id\": " << n.id << ", \"x\": ";
        write_json_number(out, n.x);
        out << ", \"y\": ";
        write_json_number(out, n.y);
        out << ", \"is_station\": " << (n.is_station ? "true" : "false") << "}";
        if (i + 1 < nodes.size()) out << ",";
        out << "\n";
    }
    out << "  ],\n";

    out << "  \"edges\": [\n";
    size_t edge_idx = 0;
    for (const auto& uv : unique_edges)
    {
        int u = uv.first;
        int v = uv.second;
        double w = 0.0;
        for (const auto& e : data.g.get_adj()[u])
        {
            if (e.v == v)
            {
                w = e.w;
                break;
            }
        }
        out << "    {\"u\": " << u << ", \"v\": " << v << ", \"w\": ";
        write_json_number(out, w);
        out << "}";
        if (++edge_idx < unique_edges.size()) out << ",";
        out << "\n";
    }
    out << "  ],\n";

    out << "  \"routes\": [\n";
    for (size_t i = 0; i < routes.size(); ++i)
    {
        const auto& route = routes[i];
        std::string color = route.color.empty() ? pick_color(i) : route.color;
        std::string label = route.label.empty() ? ("route " + std::to_string(i + 1)) : route.label;

        out << "    {\"label\": \"" << json_escape(label) << "\",\n";
        out << "     \"color\": \"" << json_escape(color) << "\",\n";
        out << "     \"nodes\": [";
        for (size_t j = 0; j < route.nodes.size(); ++j)
        {
            if (j > 0) out << ", ";
            out << route.nodes[j];
        }
        out << "],\n";

        out << "     \"pkg_ids\": [";
        for (size_t j = 0; j < route.pkg_ids.size(); ++j)
        {
            if (j > 0) out << ", ";
            out << route.pkg_ids[j];
        }
        out << "],\n";

        out << "     \"depart_time\": ";
        if (route.depart_time < 0.0)
            out << "null";
        else
            write_json_number(out, route.depart_time);
        out << ",\n";

        out << "     \"end_time\": ";
        if (route.end_time < 0.0)
            out << "null";
        else
            write_json_number(out, route.end_time);
        out << "}";
        if (i + 1 < routes.size()) out << ",";
        out << "\n";
    }
    out << "  ]\n";
    out << "}\n";
}

} // namespace output
