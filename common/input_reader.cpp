#include "input_reader.h"
#include <fstream>
#include <stdexcept>

input_data read_input(const std::string& folder_path)
{
    input_data data;

    // 1. 读 map.txt
    {
        std::ifstream f(folder_path + "/map.txt");
        if (!f.is_open())
            throw std::runtime_error("无法打开 " + folder_path + "/map.txt");

        int n, m;
        f >> n >> m;
        data.g = graph(n, m);

        for (int i = 0; i < n; ++i)
        {
            int id, is_station;
            double x, y;
            f >> id >> x >> y >> is_station;
            data.g.set_node(id, x, y, is_station != 0);
        }

        for (int i = 0; i < m; ++i)
        {
            int u, v;
            double w;
            f >> u >> v >> w;
            data.g.add_edges(u, v, w);
        }
    }

    // 2. 读 packages.txt
    {
        std::ifstream f(folder_path + "/packages.txt");
        if (!f.is_open())
            throw std::runtime_error("无法打开 " + folder_path + "/packages.txt");

        int k;
        f >> k;
        data.packages.resize(k);
        for (int i = 0; i < k; ++i)
        {
            f >> data.packages[i].id
              >> data.packages[i].weight
              >> data.packages[i].dest
              >> data.packages[i].arrive
              >> data.packages[i].deadline;
        }
    }

    // 3. 读 car.txt
    {
        std::ifstream f(folder_path + "/car.txt");
        if (!f.is_open())
            throw std::runtime_error("无法打开 " + folder_path + "/car.txt");

        f >> data.c.speed
          >> data.c.car_weight
          >> data.c.capacity;
    }

    return data;
}