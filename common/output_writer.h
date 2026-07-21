// 输出类，因为没有确定最后到底用什么格式所以单独一个类
#pragma once
#include "../task/task1.h"
#include "../task/task2.h"
#include "../task/task3.h"
#include "../task/task4.h"
#include "../task/task5.h"
#include "../task/task2_simple.h"
#include "../task/task3_simple.h"
#include "../task/task4_simple.h"
#include "../task/task5_simple.h"
#include "input_reader.h"
#include <string>
#include <vector>
#include <fstream>

namespace output {

    struct VisualizationRoute
    {
        std::vector<int> nodes;
        std::string label;
        std::string color;
        std::vector<int> pkg_ids;
        double depart_time = -1.0;
        double end_time = -1.0;
    };

    void format_md(const std::string& folder, std::ofstream& out);
    void shortestPath(const Task1Result& r, std::ofstream& out);
    void task2(const Task2Result& r, const input_data& data, std::ofstream& out);
    void task3(const Task3Result& r, const input_data& data, std::ofstream& out);
    void task4(const Task4Result& r, const input_data& data, std::ofstream& out);
    void task5(const Task5Result& r, const input_data& data, std::ofstream& out);
    void task2_simple(const Task2SimpleResult& r, const input_data& data, std::ofstream& out);
    void task3_simple(const Task3SimpleResult& r, const input_data& data, std::ofstream& out);
    void task4_simple(const Task4SimpleResult& r, const input_data& data, std::ofstream& out);
    void task5_simple(const Task5SimpleResult& r, const input_data& data, std::ofstream& out);
    void export_visualization_json(const input_data& data,
                                   const std::vector<VisualizationRoute>& routes,
                                   const std::string& output_file,
                                   const std::string& title = "");
}
