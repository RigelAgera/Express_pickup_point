// 读文件
#pragma once
#include "models.h"
#include "graph.h"
#include <string>
#include <vector>

struct input_data
{
    graph g;
    std::vector<package> packages;
    car c;
};

// 读取一个数据文件夹（含 map.txt, packages.txt, car.txt），返回完整输入
input_data read_input(const std::string& folder_path);