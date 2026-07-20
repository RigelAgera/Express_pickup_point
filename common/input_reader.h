// 读文件
#pragma once
#include "models.h"
#include "graph.h"
#include <filesystem>
#include <string>
#include <vector>

struct input_data
{
    graph g;
    std::vector<package> packages;
    car c;
};

// 将编码未知的字符串转为 filesystem::path（自动检测 UTF-8 或系统编码）
std::filesystem::path to_path(const std::string& s);

// 读取一个数据文件夹（含 map.txt, packages.txt, car.txt），返回完整输入
input_data read_input(const std::string& folder_path);
