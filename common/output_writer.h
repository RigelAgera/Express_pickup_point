// 输出类，因为没有确定最后到底用什么格式所以单独一个类
#pragma once
#include "task/task1.h"
#include <string>
#include <fstream>

namespace output {

    void format_md(const std::string& folder, std::ofstream& out);
    void shortestPath(const Task1Result& r, std::ofstream& out);
    // 将来：void task2_xxx(...);
}
