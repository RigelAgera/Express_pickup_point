// 读文件
#include "input_reader.h"
#ifdef _WIN32
  #include <windows.h>
#endif
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace {

// 检测 string 是否为合法 UTF-8 序列
bool is_utf8(const std::string& s)
{
    const unsigned char* p = reinterpret_cast<const unsigned char*>(s.data());
    const unsigned char* end = p + s.size();
    while (p < end)
    {
        unsigned char c = *p;
        if (c <= 0x7F) { ++p; continue; }                         // ASCII
        int len = 0;
        if      ((c & 0xE0) == 0xC0) len = 2;                     // 110xxxxx
        else if ((c & 0xF0) == 0xE0) len = 3;                     // 1110xxxx
        else if ((c & 0xF8) == 0xF0) len = 4;                     // 11110xxx
        else return false;
        if (p + len > end) return false;
        for (int i = 1; i < len; ++i)
            if ((p[i] & 0xC0) != 0x80) return false;              // 续字节必须是 10xxxxxx
        p += len;
    }
    return true;
}

} // anonymous

// 将 string 转为 filesystem::path，自动适配 UTF-8 / 系统代码页
std::filesystem::path to_path(const std::string& s)
{
    if (is_utf8(s))
        return std::filesystem::u8path(s);

#ifdef _WIN32
    // 按系统 ANSI 代码页（GBK 等）转宽字符
    int wlen = MultiByteToWideChar(CP_ACP, 0, s.c_str(), -1, nullptr, 0);
    if (wlen <= 0) throw std::runtime_error("路径编码转换失败: " + s);
    std::wstring ws(wlen, L'\0');
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), -1, &ws[0], wlen);
    // 去掉末尾 null
    if (!ws.empty() && ws.back() == L'\0') ws.pop_back();
    return std::filesystem::path(ws);
#else
    return std::filesystem::path(s);
#endif
}

input_data read_input(const std::string& folder_path)
{
    input_data data;
    namespace fs = std::filesystem;

    // 1. 读 map.txt
    {
        fs::path fp = to_path(folder_path) / "map.txt";
        std::ifstream f(fp);
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
        fs::path fp = to_path(folder_path) / "packages.txt";
        std::ifstream f(fp);
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
        fs::path fp = to_path(folder_path) / "car.txt";
        std::ifstream f(fp);
        if (!f.is_open())
            throw std::runtime_error("无法打开 " + folder_path + "/car.txt");

        f >> data.c.speed
          >> data.c.car_weight
          >> data.c.capacity;
    }

    return data;
}