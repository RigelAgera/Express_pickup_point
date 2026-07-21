# Express Pickup Point

这是一个用于“菜鸟驿站配送调度系统”的 C++ 项目。仓库里目前有几个主程序入口，分别对应不同的任务范围和算法版本。

## 主程序说明

- `main_completed.cpp`：完整版本入口，处理 T1 ~ T5 的正式实现。支持 `--viz` 导出可视化数据。
- `main_simple.cpp`：简化版入口，处理 T1 ~ T5 的简化算法，输出到 `result_simple.md`。

## 编译环境

建议使用支持 C++17 的 GCC / MinGW-w64

## 编译命令

下面命令都假设当前目录是仓库根目录 `D:\Express_pickup_point`。

### 1. 编译完整版本 `main_completed.cpp`

```powershell
g++ -std=c++17 -O2 `
main_completed.cpp `
common/input_reader.cpp `
common/output_writer.cpp `
common/schedule_utils.cpp `
task/task1.cpp `
task/task2.cpp `
task/task3.cpp `
task/task4.cpp `
task/task5.cpp `
-o main_completed.exe
```

使用可视化模块需要确保 Python 环境里已安装 `matplotlib`，然后用仓库根目录下的 `visualize_routes.py` 读取导出的 `result_viz.json`。

### 2. 编译简化版 `main_simple.cpp`

```powershell
g++ -std=c++17 -O2 `
main_simple.cpp `
common/input_reader.cpp `
common/output_writer.cpp `
common/schedule_utils.cpp `
task/task1.cpp `
task/task2_simple.cpp `
task/task3_simple.cpp `
task/task4_simple.cpp `
task/task5_simple.cpp `
-o main_simple.exe
```

## 运行方式

主程序都支持把测试数据目录作为命令行参数。不给参数时，默认读取 `测试数据/示例`。

### `main_completed.exe`

运行全部任务：

```powershell
.\main_completed.exe
```

只运行指定任务，例如只跑 T1 和 T4：

```powershell
.\main_completed.exe -t1,4
```

只运行 T5，并设置“超时惩罚系数”（默认是 `0.0`）：

```powershell
.\main_completed.exe -t5 --t5-penalty=10000
```

也支持用空格传参：

```powershell
.\main_completed.exe -t5 --t5-penalty 10000
```

同时导出可视化数据：

```powershell
.\main_completed.exe --viz
```

指定数据集并导出可视化数据：

```powershell
.\main_completed.exe --viz 测试数据/示例
```

指定多个数据集：

```powershell
.\main_completed.exe 测试数据/测试1 测试数据/测试2
```

### `main_simple.exe`

默认运行全部简化任务：

```powershell
.\main_simple.exe
```

只跑 T2、T3 的简化版：

```powershell
.\main_simple.exe -t2,3
```

## 输出文件

- `main_completed.exe` 会在数据目录下生成 `result.md`
- `main_completed.exe --viz` 会额外在数据目录下生成 `result_viz.json`
- `main_simple.exe` 会在数据目录下生成 `result_simple.md`

例如运行 `测试数据/示例` 后，结果会写到对应目录下的 `result.md` 或 `result_simple.md`。如果加上 `--viz`，还会额外生成 `result_viz.json`，供 Python 画图脚本使用。

对于 T5，`--t5-penalty` 只影响求解评分（成本与超时的权衡），不会改变输出文件名。

## 可视化

先导出可视化数据：

```powershell
.\main_completed.exe --viz 测试数据/示例
```

再生成图片：

```powershell
D:/Programming/python_/python.exe visualize_routes.py 测试数据/示例/result_viz.json -o 测试数据/示例/result_viz.png
```

## 常用示例

```powershell
g++ -std=c++17 -O2 main_completed.cpp common/input_reader.cpp common/output_writer.cpp common/schedule_utils.cpp task/task1.cpp task/task2.cpp task/task3.cpp task/task4.cpp task/task5.cpp -o main_completed.exe
.\main_completed.exe --viz 测试数据/示例
```

```powershell
g++ -std=c++17 -O2 main_completed.cpp common/input_reader.cpp common/output_writer.cpp common/schedule_utils.cpp task/task1.cpp task/task2.cpp task/task3.cpp task/task4.cpp task/task5.cpp -o main_completed.exe
.\main_completed.exe -t5 --t5-penalty=10000 测试数据/示例
```

```powershell
g++ -std=c++17 -O2 main_simple.cpp common/input_reader.cpp common/output_writer.cpp common/schedule_utils.cpp task/task1.cpp task/task2_simple.cpp task/task3_simple.cpp task/task4_simple.cpp task/task5_simple.cpp -o main_simple.exe
.\main_simple.exe 测试数据/测试1
```
