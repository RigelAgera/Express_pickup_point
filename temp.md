# 项目结构想法

- 完全可以再调整，这算是一个大致版

Express_pickup_point/
├─ common/                         # 公共基础层，所有任务共用
│  ├─ models.h                     # 基础数据结构：package、car 等
│  ├─ graph.h                      # 图结构：节点、边、邻接表
│  ├─ input_reader.h               # 输入接口声明：读取地图/包裹/车辆数据
│  ├─ input_reader.cpp             # 输入实现：按数据格式说明解析文件
│  ├─ min_heap.h                   # 手写最小堆，供 Dijkstra 使用
│  ├─ circular_queue.h             # 手写队列，满足题目要求/供调度使用
│  ├─ dijkstra.h                   # Dijkstra 最短路 + 路径恢复
│  └─ schedule_utils.h             # 调度辅助函数：全源最短路、路线拼接、trip 模拟、成本计算等，AI写的，完全不在我预料之内，我甚至不知道这到底做了什么
│
├─ task/                           # 各任务逻辑层，就是对应项目说明里的6个T
│  ├─ task1.cpp                    # T1：最短路查询（本质围绕 dijkstra）
│  ├─ task2.cpp                    # T2：最小化不满意度之和
│  ├─ task3.cpp                    # T3：带容量的运送成本 + 超时统计
│  ├─ task4.cpp                    # T4：退货回收（TSP）
│  ├─ task5.cpp                    # T5：双车协同
│  └─ task6.cpp                    # T6：策略对比实验
│
├─ main_required.cpp               # 计划中的必做主程序入口：只负责 T1~T3
├─ main_bonus.cpp                  # 计划中的选做主程序入口：负责 T4~T6 / 可视化，两个主程序分开我是想着比较方便修bug，至少必做部分一定能通过
│
├─ 测试数据/                        # 课程给的数据，作业包里的
│  ├─ 示例/
│  ├─ 测试1/
│  ├─ 测试2/
│  └─ 测试3/
│
├─ Report/                         # 报告相关，目前写了零个字
│  └─ Report.md
│
├─ AI_USAGE/                       # AI 使用记录，这个感觉我们可以最后合并，我是按照校园导航的那个格式做的
│  └─ AI_USAGE.md
│
├─ 项目说明.md                     # 题目说明，作业包里的
├─ 数据格式说明.md                 # 输入数据格式说明，作业包里的
└─ README.md                       # 项目说明/使用说明，作业包里的
