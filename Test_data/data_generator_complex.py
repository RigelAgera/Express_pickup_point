# 复杂测试数据，用于拉开T5和T3差距
# 多中心 + 模糊重叠带
# 地理上极近，但时间上冲突的包裹
# 减轻包裹重量，增大可优化空间（每趟包裹更多）
import random
import math
import os

def generate_test_data(folder_prefix, num_nodes, num_edges, num_packages, mode="uniform"):
    """
    生成测试数据
    :param folder_prefix: 输出文件夹路径或前缀
    :param num_nodes: 节点数
    :param num_edges: 边数
    :param num_packages: 包裹数
    :param mode: 数据模式
                 - "uniform":     均匀随机分布（标准测试）
                 - "cluster":     单向聚类分布
                 - "urgent":      急件冲突分布
                 - "alns_killer": 高级对比模式（多中心冲突带 + 严苛时空错位 + 高密度装载）
    """
    if folder_prefix and not os.path.exists(folder_prefix):
        os.makedirs(folder_prefix, exist_ok=True)
        
    # 保证边数合法（无向简单图连通的最少和最多边数）
    max_possible_edges = num_nodes * (num_nodes - 1) // 2
    num_edges = max(num_nodes - 1, min(num_edges, max_possible_edges))

    # =========================================================================
    # 1. 生成 map.txt (保证坐标与边权符合几何欧氏距离)
    # =========================================================================
    nodes = []
    map_file = os.path.join(folder_prefix, "map.txt") if folder_prefix else "map.txt"
    with open(map_file, "w", encoding="utf-8") as f:
        f.write(f"{num_nodes} {num_edges}\n")
        for i in range(num_nodes):
            is_station = 1 if i == 0 else 0
            if i == 0:
                x, y = 500, 500  # 驿站放在地图中心
            else:
                x, y = random.randint(0, 1000), random.randint(0, 1000)
            nodes.append((x, y))
            f.write(f"{i} {x} {y} {is_station}\n")
        
        edges = set()
        # 确保图连通: 先生成一棵生成树 (且倾向于连接地理位置较近的节点)
        for i in range(1, num_nodes):
            u = min(range(i), key=lambda idx: math.hypot(nodes[idx][0] - nodes[i][0], nodes[idx][1] - nodes[i][1]) * random.uniform(0.8, 1.5))
            dist = math.hypot(nodes[u][0] - nodes[i][0], nodes[u][1] - nodes[i][1])
            w = max(1.0, round(dist * 0.05 * random.uniform(0.9, 1.1), 1))
            edges.add((min(u, i), max(u, i), w))
        
        # 补齐剩余的边 (优先连欧氏距离近的，增强真实路网特征)
        all_possible_pairs = [(u, v) for u in range(num_nodes) for v in range(u + 1, num_nodes) if (u, v) not in edges]
        all_possible_pairs.sort(key=lambda pair: math.hypot(nodes[pair[0]][0] - nodes[pair[1]][0], nodes[pair[0]][1] - nodes[pair[1]][1]))
        
        candidate_pool = all_possible_pairs[:max(len(all_possible_pairs), (num_edges - len(edges)) * 3)]
        random.shuffle(candidate_pool)
        
        for u, v in candidate_pool:
            if len(edges) >= num_edges:
                break
            dist = math.hypot(nodes[u][0] - nodes[v][0], nodes[u][1] - nodes[v][1])
            w = max(1.0, round(dist * 0.05 * random.uniform(0.9, 1.1), 1))
            edges.add((u, v, w))
                    
        for u, v, w in edges:
            f.write(f"{u} {v} {w}\n")

    # =========================================================================
    # 2. 生成 packages.txt
    # =========================================================================
    packages_file = os.path.join(folder_prefix, "packages.txt") if folder_prefix else "packages.txt"
    with open(packages_file, "w", encoding="utf-8") as f:
        f.write(f"{num_packages}\n")
        for i in range(1, num_packages + 1):
            if mode == "cluster":
                # 聚类模式：80%的包裹去往同一特定方向
                if random.random() < 0.8:
                    dest = random.choice([idx for idx in range(1, num_nodes) if nodes[idx][0] > 500 and nodes[idx][1] > 500] or list(range(1, num_nodes)))
                else:
                    dest = random.randint(1, num_nodes - 1)
                weight = round(random.uniform(2.0, 7.0), 1)
                arrive = round(random.uniform(0.0, 30.0), 1)
                deadline = arrive + round(random.uniform(50.0, 150.0), 1)

            elif mode == "urgent":
                # 急件冲突模式：超急轻件 vs 不急重件
                dest = random.randint(1, num_nodes - 1)
                if random.random() < 0.3:  # 30% 急件
                    weight = round(random.uniform(1.0, 3.0), 1)
                    arrive = round(random.uniform(0.0, 20.0), 1)
                    deadline = arrive + round(random.uniform(10.0, 25.0), 1)
                else:  # 70% 普通重件
                    weight = round(random.uniform(5.0, 10.0), 1)
                    arrive = round(random.uniform(0.0, 20.0), 1)
                    deadline = arrive + round(random.uniform(80.0, 200.0), 1)

            elif mode == "alns_killer":
                # -------------------------------------------------------------
                # ALNS 专属优化模式：制造多中心重叠 + 严苛时空错位 + 轻包裹高密度
                # -------------------------------------------------------------
                weight = round(random.uniform(1.0, 2.5), 1) # 较轻包裹，增加单趟可装载数量
                
                # 构造双簇（左下 / 右上）以及中间的模糊冲突带（占 20%）
                r = random.random()
                if r < 0.4:
                    # 簇 1：左下角区域
                    candidates = [idx for idx in range(1, num_nodes) if nodes[idx][0] < 450 and nodes[idx][1] < 450]
                elif r < 0.8:
                    # 簇 2：右上角区域
                    candidates = [idx for idx in range(1, num_nodes) if nodes[idx][0] > 550 and nodes[idx][1] > 550]
                else:
                    # 冲突重叠带：中间交界处（给 K-means/贪心设陷阱，需要 ALNS 破局）
                    candidates = [idx for idx in range(1, num_nodes) if 400 <= nodes[idx][0] <= 600]
                
                dest = random.choice(candidates) if candidates else random.randint(1, num_nodes - 1)

                # 构造时空错位：地理邻近但时间极度冲突
                arrive = round(random.uniform(0.0, 40.0), 1)
                if random.random() < 0.35: # 35% 超急件，迫使车辆进行合理分工
                    deadline = arrive + round(random.uniform(12.0, 28.0), 1)
                else:
                    deadline = arrive + round(random.uniform(100.0, 220.0), 1)

            else:
                # 默认均匀随机
                weight = round(random.uniform(1.0, 8.0), 1)
                dest = random.randint(1, num_nodes - 1)
                arrive = round(random.uniform(0.0, 50.0), 1)
                deadline = arrive + round(random.uniform(20.0, 100.0), 1)
                
            f.write(f"{i} {weight} {dest} {arrive} {deadline}\n")

    # =========================================================================
    # 3. 生成 car.txt
    # =========================================================================
    car_file = os.path.join(folder_prefix, "car.txt") if folder_prefix else "car.txt"
    with open(car_file, "w", encoding="utf-8") as f:
        # 如果是 ALNS 模式，提升车容为 30.0，使一趟能装载更多包裹，增加 TSP / 路径重构复杂度
        capacity = 30.0 if mode == "alns_killer" else 15.0
        speed = 10.0
        car_weight = 20.0
        f.write(f"{speed} {car_weight} {capacity}\n")

    print(f"[{mode}模式] 数据生成完成！路径:'{folder_prefix}' | 节点:{num_nodes}, 边:{num_edges}, 包裹:{num_packages}")


# =============================================================================
# 主程序入口：在此配置你要测试生成的数据集
# =============================================================================
if __name__ == "__main__":
    # 1. 生成默认均匀数据（放在当前目录）
    generate_test_data("", num_nodes=60, num_edges=100, num_packages=80, mode="uniform")
    
    # 2. 生成能够充分发挥 T5 (ALNS) 算法优势的对比数据集（放在 test_alns/ 目录下）
    generate_test_data("test_alns/", num_nodes=80, num_edges=150, num_packages=120, mode="alns_killer")