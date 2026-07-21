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
    :param mode: 数据模式，用于 T6 策略验证
                 - "uniform": 均匀随机分布（标准测试）
                 - "cluster": 聚类分布（同一方向包裹较多，验证“区域/路线聚合”策略优于“编号/纯最近优先”）
                 - "urgent":  急件冲突分布（验证“时间截止优先/综合打分”策略优于“单纯最近优先”）
    """
    if folder_prefix and not os.path.exists(folder_prefix):
        os.makedirs(folder_prefix, exist_ok=True)
        
    # 保证边数合法（无向简单图连通的最少和最多边数）
    max_possible_edges = num_nodes * (num_nodes - 1) // 2
    num_edges = max(num_nodes - 1, min(num_edges, max_possible_edges))

    # 1. 生成 map.txt (保证坐标与边权符合几何欧氏距离)
    nodes = []
    map_file = os.path.join(folder_prefix, "map.txt") if folder_prefix else "map.txt"
    with open(map_file, "w") as f:
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
            # 在已生成的节点中找一个较近的节点相连
            u = min(range(i), key=lambda idx: math.hypot(nodes[idx][0] - nodes[i][0], nodes[idx][1] - nodes[i][1]) * random.uniform(0.8, 1.5))
            dist = math.hypot(nodes[u][0] - nodes[i][0], nodes[u][1] - nodes[i][1])
            # 边权 = 欧氏距离 * [0.8, 1.2] 的随机折立因子（按比例缩小以便在合理范围如 1~50）
            w = max(1.0, round(dist * 0.05 * random.uniform(0.9, 1.1), 1))
            edges.add((min(u, i), max(u, i), w))
        
        # 补齐剩余的边 (优先连欧氏距离近的，增强真实路网特征)
        all_possible_pairs = [(u, v) for u in range(num_nodes) for v in range(u + 1, num_nodes) if (u, v) not in edges]
        all_possible_pairs.sort(key=lambda pair: math.hypot(nodes[pair[0]][0] - nodes[pair[1]][0], nodes[pair[0]][1] - nodes[pair[1]][1]))
        
        # 从距离最近的前 3*(num_edges - len(edges)) 对节点中随机选，或者随机补充
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

    # 2. 生成 packages.txt
    packages_file = os.path.join(folder_prefix, "packages.txt") if folder_prefix else "packages.txt"
    with open(packages_file, "w") as f:
        f.write(f"{num_packages}\n")
        for i in range(1, num_packages + 1):
            if mode == "cluster":
                # 聚类模式：80%的包裹去往同一个特定方向/区域（如右上角区域节点），考核区域聚合策略
                if random.random() < 0.8:
                    dest = random.choice([idx for idx in range(1, num_nodes) if nodes[idx][0] > 500 and nodes[idx][1] > 500] or range(1, num_nodes))
                else:
                    dest = random.randint(1, num_nodes - 1)
                weight = round(random.uniform(2.0, 7.0), 1)
                arrive = round(random.uniform(0.0, 30.0), 1)
                deadline = arrive + round(random.uniform(50.0, 150.0), 1)
            elif mode == "urgent":
                # 急件冲突模式：生成部分“超急且轻”的包裹 vs “不急但极重”的包裹，考核截止时间与综合权重策略
                dest = random.randint(1, num_nodes - 1)
                if random.random() < 0.3:  # 30% 急件
                    weight = round(random.uniform(1.0, 3.0), 1)
                    arrive = round(random.uniform(0.0, 20.0), 1)
                    deadline = arrive + round(random.uniform(10.0, 25.0), 1)  # 窗口极短
                else:  # 70% 普通重件
                    weight = round(random.uniform(5.0, 10.0), 1)
                    arrive = round(random.uniform(0.0, 20.0), 1)
                    deadline = arrive + round(random.uniform(80.0, 200.0), 1)
            else:
                # 默认均匀随机
                weight = round(random.uniform(1.0, 8.0), 1)
                dest = random.randint(1, num_nodes - 1)
                arrive = round(random.uniform(0.0, 50.0), 1)
                deadline = arrive + round(random.uniform(20.0, 100.0), 1)
                
            f.write(f"{i} {weight} {dest} {arrive} {deadline}\n")

    # 3. 生成 car.txt
    car_file = os.path.join(folder_prefix, "car.txt") if folder_prefix else "car.txt"
    with open(car_file, "w") as f:
        # speed = 10.0, car_weight = 20.0, capacity = 15.0
        f.write("10.0 20.0 15.0\n")

    print(f"[{mode}模式] 数据已生成！目录/前缀:'{folder_prefix}' | 节点数:{num_nodes}, 边数:{num_edges}, 包裹数:{num_packages}")

# 示例：生成用于 T6 的不同特征对比数据
if __name__ == "__main__":
    # 1. 默认均匀随机数据
    generate_test_data("", num_nodes=60, num_edges=100, num_packages=80, mode="uniform")
    # 2. 针对 T6 生成聚类对比数据
    generate_test_data("test_cluster/", num_nodes=40, num_edges=70, num_packages=50, mode="cluster")
    # 3. 针对 T6 生成紧急时间限制对比数据
    generate_test_data("test_urgent/", num_nodes=80, num_edges=140, num_packages=100, mode="urgent")