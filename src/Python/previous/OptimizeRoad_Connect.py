import pulp
import networkx as nx
import numpy as np

# === グリッドサイズとノード定義 ===
grid_size_x, grid_size_y = 4, 4
nodes = [(i, j) for i in range(grid_size_x) for j in range(grid_size_y)]

# === 切土・盛土・ゼロ土量セル ===
cut_indices = [[(1, 0), 1], [(1, 2), 1], [(2, 0), 1], [(2, 1), 1],
               [(2, 2), 1], [(2, 3), 1], [(3, 0), 1], [(3, 1), 1], [(3, 3), 2]]
fill_indices = [[(0, 0), 2], [(0, 1), 1], [(0, 2), 1], [(0, 3), 1],
                [(1, 3), 2], [(1, 1), 1], [(3, 2), 2]]
zero_indices = []

total_soil = sum(k for _, k in cut_indices) 

soil_amount = np.zeros((grid_size_x, grid_size_y))
for (i, j), k in cut_indices:
    soil_amount[i, j] = k
for (i, j), k in fill_indices:
    soil_amount[i, j] = -k
for (i, j), k in zero_indices:
    soil_amount[i, j] = 0

connect_nodes = [[((1, 0), (0, 0))], [((1, 0), (0, 1))], [((1, 2), (0, 2))]]

def MILP_OptimizeRoad_Connect(soil_amount, connect_nodes):    
    # === エッジと距離（斜め方向含む） ===
    edges = []
    s = {}
    for (i1, j1) in nodes:
        neighbors = [(i1 + 1, j1), (i1, j1 + 1),  # 下、右
                    (i1 - 1, j1), (i1, j1 - 1),  # 上、左
                    (i1 + 1, j1 + 1), (i1 + 1, j1 - 1),
                    (i1 - 1, j1 + 1), (i1 - 1, j1 - 1)
                ]  # 右下、左下、右上、左上
        for (i2, j2) in neighbors:
            if 0 <= i2 < grid_size_x and 0 <= j2 < grid_size_y:
                edge = ((i1, j1), (i2, j2))
                edges.append(edge)
                dist = np.hypot(i1 - i2, j1 - j2)
                s[edge] = dist

    # === 時間係数 ===
    t1 = {e: s[e] for e in edges}
    temp_eff = 0.25
    t2 = {e: s[e] * temp_eff for e in edges}

    # === コスト係数 ===
    alpha = 1000.0      # 時間コスト
    beta = 1400.0         # 舗装建設コスト

    # === グラフ構築 ===
    G = nx.Graph()
    G.add_edges_from(edges)

    # === モデル定義 ===
    model = pulp.LpProblem("HaulRoad_Design", pulp.LpMinimize)

    x = pulp.LpVariable.dicts("x", edges, cat="Binary")     # 舗装道路
    f1 = pulp.LpVariable.dicts("f1", edges, lowBound=0)     # 未舗装に流す交通量
    f2 = pulp.LpVariable.dicts("f2", edges, lowBound=0)     # 舗装に流す交通量

    # === 目的関数（維持費なし） ===
    model += (
        alpha * pulp.lpSum(f1[e] * t1[e] + f2[e] * t2[e] for e in edges) +
        beta * pulp.lpSum(s[e] * x[e] for e in edges)
    )

    for e in edges:
        # 舗装されたら未舗装の流量は禁止（f1 = 0）
        model += f1[e] <= (1 - x[e]) * total_soil, f"no_unpaved_flow_if_paved_{e}"
        
        # 未舗装なら舗装の流量は禁止（f2 = 0）
        model += f2[e] <= x[e] * total_soil, f"no_paved_flow_if_unpaved_{e}"

    # === フロー保存制約 ===
    for (i, j) in nodes:
        inflow = pulp.lpSum(f1[(u, v)] + f2[(u, v)] for (u, v) in edges if v == (i, j))
        outflow = pulp.lpSum(f1[(u, v)] + f2[(u, v)] for (u, v) in edges if u == (i, j))
        model += (inflow - outflow == -soil_amount[i, j],
                f"flow_conservation_{i}_{j}")

    # === 連結成分の制約 ===
    # model += (
    #     x[((1, 0), (0, 0))] +
    #     x[((1, 0), (0, 1))] +
    #     x[((1, 2), (0, 2))] >= 1,
    #     "at_least_one_connection"
    # )
    for conn in connect_nodes:
        model += (
            pulp.lpSum(x[e] for e in conn) >= 1,
            f"at_least_one_connection_{conn}"
        )

    # === 解く ===
    model.solve()
    return model, x, f1, f2, edges

# === 結果出力 ===
model, x, f1, f2, edges = MILP_OptimizeRoad_Connect(soil_amount, connect_nodes)
print("Status:", pulp.LpStatus[model.status])
for (u, v) in edges:
    f1_val = pulp.value(f1[(u, v)])
    f2_val = pulp.value(f2[(u, v)])
    if abs(f1_val) > 1e-6 or abs(f2_val) > 1e-6:
        print(f"{u} -> {v}: x={int(pulp.value(x[(u, v)]))}, f1={f1_val:.2f}, f2={f2_val:.2f}")

print("Objective value:", pulp.value(model.objective))