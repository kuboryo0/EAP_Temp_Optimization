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

# === エッジ定義（有向エッジで追加） ===
edges = []
s = {}
for (i1, j1) in nodes:
    neighbors = [(i1 + 1, j1), (i1, j1 + 1), (i1 - 1, j1), (i1, j1 - 1),
                 (i1 + 1, j1 + 1), (i1 + 1, j1 - 1), (i1 - 1, j1 + 1), (i1 - 1, j1 - 1)]
    for (i2, j2) in neighbors:
        if 0 <= i2 < grid_size_x and 0 <= j2 < grid_size_y:
            edge = ((i1, j1), (i2, j2))
            edges.append(edge)
            dist = np.hypot(i1 - i2, j1 - j2)
            s[edge] = dist

temp_eff = 0.25
alpha = 1000.0
beta = 1400.0
neighbor_offsets = [(-1, -1), (-1, 0), (-1, 1),
                    (0, -1),          (0, 1),
                    (1, -1),  (1, 0), (1, 1)]
entrance_node = (0, 0)

# === MILP関数（connect_constraints を受け取りカット制約追加可能） ===
def MILP_OptimizeRoad_Connect(soil_amount, connect_constraints):
    model = pulp.LpProblem("HaulRoad_Design", pulp.LpMinimize)

    x = pulp.LpVariable.dicts("x", edges, cat="Binary")
    f1 = pulp.LpVariable.dicts("f1", edges, lowBound=0)
    f2 = pulp.LpVariable.dicts("f2", edges, lowBound=0)

    t1 = {e: s[e] for e in edges}
    t2 = {e: s[e] * temp_eff for e in edges}

    # 目的関数
    model += (
        alpha * pulp.lpSum(f1[e] * t1[e] + f2[e] * t2[e] for e in edges) +
        beta * pulp.lpSum(s[e] * x[e] for e in edges)
    )

    # 流量制約
    for e in edges:
        model += f1[e] <= (1 - x[e]) * total_soil, f"no_unpaved_flow_if_paved_{e}"
        model += f2[e] <= x[e] * total_soil, f"no_paved_flow_if_unpaved_{e}"

    # フロー保存制約
    for (i, j) in nodes:
        inflow = pulp.lpSum(f1[(u, v)] + f2[(u, v)] for (u, v) in edges if v == (i, j))
        outflow = pulp.lpSum(f1[(u, v)] + f2[(u, v)] for (u, v) in edges if u == (i, j))
        model += (inflow - outflow == -soil_amount[i, j], f"flow_conservation_{i}_{j}")

    # 連結制約（カット制約）
    if len(connect_constraints) > 0:
        for conn in connect_constraints:
            model += pulp.lpSum(x[e] for e in conn) >= 1

    model.solve()
    print("Status:", pulp.LpStatus[model.status])
    print("Objective value:", pulp.value(model.objective))

    for (u, v) in edges:
        f1_val = pulp.value(f1[(u, v)])
        f2_val = pulp.value(f2[(u, v)])
        if abs(f1_val) > 1e-6 or abs(f2_val) > 1e-6:
            print(f"{u} -> {v}: x={int(pulp.value(x[(u, v)]))}, f1={f1_val:.2f}, f2={f2_val:.2f}")

    return model, x, f1, f2, edges

# === 接続チェック関数 ===
def check_connectivity(selected_edges):
    G = nx.Graph()
    G.add_edges_from(selected_edges)
    return nx.node_connected_component(G, entrance_node)

# === カットエッジ探索 ===
def find_cut_edges(current_edges):
    G = nx.Graph()
    G.add_edges_from(current_edges)
    largest_component = max(nx.connected_components(G), key=len)

    connected_set = set(current_edges)
    cut_edges = set()

    for node in largest_component:
        i, j = node
        for di, dj in neighbor_offsets:
            ni, nj = i + di, j + dj
            neighbor = (ni, nj)
            if 0 <= ni < grid_size_x and 0 <= nj < grid_size_y:
                edge = (node, neighbor)  # **ここを有向エッジに合わせて変更**
                if edge not in connected_set and (neighbor not in largest_component):
                    cut_edges.add(edge)

    return cut_edges

# === メインループ ===
connect_constraints = []
entrance_constraint_added = False  # entrance制約を追加済かどうか
while True:
    model, x, f1, f2, edges = MILP_OptimizeRoad_Connect(soil_amount, connect_constraints)

    selected_edges = [e for e in edges if pulp.value(x[e]) > 0.5]

    # ノード接続状況チェック（次数0の単独ノードを除く）
    G = nx.Graph()
    G.add_edges_from(selected_edges)
    non_singleton_components = [c for c in nx.connected_components(G) if len(c) > 1]
    print("Non-singleton components found:", non_singleton_components)

    if len(non_singleton_components) == 1:
        largest_component = non_singleton_components[0]
        if entrance_node in largest_component:
            print("All nodes connected to entrance. Optimization complete.")
            break
        else:
            print(f"Entrance node {entrance_node} is NOT included in the component. Adding entrance connection constraint.")
            if not entrance_constraint_added:
                entrance_edges = []
                i, j = entrance_node
                for di, dj in neighbor_offsets:
                    ni, nj = i + di, j + dj
                    if 0 <= ni < grid_size_x and 0 <= nj < grid_size_y:
                        edge = (entrance_node, (ni, nj))
                        entrance_edges.append(edge)
                connect_constraints.append(entrance_edges)
                entrance_constraint_added = True
            else:
                print("Entrance connection constraint already added.")
    else:
        largest_component = max(non_singleton_components, key=len)
        print(f"Multiple components found. Largest component size: {len(largest_component)}")

        cut_edges = set()
        for node in largest_component:
            i, j = node
            for di, dj in neighbor_offsets:
                ni, nj = i + di, j + dj
                neighbor = (ni, nj)
                edge = (node, neighbor)
                if 0 <= ni < grid_size_x and 0 <= nj < grid_size_y:
                    if edge not in selected_edges and (neighbor not in largest_component):
                        cut_edges.add(edge)
        if cut_edges:
            connect_constraints.append(list(cut_edges))
            print(f"Added constraint for cut edges: {cut_edges}")
        else:
            print("No further cut edges found. Cannot connect remaining nodes.")
            break

# === 最終結果出力 ===
print("Final Selected Edges:")
for e in selected_edges:
    print(e)

print("Objective value:", pulp.value(model.objective))
