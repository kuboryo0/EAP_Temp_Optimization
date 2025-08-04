import networkx as nx

# Grid size
GRID_WIDTH = 4
GRID_HEIGHT = 4

# 現在の仮設道路（既に接続されているエッジ）
connected_edges = [
    ((0, 0), (0, 1)),
    ((0, 0), (1, 0)),
    ((1, 0), (2, 0)),
    ((2, 1), (2, 2)),
]

# 入口ノード
entrance_node = (0, 0)

# 8方向の隣接オフセット
neighbor_offsets = [(-1, -1), (-1, 0), (-1, 1),
                    (0, -1),          (0, 1),
                    (1, -1),  (1, 0), (1, 1)]

# グリッド内にあるかを判定
def in_bounds(i, j):
    return 0 <= i < GRID_HEIGHT and 0 <= j < GRID_WIDTH

def find_connected_components(connected_edges):
    all_nodes = [(i, j) for i in range(GRID_HEIGHT) for j in range(GRID_WIDTH)]

    # グラフ構築
    G = nx.Graph()
    G.add_nodes_from(all_nodes)
    G.add_edges_from(connected_edges)

    # --- 連結成分の確認 ---
    components = list(nx.connected_components(G))
    print(" Connected components:", components)

    non_singleton_components = [comp for comp in components if len(comp) > 1]
    print("Connected components (excluding singletons):", non_singleton_components)

    if len(non_singleton_components) == 0:
        print("No connected components with more than one node.")
        return
    
    largest_component = non_singleton_components[0]
    if len(non_singleton_components) == 1:        
        print("Only one large component found:", largest_component)
        if entrance_node in largest_component:
            print(f"Entrance node {entrance_node} is included in the component. No further action needed.")
            return
        else:
            print(f"Entrance node {entrance_node} is NOT included in the component.")
    else:
        if entrance_node in largest_component:
            print(f"Entrance node {entrance_node} is included in the largest component.")
        else:
            print(f"Entrance node {entrance_node} is NOT included in the largest component.")
    # 道路がつながっていない場合または入り口ノードを含まない場合
    largest_component = max(non_singleton_components, key=len)
    print("Largest component (excluding singletons):", largest_component)
    # 最大連結成分
    largest_component = max(components, key=len)
    print(" Largest component:", largest_component)

    # --- δ(S)（最大連結成分のカットエッジ）---
    connected_set = {tuple(sorted(edge)) for edge in connected_edges}
    cut_edges_largest = set()
    for node in largest_component:
        i, j = node
        for di, dj in neighbor_offsets:
            ni, nj = i + di, j + dj
            neighbor = (ni, nj)
            if in_bounds(ni, nj):
                edge = tuple(sorted((node, neighbor)))
                if edge not in connected_set:
                    if (node in largest_component) != (neighbor in largest_component):
                        cut_edges_largest.add(edge)

    print("\n δ(S): Cut edges for largest component:")
    print(cut_edges_largest)

    # --- δ({entrance}) ---
    entrance_cut_edges = set()
    i, j = entrance_node
    for di, dj in neighbor_offsets:
        ni, nj = i + di, j + dj
        neighbor = (ni, nj)
        if in_bounds(ni, nj):
            edge = tuple(sorted((entrance_node, neighbor)))
            if edge not in connected_set:
                entrance_cut_edges.add(edge)

    print(f"\n δ(entrance): Cut edges around entrance {entrance_node}:")
    print(entrance_cut_edges)

    # --- 入口から到達できるノード集合 ---
    reachable_from_entrance = nx.node_connected_component(G, entrance_node)
    print(f"\nNodes reachable from entrance {entrance_node}: {reachable_from_entrance}")

find_connected_components(connected_edges)
