import heapq

# 4x4グリッドのサイズ
GRID_SIZE = 4
temp_eff = 0.5

# 各セルが持つ仮設道路情報 (座標: [方向のセット])
temporary_roads = [
    {
    (2, 0): {2},  
    (1, 1): {2,5},  
    (0, 2): {5}
    },
    {
    (1, 2): {4},
    (1, 3):{3}
    }
]

# 移動方向（上下左右および斜め）
DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def get_distance(a, b):
    """current と neighbor のユークリッド距離を計算"""
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def heuristic(a, b):
    """ユークリッド距離のヒューリスティック関数"""
    return temp_eff * get_distance(a, b)


def get_cost(current, neighbor):
    """移動コストを計算"""
    distance = get_distance(current, neighbor)
    direction = (neighbor[0] - current[0], neighbor[1] - current[1])
    direction_index_current = DIRECTIONS.index(direction)
    direction_index_neighbor = DIRECTIONS.index((-direction[0], -direction[1]))
    current_distance = distance/2
    neighbor_distance = distance/2
    # 仮設道路がある場合
    for temp in temporary_roads:
        if direction_index_current in temp.get(current, set()):
            current_distance *=  temp_eff
        if direction_index_neighbor in temp.get(neighbor, set()):
            neighbor_distance *=  temp_eff 
    
    return current_distance + neighbor_distance


def a_star(start, goal):
    """A*アルゴリズムでstartからgoalへの最小コスト経路を探索"""
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            break

        for direction in DIRECTIONS:
            neighbor = (current[0] + direction[0], current[1] + direction[1])

            # グリッド範囲外を除外
            if not (0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE):
                continue

            new_cost = cost_so_far[current] + get_cost(current, neighbor)

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(goal, neighbor)
                heapq.heappush(open_set, (priority, neighbor))
                came_from[neighbor] = current

    # ゴールからスタートまでの経路を再構築
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    # 仮設道路を削除
    remove_temporary_roads(start, goal)

    return path, cost_so_far[goal]


def remove_temporary_roads(start, goal):
    """指定されたセル（start, goal）から仮設道路を削除"""
    for cell in [start, goal]:
        if cell in temporary_roads:
            del temporary_roads[cell]


# ペアの開始・終了地点を設定 (例)
start_goal_pairs = [((2, 0), (1, 3))]

# 各ペアに対する最小コスト経路を計算
for start, goal in start_goal_pairs:
    path, cost = a_star(start, goal)
    print(f"Start: {start}, Goal: {goal}")
    print(f"Path: {path}")
    print(f"Cost: {cost}\n")
    print(f"Temporary Roads After Removal: {temporary_roads}")
