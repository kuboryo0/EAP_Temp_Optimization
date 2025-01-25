import pulp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import heapq
import time
import copy
# セルの土量設定
# 土量 = np.array([-1000, -4000, -5000, 550, -500, 800, 450, 6700, 2000]).reshape((3, 3))
# temp_eff = 0.7
# v =  1/temp_eff
# tan_alpha = math.sqrt(v**2-1)
# sin_alpha = math.sqrt(v**2-1)/v
# distance = 0

def validate_dict(data):
    """
    辞書が以下の形式を満たしているかを確認する:
    - キー: (x, y) の形式 (タプル)
    - 値: 0～7 の整数のみを要素に持つセット
    """
    if not isinstance(data, dict):
        return False, "Input is not a dictionary."

    for key, value in data.items():
        # キーがタプルで2つの整数を持つ形式か確認
        if not (isinstance(key, tuple) and len(key) == 2 and all(isinstance(coord, int) for coord in key)):
            return False, f"Invalid key: {key} (must be a tuple of two integers)."

        # 値がセットで0～7の整数のみを含むか確認
        if not (isinstance(value, set) and all(isinstance(v, int) and 0 <= v <= 7 for v in value)):
            return False, f"Invalid value for key {key}: {value} (must be a set of integers between 0 and 7)."

    return True, "Dictionary is valid."


def earth_allocation(cut_indices, fill_indices):
    def calculate_cost(cut_indices_float, fill_indices_float):
        """各切土地点と盛土地点間のユークリッド距離を計算してコスト行列を作成"""
        num_cut = len(cut_indices_float)
        num_fill = len(fill_indices_float)
        costs = [[0 for _ in range(num_fill)] for _ in range(num_cut)]
        
        for c in range(num_cut):
            for f in range(num_fill):
                # 各切土地点と盛土地点のユークリッド距離を計算
                distance = math.sqrt(
                    (cut_indices_float[c][0][0] - fill_indices_float[f][0][0])**2 +
                    (cut_indices_float[c][0][1] - fill_indices_float[f][0][1])**2
                )
                costs[c][f] = distance
        
        return costs
    
    
    #切土の座標と土量
    cut_indices_float = cut_indices
    for i in range(len(cut_indices)):
        c_ix = float(cut_indices[i][0][0])
        c_iy = float(cut_indices[i][0][1])
        cut_indices_float[i][0] = (c_ix,c_iy)
    #盛土の座標と土量
    fill_indices_float = fill_indices
    for i in range(len(fill_indices)):
        f_ix = float(fill_indices[i][0][0])
        f_iy = float(fill_indices[i][0][1])
        fill_indices_float[i][0] = (f_ix,f_iy)



    sum_cut = sum(cut_indices[i][1] for i in range(len(cut_indices)))
    sum_fill = sum(fill_indices[i][1] for i in range(len(fill_indices)))
    # 土量の合計が一致しているか確認
    if (sum_cut != sum_fill):
        print("input error:切土と盛土の土量が違います")
        exit() 


    # コスト行列の作成
    costs = calculate_cost(cut_indices_float, fill_indices_float)

    # 問題の設定
    prob = pulp.LpProblem("土砂運搬最適化", pulp.LpMinimize)

    # 変数の定義
    T = sum_cut  # ステップ数
    num_fill = len(fill_indices)
    num_cut = len(cut_indices)

    # 変数xはステップtで切土cから盛土fに運んだ時、x[t][c][f]=1となる。それ以外は0
    x_vars = pulp.LpVariable.dicts("x", (range(T), range(num_cut), range(num_fill)), cat='Binary')
    
    objective = pulp.LpAffineExpression()
    m=[0] * T
    n=[0] * T
    #各ステップで切土から盛土に土を運んだ時のコスト
    for t in range(T):
        for f in range(num_fill):
            for c in range(num_cut):
                if (x_vars[t][c][f]==1):
                    m[t] = c
                    n[t] = f
                objective += 20 * costs[c][f] * x_vars[t][c][f]


    prob += objective
    # 制約条件
    # 各ステップでちょうど1つの切土地点と盛土地点が選ばれる
    for t in range(T):
        prob += pulp.lpSum(x_vars[t][c][f] for c in range(num_cut) for f in range(num_fill)) == 1



    # 最終ステップで全ての土量がゼロになるように制約
    for c in range(num_cut):
        prob += pulp.lpSum(x_vars[t][c][f] for t in range(T) for f in range(num_fill)) == cut_indices[c][1]

    for f in range(num_fill):
        prob += pulp.lpSum(x_vars[t][c][f] for t in range(T) for c in range(num_cut)) == fill_indices[f][1]
    # 問題の解決
    prob.solve()

    # 結果の表示
    print("ステータス:", pulp.LpStatus[prob.status])
    print("最小コスト:", pulp.value(prob.objective))
    # 複数の矢印を描画するための例
    routes = []

    # 解の表示
    for t in range(T):
        print(f"\nステップ {t+1}:")
        for f in range(num_fill):
            for c in range(num_cut):
                if pulp.value(x_vars[t][c][f]) > 0.5:
                    print(f"  切土地点 ({cut_indices[c]}) から 盛土地点 ({fill_indices[f]}) に土が運ばれました")
                    routes.append((cut_indices[c][0],fill_indices[f][0]))

    print("routes",routes)
    return routes


def a_star(start_goal_pairs,temporary_roads,grid_size_x,grid_size_y,temp_eff,soil_amount):
    DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    def get_distance(a, b,soil):
        """current と neighbor の3次元距離を計算"""
        # print("a",a)
        # print("b",b)
        horizontal_distance = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        vertical_distance = abs(soil[int(a[0])][int(a[1])] - soil[int(b[0])][int(b[1])])
        return (horizontal_distance**2 + vertical_distance**2)**0.5


    def heuristic(a, b,soil_amount):
        """ユークリッド距離のヒューリスティック関数"""
        return temp_eff * get_distance(a, b,soil_amount)


    def get_cost(current, neighbor,soil_amount):
        """移動コストを計算"""
        distance = get_distance(current, neighbor,soil_amount)
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
    
    def remove_temporary_roads(start, goal,temps):
        """指定されたセル（start, goal）から仮設道路を削除"""
        for cell in [start, goal]:
            for temp in temps:
                if cell in temp:
                    del temp[cell]
    
    def change_soil(start,goal,soil_amount):
        """指定されたセル（start, goal）から土量を変更"""
        soil_amount[int(start[0])][int(start[1])] -= 1
        soil_amount[int(goal[0])][int(goal[1])] += 1

    """A*アルゴリズムでstartからgoalへの最小コスト経路を探索"""
    def astar(start, goal,temp,soil_amount):
        temp_copy =copy.deepcopy(temp)
        soil_amount_copy = copy.deepcopy(soil_amount)
        # print(f"Temporary Roads Before Removal: {temp_copy}")
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
                if not (0 <= neighbor[0] < grid_size_x and 0 <= neighbor[1] < grid_size_y):
                    continue

                new_cost = cost_so_far[current] + get_cost(current, neighbor,soil_amount_copy)

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor,soil_amount_copy)
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
        remove_temporary_roads(start, goal,temp_copy)
        change_soil(start,goal,soil_amount_copy)
        # print(f"Temporary Roads After Removal: {temp_copy}")

        return path, cost_so_far[goal],temp_copy,soil_amount_copy
    

    total_cost = 0
    path_list = []
    # print("temporary_roads in astar",temporary_roads)
    temp_deepcopy = copy.deepcopy(temporary_roads)
    soil_amount_deepcopy = copy.deepcopy(soil_amount)
    for start, goal in start_goal_pairs:
        # print("soil_amount before",soil_amount_deepcopy)
        path, cost,temp_deepcopy,soil_amount_deepcopy = astar(start, goal,temp_deepcopy,soil_amount_deepcopy)
        total_cost += cost
        path_list.append(path)
        # print(f"Start: {start}, Goal: {goal}")

        # print(f"Path: {path}")
        # print(f"Cost: {cost}\n")
    # print(f"Total Cost: {total_cost}")
    return  path_list,total_cost

def temporary_road_check(temporary_roads,grid_size_x,grid_size_y):
    DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    for temp in temporary_roads:
        for start, directions in temp.items():
            for direction in directions:
                if direction not in range(8):
                    print("direction error")
                    exit()
                neighbor_x = start[0] + DIRECTIONS[direction][0]
                neighbor_y = start[1] + DIRECTIONS[direction][1]
                if not(0<=neighbor_x<grid_size_x and 0<=neighbor_y<grid_size_y):
                    print("neighbor error")
                    exit()


def temp_length(temp_list,soil_amount):
    DIRECTIONS = [
    (-1, -1),  # 0
    (-1, 0),   # 1
    (-1, 1),   # 2
    (0, -1),   # 3
    (0, 1),    # 4
    (1, -1),   # 5
    (1, 0),    # 6
    (1, 1)     # 7
]
    """
    仮設道路の総長を計算する関数。
    
    Parameters:
        temporary_roads (dict): 各座標における方向情報。
        directions (list): 方向ベクトルのリスト。
    
    Returns:
        float: 仮設道路の総長。
    """
    total_length = 0
    for temp in temp_list:
        for start, dir_set in temp.items():
            for direction in dir_set:
                # 各方向の移動量を取得
                dx, dy = DIRECTIONS[direction]
                # print("dx,dy",dx,dy)
                # 仮設道路の長さはベクトルの長さとして計算
                horizontal_length = ((dx/2)**2 + (dy/2)**2)**0.5
                vertical_length = abs(soil_amount[int(start[0])][int(start[1])] - soil_amount[int(start[0] + dx)][int(start[1] + dy)])/2
                # print("length",length)
                total_length += (horizontal_length**2+vertical_length**2)**0.5
    print("total_length",total_length)
    return total_length

#要修正
def calculate_temporary_road_usage(routes, temporary_roads,soil_amount):
    def get_distance(a, b,soil):
        """current と neighbor の3次元距離を計算"""
        horizontal_distance = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        vertical_distance = abs(soil[int(a[0])][int(a[1])] - soil[int(b[0])][int(b[1])])
        return (horizontal_distance**2 + vertical_distance**2)**0.5
    def road_length(temp,soil_amount):
        DIRECTIONS = [
        (-1, -1),  # 0
        (-1, 0),   # 1
        (-1, 1),   # 2
        (0, -1),   # 3
        (0, 1),    # 4
        (1, -1),   # 5
        (1, 0),    # 6
        (1, 1)     # 7
        ]   
        """
        仮設道路の総長を計算する関数。
        
        Parameters:
            temporary_roads (dict): 各座標における方向情報。
            directions (list): 方向ベクトルのリスト。
        
        Returns:
            float: 仮設道路の総長。
        """
        validate_dict(temp)
        total_length = 0
        for start, dir_set in temp.items():
            for direction in dir_set:
                # 各方向の移動量を取得
                dx, dy = DIRECTIONS[direction]
                # print("dx,dy",dx,dy)
                # 仮設道路の長さはベクトルの長さとして計算
                horizontal_length = ((dx/2)**2 + (dy/2)**2)**0.5
                # print("start",start)
                # print("dx,dy",dx,dy)
                vertical_length = abs(soil_amount[int(start[0])][int(start[1])] - soil_amount[int(start[0] + dx)][int(start[1] + dy)])/2
                # print("length",length)
                total_length += (horizontal_length**2+vertical_length**2)**0.5
        # print("total_length",total_length)
        return total_length
    soil_amount_copy = copy.deepcopy(soil_amount)
    temp_roads_copy = copy.deepcopy(temporary_roads)
    temp_length = road_length(temporary_roads,soil_amount_copy)
    total_usage = 0
    DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    # 仮設道路データを破壊しないようにコピーを作成

    for route in routes:
        # print("temp_roads before",temp_roads_copy)
        # print("route",route)
        total_usage_route = 0
        for i in range(len(route) - 1):
            current = tuple(map(int, route[i]))  # 現在の座標
            neighbor = tuple(map(int, route[i + 1]))  # 次の座標

            # 移動方向を計算
            direction = (neighbor[0] - current[0], neighbor[1] - current[1])
            direction_length = get_distance(current, neighbor,soil_amount_copy)
            current_direction_index = DIRECTIONS.index(direction)
            neighbor_direction_index = DIRECTIONS.index((-direction[0], -direction[1]))
            
            # 仮設道路の利用を確認
            if current in temp_roads_copy and current_direction_index in temp_roads_copy[current]:
                # print(f"{current}で仮設道路を利用")
                total_usage_route += direction_length/2 # 長さを加算
            if neighbor in temp_roads_copy and neighbor_direction_index in temp_roads_copy[neighbor]:
                # print(f"{neighbor}で仮設道路を利用")
                total_usage_route += direction_length/2 # 長さを加算
        """指定されたセル（current, neighbor）から仮設道路を削除"""

        start = tuple(map(int, route[0]))
        goal = tuple(map(int, route[-1]))
        for cell in [start, goal]:
            if cell in temp_roads_copy:
                del temp_roads_copy[cell]
        soil_amount_copy[int(start[0])][int(start[1])] -= 1
        soil_amount_copy[int(goal[0])][int(goal[1])] += 1
        # print("temp_roads after",temp_roads_copy)
        # print("total_length_route",total_length_route)
        # print("\n")
        total_usage += total_usage_route
    # print("total_usage",total_usage)
    # print("temp_length",temp_length)
    return total_usage/temp_length

def plot(grid_size_x,grid_size_y, route,cut_indices,fill_indices):  #結果の可視化(土砂割り当て)



    # 格子点の座標を生成
    x = [i for i in range(grid_size_x)]
    y = [i for i in range(grid_size_y)]
    X, Y = np.meshgrid(x, y)

    # プロット用の格子点を設定
    x_coords = X.flatten()
    y_coords = Y.flatten()

    # 土量マトリックスを作成（仮に色付けのためのデータを用意）
    soil_amount = np.zeros((grid_size_y, grid_size_x))
    # print("max_soil",max_soil)
    for [(i, j),k] in cut_indices:
        soil_amount[int(j), int(i)] = 1
    for [(i, j),k] in fill_indices:
        soil_amount[int(j),int(i)] = 0

    soil_amount_real = np.zeros((grid_size_y, grid_size_x))
    for [(i, j),k] in cut_indices:
        soil_amount_real[int(j), int(i)] = 2*k
    for [(i, j),k] in fill_indices:
        soil_amount_real[int(j),int(i)] = -2*k
    # print("soil_amount",soil_amount)


    def init_animate (): #初期化関数(これがないとanimate関数のi=0が2回繰り返される)
        pass

    def animate(i):
        # 前のフレームの矢印をクリア
        ax.clear()

        print("i:",i)

        #土量を更新

        arrows_i = route[i]
        start_point = arrows_i[0]
        end_point = arrows_i[-1]


        soil_amount_real[int(start_point[1]), int(start_point[0])] -= 1  # 開始点の土量を減少
        soil_amount_real[int(end_point[1]), int(end_point[0])] += 1 # 終了点の土量を増加

        if soil_amount_real[int(start_point[1]), int(start_point[0])] == 0:
            soil_amount[int(start_point[1]), int(start_point[0])] = 0.5
        if soil_amount_real[int(end_point[1]), int(end_point[0])] == 0:
            soil_amount[int(end_point[1]), int(end_point[0])] = 0.5
    # print("soil_amount_real",soil_amount_real)
        print("soil_amount_real",soil_amount_real)

        # グリッドの描画（背景）
        ax.pcolormesh(soil_amount, edgecolors='gray', linewidth=2, cmap='viridis', shading='flat', alpha=0.2)
        ax.scatter(x_coords+0.5, y_coords+0.5, color='blue', marker='o')  # 格子点のプロット

        # 格子点のラベルを表示（x, y方向に0.5ずらす）
        for x_val, y_val in zip(x_coords, y_coords):
            ax.text(x_val + 0.5, y_val + 0.4, f'{int(soil_amount_real[y_val][x_val])}', fontsize=12, ha='center', va='top')
        



        #矢印を描画
        if i <=2 or i == len(route)-1: #最初と最後の矢印の描画
            for j in range(i + 1):
                # print("j", j)
                #切土から盛土への矢印
                arrows_j = route[j]
                # print("j%2==0")
                # print("arrows_j", arrows_j)
                if len(arrows_j) == 2:
                    start_point, end_point = arrows_j
                    adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                    adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                    ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                    arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))

                elif len(arrows_j) > 2: #中継点がある場合
                    for k in range(len(arrows_j) - 1):
                        start_point = arrows_j[k]
                        end_point = arrows_j[k+1]
                        adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                        adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                        ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                                    arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))
                # 矢印の番号を矢印の中心に表示
                mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                ax.text(mid_point[0], mid_point[1]+0.1, f'{j//2+1}', fontsize=12, ha='center', color='black')





        else: #i>2での矢印の描画(描画される矢印が2つまでにする)
            for j in range(i - 1,i + 1):
                #切土から盛土への矢印
                
                arrows_j = route[j]
                # print("j%2==0")
                # print("arrows_j", arrows_j)
                if len(arrows_j) == 2:
                    start_point, end_point = arrows_j
                    adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                    adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                    ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                            arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))
                elif len(arrows_j) > 2:
                    for k in range(len(arrows_j) - 1):
                        start_point = arrows_j[k]
                        end_point = arrows_j[k+1]

                        adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                        adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                        ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                            arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))
                # 矢印の番号を矢印の中心に表示
                mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                ax.text(mid_point[0], mid_point[1]+0.1, f'{j//2+1}', fontsize=12, ha='center', color='black')
                    
    # アニメーションの準備
    fig, ax = plt.subplots(figsize=(8, 6))

    # アニメーションの実行
    ani = animation.FuncAnimation(fig, animate,init_func=init_animate, frames=len(route)  , interval=1000, repeat=False)

    # GIFや動画として保存したい場合
    ani.save('animation.gif', writer='Pillow')

    # アニメーションを表示
    plt.show()
    return ani
    



def plot_route(grid_size_x,grid_size_y, path_list,temporary_roads,cut_indices,fill_indices):  #結果の可視化（経路）

    # 格子点の座標を生成
    x = [i for i in range(grid_size_x)]
    y = [i for i in range(grid_size_y)]
    X, Y = np.meshgrid(x, y)

    DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    
    # 仮設道路の情報をコピーして操作
    # modified_temporary_roads_list = {
    #     key: directions.copy() for key, directions in temporary_roads.items()
    # }
    modified_temporary_roads = copy.deepcopy(temporary_roads)
    # プロット用の格子点を設定
    x_coords = X.flatten()
    y_coords = Y.flatten()

    # 土量マトリックスを作成（仮に色付けのためのデータを用意）
    soil_amount = np.zeros((grid_size_y, grid_size_x))
    # print("max_soil",max_soil)
    for [(i, j),k] in cut_indices:
        soil_amount[int(j), int(i)] = 1
    for [(i, j),k] in fill_indices:
        soil_amount[int(j),int(i)] = 0
    soil_amount_copy = soil_amount.copy()

    soil_amount_real = np.zeros((grid_size_y, grid_size_x))
    for [(i, j),k] in cut_indices:
        soil_amount_real[int(j), int(i)] = k
    for [(i, j),k] in fill_indices:
        soil_amount_real[int(j),int(i)] = -k
    # print("soil_amount",soil_amount)
    soil_amount_real_copy = soil_amount_real.copy()

    def init_animate (): #初期化関数(これがないとanimate関数のi=0が2回繰り返される)
        pass


    def draw_temporary_roads(ax,temp_list,start_point,end_point):
        # 仮設道路の描画
        #start_pointとend_point上の仮設道路は描画しない
        # print("start_point",start_point)
        # print("end_point",end_point)
        # print("modified_temporary_roads_before",temp)

        # print("modified_temporary_roads_after",temp)
        for temp in temp_list:
            for start, directions in temp.items():
                for direction in directions:
                    dx, dy = DIRECTIONS[direction]
                    end = (start[0] + dx/2, start[1] + dy/2)
                    adjusted_start = (start[0] + 0.5, start[1] + 0.5)
                    adjusted_end = (end[0] + 0.5, end[1] + 0.5)
                    ax.plot(
                        [adjusted_start[0], adjusted_end[0]],
                        [adjusted_start[1], adjusted_end[1]],
                        color="grey",
                        linewidth=10,
                        alpha=0.7,
                    )
        
        for temp in temp_list:
            if start_point != None and end_point != None:
                for point in [start_point, end_point]:  
                    if point in temp:
                        del temp[point]
        filtered_temp_list = []
        for temp in temp_list:
            if temp:  # 空でない場合のみ追加
                filtered_temp_list.append(temp)
        # print("filtered_temp_list",filtered_temp_list)
        temp_list = filtered_temp_list

    def animate(i):
        ax.clear()
        # print("i+1",i+1)
            #土量を更新
        if i <= len(path_list)-1: 
            arrows_i = path_list[i]
            start_point = arrows_i[0]
            end_point = arrows_i[-1]

            # 仮設道路の描画
            draw_temporary_roads(ax,modified_temporary_roads_copy,start_point,end_point)

            # for start_point, end_point in zip(arrows_i[:-1], arrows_i[1:]):
            start_point = arrows_i[0]
            end_point = arrows_i[-1]
            soil_amount_real_copy_copy[int(start_point[1]), int(start_point[0])] -= 1
            soil_amount_real_copy_copy[int(end_point[1]), int(end_point[0])] += 1
            if soil_amount_real_copy_copy[int(start_point[1]), int(start_point[0])] == 0:
                soil_amount_copy_copy[int(start_point[1]), int(start_point[0])] = 0.5
            if soil_amount_real_copy_copy[int(end_point[1]), int(end_point[0])] == 0:
                soil_amount_copy_copy[int(end_point[1]), int(end_point[0])] = 0.5
            # グリッドの描画（背景）途中経過では土量が変化する
            ax.pcolormesh(soil_amount_copy_copy, edgecolors='gray', linewidth=2, cmap='viridis', shading='flat', alpha=0.2)
            ax.scatter(x_coords+0.5, y_coords+0.5, color='blue', marker='o')  # 格子点のプロット
                    # 格子点のラベルを表示（x, y方向に0.5ずらす）
            for x_val, y_val in zip(x_coords, y_coords):
                ax.text(x_val + 0.5, y_val + 0.4, f'{int(soil_amount_real_copy_copy[y_val][x_val])}', fontsize=12, ha='center', va='top')
        else:
            # グリッドの描画（背景）最後のステップでは元の土量を表示
            draw_temporary_roads(ax,temporary_roads,None,None)
            ax.pcolormesh(soil_amount, edgecolors='gray', linewidth=2, cmap='viridis', shading='flat', alpha=0.2)
            ax.scatter(x_coords+0.5, y_coords+0.5, color='blue', marker='o')  # 格子点のプロット
        # 格子点のラベルを表示（x, y方向に0.5ずらす）
            for x_val, y_val in zip(x_coords, y_coords):
                ax.text(x_val + 0.5, y_val + 0.4, f'{int(soil_amount_real[y_val][x_val])}', fontsize=12, ha='center', va='top')

        

        #矢印を描画
        if i >= len(path_list): #最後の矢印の描画
            i = len(path_list)-1
            for j in range(i + 1):
                #切土から盛土への矢印
                arrows_j = path_list[j]
                for k, (start_point, end_point) in enumerate(zip(arrows_j[:-1], arrows_j[1:])):
                    adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                    adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                    ax.annotate(
                        "",
                        xy=adjusted_end,
                        xytext=adjusted_start,
                        arrowprops=dict(facecolor="red", edgecolor="black", linewidth=2, alpha=0.7, shrink=0.05),
                    )
                                        # 矢印の番号を矢印の中心に表示
                    mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                    ax.text(mid_point[0], mid_point[1]+0.1, f'({j+1})', fontsize=12, ha='center', color='grey')
                       

        else: #最後以外の矢印の描画(描画される矢印が1つまでにする)
            j = i
            #切土から盛土への矢印
            arrows_j = path_list[j]
            for k, (start_point, end_point) in enumerate(zip(arrows_j[:-1], arrows_j[1:])):
                adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)        
                ax.annotate(
                    "",
                    xy=adjusted_end,
                    xytext=adjusted_start,
                    arrowprops=dict(facecolor="red", edgecolor="black", linewidth=2, alpha=0.7, shrink=0.05),
                )
                mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                ax.text(mid_point[0], mid_point[1]+0.1, f'({j+1})', fontsize=12, ha='center', color='grey')                        

    # アニメーションの準備
    fig, ax = plt.subplots(figsize=(8, 6))
    # アニメーションの実行
    ani = animation.FuncAnimation(fig, animate,init_func=init_animate, frames=len(path_list)+1 , interval=1500, repeat=False,blit=False)
    
    soil_amount_real_copy_copy = soil_amount_real_copy.copy() #gif保存用の土量と表示用の土量を作成
    soil_amount_copy_copy = soil_amount_copy.copy()
    # modified_temporary_roads_copy = {
    #     key: directions.copy() for key, directions in modified_temporary_roads.items()
    # }
    modified_temporary_roads_copy = copy.deepcopy(modified_temporary_roads)
    # GIFや動画として保存したい場合
    ani.save('animation_ver2.gif', writer='Pillow')

    soil_amount_real_copy_copy = soil_amount_real_copy.copy()
    soil_amount_copy_copy = soil_amount_copy.copy()
    modified_temporary_roads_copy = copy.deepcopy(modified_temporary_roads)
    # アニメーションを表示
    plt.show()




def plot_solution_flow(grid_size_x,grid_size_y,roads_flow,cut_indices,fill_indices):  #結果の可視化（経路）

    # 格子点の座標を生成
    x = [i for i in range(grid_size_x)]
    y = [i for i in range(grid_size_y)]
    X, Y = np.meshgrid(x, y)

    DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    
    # 仮設道路の情報をコピーして操作
    # modified_temporary_roads_list = {
    #     key: directions.copy() for key, directions in temporary_roads.items()
    # }
    roads_flow_copy = copy.deepcopy(roads_flow)

    # プロット用の格子点を設定
    x_coords = X.flatten()
    y_coords = Y.flatten()

    # 土量マトリックスを作成（仮に色付けのためのデータを用意）
    soil_amount = np.zeros((grid_size_y, grid_size_x))
    # print("max_soil",max_soil)
    for [(i, j),k] in cut_indices:
        soil_amount[int(j), int(i)] = 1
    for [(i, j),k] in fill_indices:
        soil_amount[int(j),int(i)] = 0
    soil_amount_copy = soil_amount.copy()

    soil_amount_real = np.zeros((grid_size_x, grid_size_y))
    for [(i, j),k] in cut_indices:
        soil_amount_real[int(j), int(i)] = k
    for [(i, j),k] in fill_indices:
        soil_amount_real[int(j),int(i)] = -k
    # print("soil_amount",soil_amount)
    soil_amount_real_copy = soil_amount_real.copy()

    def init_animate (): #初期化関数(これがないとanimate関数のi=0が2回繰り返される)
        pass

    def animate(i):
        ax.clear()
        temp_list = roads_flow[i][1]
        for temp in temp_list:
            for start, directions in temp.items():
                for direction in directions:
                    dx, dy = DIRECTIONS[direction]
                    end = (start[0] + dx/2, start[1] + dy/2)
                    adjusted_start = (start[0] + 0.5, start[1] + 0.5)
                    adjusted_end = (end[0] + 0.5, end[1] + 0.5)
                    ax.plot(
                        [adjusted_start[0], adjusted_end[0]],
                        [adjusted_start[1], adjusted_end[1]],
                        color="grey",
                        linewidth=10,
                        alpha=0.7,
                    )
        
        # グリッドの描画（背景）
        ax.pcolormesh(soil_amount, edgecolors='gray', linewidth=2, cmap='viridis', shading='flat', alpha=0.2)
        ax.scatter(x_coords+0.5, y_coords+0.5, color='blue', marker='o')  # 格子点のプロット
        # 格子点のラベルを表示（x, y方向に0.5ずらす）
        for x_val, y_val in zip(x_coords, y_coords):
            ax.text(x_val + 0.5, y_val + 0.4, f'{int(soil_amount_real_copy[y_val][x_val])}', fontsize=12, ha='center', va='top')
                       

    # アニメーションの準備
    fig, ax = plt.subplots(figsize=(8, 6))
    # アニメーションの実行
    ani = animation.FuncAnimation(fig, animate,init_func=init_animate, frames=len(roads_flow) , interval=500, repeat=False,blit=False)
    # GIFや動画として保存したい場合
    ani.save('road_flow.gif', writer='Pillow')
    # アニメーションを表示
    plt.show()
