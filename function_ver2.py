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
temp_eff = 0.7
v =  1/temp_eff
tan_alpha = math.sqrt(v**2-1)
sin_alpha = math.sqrt(v**2-1)/v
distance = 0
temp = [[(0, 0), (0, 1)],[(1, 1),(2, 2)],[(1, 2),(2, 1)]]





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

def a_star(start_goal_pairs,temporary_roads,grid_size_x,grid_size_y):
    # 各セルが持つ仮設道路情報 (座標: [方向のセット])

    # 移動方向（上下左右および斜め）
    DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    temporary_roads_copy = copy.deepcopy(temporary_roads)
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
        for temp in temporary_roads_copy:
            if direction_index_current in temp.get(current, set()):
                current_distance *=  temp_eff
            if direction_index_neighbor in temp.get(neighbor, set()):
                neighbor_distance *=  temp_eff 
        
        return current_distance + neighbor_distance


    def astar(start, goal):
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
                if not (0 <= neighbor[0] < grid_size_x and 0 <= neighbor[1] < grid_size_y):
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
            if cell in temporary_roads_copy:
                del temporary_roads_copy[cell]
    path_list = []
    total_cost = 0

    # 各ペアに対する最小コスト経路を計算
    for start, goal in start_goal_pairs:
        path, cost = astar(start, goal)
        # print(f"Start: {start}, Goal: {goal}")
        # print(f"Path: {path}")
        # print(f"Cost: {cost}\n")
        # print(f"Temporary Roads After Removal: {temporary_roads_copy}")
        path_list.append(path)
        total_cost += cost
    return path_list, total_cost


def temp_length(temp_list):
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
                length = ((dx/2)**2 + (dy/2)**2)**0.5
                # print("length",length)
                total_length += length
    print("total_length",total_length)
    return total_length

#仮設道路がどれくらい利用されているかを計算
def temp_usage(route_list, temp):
    def on_segment(p:tuple, q:tuple, r:tuple) -> bool:
            """点rが線分pq上にあるかどうかを確認する関数"""
            if min(p[0], q[0]) <= r[0] <= max(p[0], q[0]) and min(p[1], q[1]) <= r[1] <= max(p[1], q[1]):
                return True
            return False

    def on_line(p:tuple, q:tuple, r:tuple):
                delta = 0.000001
                bool = False
                    # ベクトル pr の傾きと qr の傾きを比較,rがpqの間にあるかどうか⇒rがpq上にあるかを確認
                if abs((r[1] - p[1]) * (q[0] - r[0]) - (r[0] - p[0]) * (q[1] - r[1])) < delta and on_segment(p, q, r):
                    bool = True
                return bool
                

    # 2点間の距離を計算する関数
    def distance(point1, point2):
        return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)
    

    # 仮設道路の形式をチェック
    if not (isinstance(temp, list) and len(temp) == 2 and
            all(isinstance(point, tuple) and len(point) == 2 and
                all(isinstance(coord, (int, float)) for coord in point)
                for point in temp)):
        print("仮設道路は1つである必要があります")
        return exit(1)
    
    # 仮設道路の総距離を計算
    temp_distance = distance(temp[0], temp[1])
    
    # 全経路で仮設道路が使われている割合を計算
    total_temp_usage = 0

    for route in route_list:
        route_temp_usage = 0
        
        # 経路上の各セグメントについて距離を計算
        for i in range(len(route) - 1):
            segment1 = route[i]
            segment2 = route[i + 1]
            # print("segment1,segment2",segment1,segment2)
            
            #経路上に仮設道路上がある場合
            if (on_line(segment1,segment2,temp[0]) and on_line(segment1,segment2,temp[1])):
                route_temp_usage += distance(temp[0], temp[1])
            #     print("distance",distance(temp[0], temp[1]))
            #     print("経路上に仮設道路上がある場合")
            # # segmetn1とsegment2が両方仮設道路上にある場合 または segment1とsegment2が仮設道路の端点と一致する場合
            # elif((on_line(temp[0], temp[1], segment1)  and on_line(temp[0], temp[1], segment2))
            #    or ((segment1 == temp[0] and segment2 == temp[1]) or (segment1 == temp[1] and segment2 == temp[0]))):
            #     print("segment1とsegment2が両方仮設道路上にある場合")
            #     route_temp_usage += distance(segment1, segment2)
            elif(on_line(temp[0], temp[1], segment1)  and on_line(temp[0], temp[1], segment2)):
                # print("distance",distance(segment1, segment2))
                # print("segment1とsegment2が両方仮設道路上にある場合")
                route_temp_usage += distance(segment1, segment2)
            # segment1が仮設道路上にあり、segment2が仮設道路上にない場合
            elif (on_line(temp[0], temp[1], segment1) and not on_line(temp[0], temp[1], segment2)):
                if distance(segment2, temp[0]) < distance(segment2, temp[1]): # segment2,temp[0],segment1,temp[1]の順番で並んでいる場合
                    route_temp_usage += distance(segment1, temp[0])
                #     print("distance",distance(segment1, temp[0]))
                #     print("segment2,temp[0],segment1,temp[1]の順番で並んでいる場合")
                else: # temp[0],segment1,temp[1],segment2の順番で並んでいる場合
                    route_temp_usage += distance(segment1, temp[1])
                #     print("distance",distance(segment1, temp[1]))
                #     print("temp[0],segment1,temp[1],segment2の順番で並んでいる場合")
                # print("segment1が仮設道路上にあり、segment2が仮設道路上にない場合")
            #segment2が仮設道路上にあり、segment1が仮設道路上にない場合
            elif (on_line(temp[0], temp[1], segment2) and not on_line(temp[0], temp[1], segment1)):
                if distance(segment1, temp[0]) < distance(segment1, temp[1]): # segment2,temp[0],segment1,temp[1]の順番で並んでいる場合
                    route_temp_usage += distance(segment2, temp[0])
                    # print("distance",distance(segment2, temp[0]))
                    # print("segment1,temp[0],segment2,temp[1]の順番で並んでいる場合")
                else: # temp[0],segment1,temp[1],segment2の順番で並んでいる場合
                    route_temp_usage += distance(segment2, temp[1])
                #     print("distance",distance(segment2, temp[1]))
                #     print("temp[0],segment2,temp[1],segment1の順番で並んでいる場合")
                # print("segment2が仮設道路上にあり、segment1が仮設道路上にない場合")

            else:
                route_temp_usage += 0
                # print("その他の場合")
        # 仮設道路が使われた割合を加算
        total_temp_usage += route_temp_usage


    
    # 仮設道路が使われた割合を仮設道路の距離に対して返す
    if temp_distance == 0:
        return 0
    weight = total_temp_usage / temp_distance
    return total_temp_usage / temp_distance


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
            if start_point != None and end_point != None:
                for point in [start_point, end_point]:  
                    if point in temp:
                        del temp[point]
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
