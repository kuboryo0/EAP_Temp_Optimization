#simulated annealingを用いた仮設道路デザインの最適化
#地形変化も考慮。土砂の割り当て方は事前に計算してから仮設道路の最適化を行う
import numpy as np
import random
import pulp
import time
import math
import copy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq
import time

#solution = [[[座標ペア],[タイミング]],...], 道路の座標:道路の部分。2つの隣接座標からなる。タイミング= [0,1,0,1,0,1,0,1] 0:未建設 1:建設済み
#current_temps = [[[[道路の座標1,0],[道路の座標2,0]]],...] 0:未建設 1:建設済み

DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
grid_size_x = 4
grid_size_y = 4
temp_eff = 0.5
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
    # print("ステータス:", pulp.LpStatus[prob.status])
    # print("最小コスト:", pulp.value(prob.objective))
    # 複数の矢印を描画するための例
    routes = []

    # 解の表示
    for t in range(T):
        for f in range(num_fill):
            for c in range(num_cut):
                if pulp.value(x_vars[t][c][f]) > 0.5:
                    routes.append((cut_indices[c][0],fill_indices[f][0]))
    return routes

def a_star_alogrithm(start_goal_pairs,soil_amount,solution):
    def temp_in_step(current_temps,step,solution,built_length_list):
        def pair_length(pair,soil_amount):

            horizontal_distance = ((pair[0][0] - pair[1][0]) ** 2 + (pair[0][1] - pair[1][1]) ** 2) ** 0.5
            vertical_distance = soil_amount[int(pair[0][0])][int(pair[0][1])] - soil_amount[int(pair[1][0])][int(pair[1][1])]   
            return (horizontal_distance**2 + vertical_distance**2)**0.5
        current_temps_copy = copy.deepcopy(current_temps)
        built_length = 0
        for i,(pair,timing) in enumerate(solution):
            if timing[step] == 1:
                if current_temps[i][0][1] == 0:
                    length = pair_length(pair,soil_amount)/2
                    built_length += length
                    built_length_list[i] += length
                    current_temps_copy[i][0][1] = 1
                if current_temps[i][1][1] == 0:
                    length = pair_length(pair,soil_amount)/2
                    built_length += length
                    built_length_list[i] += length
                    current_temps_copy[i][1][1] = 1
        return current_temps_copy,built_length

    #timingに基づくタイミングで道路を建設した場合のコストを計算する
    def get_distance(a, b,soil):
        """current と neighbor の3次元距離を計算"""
        horizontal_distance = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        vertical_distance = abs(soil[int(a[0])][int(a[1])] - soil[int(b[0])][int(b[1])])
        return (horizontal_distance**2 + vertical_distance**2)**0.5

    def heuristic(a, b,soil_amount):
        """ユークリッド距離のヒューリスティック関数"""
        return temp_eff * get_distance(a, b,soil_amount)

    def get_cost(current, neighbor,soil_amount,temporary_roads):
        """移動コストを計算"""
        distance = get_distance(current, neighbor,soil_amount)
        current_distance = distance/2
        neighbor_distance = distance/2
        for temp in temporary_roads:
            if current == temp[0][0] and neighbor == temp[1][0]:
                if temp[0][1] == 1:
                    current_distance *= temp_eff
                if temp[1][1] == 1:
                    neighbor_distance *= temp_eff
            elif current == temp[1][0] and neighbor == temp[0][0]:
                if temp[0][1] == 1:
                    neighbor_distance *= temp_eff
                if temp[1][1] == 1:
                    current_distance *= temp_eff
        return current_distance + neighbor_distance
    
    def hasUsedRoad(path,temp):
        """経路上で仮設道路を利用したかどうか"""
        coord1 = temp[0][0]
        coord2 = temp[1][0]
        for i in range(len(path) - 1):
            current = path[i]
            next = path[i + 1]
            if (coord1 == current and coord2 == next) or (coord1 == next and coord2 == current):
                return True
        return False

    def remove_temporary_roads(start, goal,temps):
        """指定されたセル（start, goal）から仮設道路を削除"""
        for cell in [start, goal]:
            for i,temp in enumerate(temps):
                for j,(coord,_) in enumerate(temp):
                    if coord == cell:
                        temps[i][j][1] = 0
                        break

    def change_soil(start,goal,soil_amount):
        """指定されたセル（start, goal）から土量を変更"""
        soil_amount[int(start[0])][int(start[1])] -= 1
        soil_amount[int(goal[0])][int(goal[1])] += 1

    def astar(start, goal,temp,soil_amount,used_temp_list):
        temp_copy =copy.deepcopy(temp)
        soil_amount_copy = copy.deepcopy(soil_amount)
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

                new_cost = cost_so_far[current] + get_cost(current, neighbor,soil_amount_copy,temp_copy)

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

        #経路上で仮設道路をどれくらい利用したかを記録
        used_temp_list_inStep = [0]*len(temp_copy)
        for temp_index,temp in enumerate(temp_copy):
            if temp[0][1] == 1 or temp[1][1] == 1:
                if hasUsedRoad(path,temp):
                    used_distance = get_distance(temp[0][0],temp[1][0],soil_amount_copy)
                    used_distance *= 0.5 if not (temp[0][1] == 1 and temp[1][1] == 1) else 1
                    used_temp_list_inStep[temp_index] = used_distance
            
        used_temp_list.append(used_temp_list_inStep)
        # 仮設道路を削除
        remove_temporary_roads(start, goal,temp_copy)
        for i,temp in enumerate(temp_copy):
            if temp is not None:
                if len(temp) == 0:
                   temp_copy[i] = None
        change_soil(start,goal,soil_amount_copy)
        # print(f"Temporary Roads After Removal: {temp_copy}")

        return path, cost_so_far[goal],temp_copy,soil_amount_copy
    
    total_cost = 0
    path_list = []
    current_temporary = [[[solution[i][0][0],0],[solution[i][0][1],0]] for i in range(len(solution))]
    soil_amount_deepcopy = copy.deepcopy(soil_amount)
    total_built_length = 0
    used_temp_list = []
    built_length_list = [0]*len(solution)
    for i,(start, goal) in enumerate(start_goal_pairs):
        current_temporary,built_length = temp_in_step(current_temporary,i,solution,built_length_list)
        total_built_length += built_length
        path, cost,current_temporary,soil_amount_deepcopy = astar(start, goal,current_temporary,soil_amount_deepcopy,used_temp_list)
        total_cost += cost
        path_list.append(path)
        # print("current_temporary after",current_temporary)
        # print(f"Start: {start}, Goal: {goal}")
        # print(f"Path: {path}")
        # print(f"Cost: {cost}\n")
        # print("\n")
    # print(f"Total Cost: {total_cost}")
    return path_list,total_cost,used_temp_list,total_built_length,built_length_list

def evaluate_design(solution):
    "工事作業（時間依存）のコスト"
    #定数の値
    grid_length =150 # 1マスの１辺の長さ(m)
    volume_unit_truck = 30 # トラック1台あたりの土砂輸送量(m3)
    depth_unit = 1 #土量1単位の深さ(m)
    speed_rough_road = 24 # 未舗装道路の速度(km/h)
    speed_paved_road = 36 # 舗装道路の速度(km/h)
    temp_eff = speed_rough_road/speed_paved_road # 仮設道路の効率
    work_eff = 0.75 # 作業効率
    truck_num = 1 # トラックの台数
    cost_hour = 2000# 1時間あたりのコスト($/h)
    #コスト計算
    route,cost1,used_road_flow,road_length,built_length_list = a_star_alogrithm(allocation,soil_amount,solution)
    # print("運搬距離コスト",cost1)
    soil_volume =  grid_length**2 * depth_unit # 1ブロックの土量(m3)
    time_average = cost1 * grid_length / (speed_rough_road*1000) # 1往復のみと仮定した時のn台トラックの所要時間(h) 
    cost_construction = cost_hour * soil_volume/(truck_num * volume_unit_truck) * time_average / work_eff # 作業コスト($)
    "仮設道路の建設にかかるコスト"
    construction_temp = 17.5 # 仮設道路の建設単位長さあたりのコスト($/m)
    #コスト計算
    cost2 = road_length
    # print("仮設道路の長さ",cost2)
    cost_construction_temp = cost2 * construction_temp * grid_length# 仮設道路の建設コスト($)
    #総コスト
    total_cost = cost_construction + cost_construction_temp

    print("cost_construction:$",cost_construction)
    print("cost_construction_temp:$",cost_construction_temp)
    print("total_cost:$",total_cost)
    return route,total_cost,used_road_flow,built_length_list

def generate_neighbor(current_solution,temp_usage_flow,allocation,built_length_list):
    #新たな解を生成
    temp_lists = [[current_solution[i][0][0],current_solution[i][0][1]] for i in range(len(current_solution))]
    total_step_num = len(allocation)
    current_solution_copy = copy.deepcopy(current_solution)
    # 仮設道路の使用量を計算
    usage_weights = list([current_solution[i],0] for i in range(len(current_solution)))
    for i in range(len(usage_weights)):
        for j in range(len(temp_usage_flow)):
            usage_weights[i][1] += temp_usage_flow[j][i]
        if built_length_list[i] != 0:
            usage_weights[i][1] /= built_length_list[i]
        else:
            usage_weights[i][1] = 0
    
    # 使用量が低い道路のリストを作成
    low_usage_roads = sorted(usage_weights, key=lambda x: x[1])  # 使用量が低い順にソート
    random_num = random.random()
    if len(low_usage_roads) == 0 or random_num < 0.15 : 
        #道路を追加
        while True:
            coord1 = (random.randint(0, grid_size_x)-1, random.randint(0, grid_size_y)-1)
            dx, dy = random.choice(DIRECTIONS)
            coord2 = (coord1[0] + dx, coord1[1] + dy)
            isSame = any((coord1 in temp_list and coord2 in temp_list) for temp_list in temp_lists)
            if not isSame and (0 <= coord2[0] < grid_size_x and 0 <=coord2[1] < grid_size_y):
                break
        new_timing = []
        for i in range(total_step_num):
            new_timing.append(random.randint(0,1)) 
        current_solution_copy.append([[coord1,coord2],new_timing])
    else:
        total_weight = sum(1 / (usage + 1e-6) for _, usage in low_usage_roads)  # 小さい値を追加してゼロ除算を防止
        probabilities = [(1 / (usage + 1e-6)) / total_weight for _, usage in low_usage_roads]
        # 確率に基づいて選択
        road_index = np.random.choice(len(low_usage_roads), p=probabilities)
        if len(low_usage_roads) == 1 or random_num < 0.60:
            if random_num < 0.3:
                timing_choice = random.randint(0,total_step_num-1)
                current_solution_copy[road_index][1][timing_choice] ^= 1 
                for i in range(len(current_solution_copy[road_index][1])):
                    if random.random() < 0.2:
                        current_solution_copy[road_index][1][i] ^= 1
            elif random_num < 0.45:
                current_solution_copy.remove(low_usage_roads[road_index][0])
            else:
                # 道路を変更
                while True:
                    coord1 = (random.randint(0, grid_size_x), random.randint(0, grid_size_y))
                    dx, dy = random.choice(DIRECTIONS)
                    coord2 = (coord1[0] + dx, coord1[1] + dy)
                    isSame = any((coord1 in temp_list and coord2 in temp_list) for temp_list in temp_lists)
                    if not isSame and (0 <= coord2[0] < grid_size_x and 0 <=coord2[1] < grid_size_y):
                        break
                current_solution_copy[road_index][0] = [coord1,coord2]
        else:
            # 2つ目の道路を選択
            while True:
                road_index2 = np.random.choice(len(low_usage_roads))
                if road_index2 != road_index:
                    break
            if random_num < 0.75:
                # 点を交換
                random_index1 = random.randint(0,1)
                random_index2 = random.randint(0,1)
                for i in range(2):
                    for j in range(2):
                        if current_solution_copy[road_index][0][i] == current_solution_copy[road_index2][0][j]:
                            random_index1,random_index2 = 1-i,1-j
                            break
                current_solution_copy[road_index][0][random_index1],current_solution_copy[road_index2][0][random_index2] = current_solution_copy[road_index2][0][random_index2],current_solution_copy[road_index][0][random_index1]
            elif random_num < 0.9:
                # 建設タイミングを交換
                current_solution_copy[road_index][1],current_solution_copy[road_index2][1] = current_solution_copy[road_index2][1],current_solution_copy[road_index][1]
            else:
                #建設タイミングを同化
                current_solution_copy[road_index][1] = current_solution_copy[road_index2][1]
    
    return current_solution_copy

def simulated_annealing(allocation,timing):
    current_solution = timing
    current_route,current_score,current_used_flow,current_built_length_list = evaluate_design(current_solution)
    best_solution = current_solution
    best_score = current_score
    best_route = current_route
    # best_used_flow = current_used_flow
    temperature = initial_temprature
    best_score_loop = 0
    best_solution_flow = []
    current_solution_flow = []
    neighbor_solution_flow = []
    print("\n")

    for _ in range(max_iter):
        print("simulated anealing loop",_)
        temperature *= alpha
        # current_route_copy = copy.deepcopy(current_route)
        current_solution_copy = copy.deepcopy(current_solution)
        neighbor_solution= generate_neighbor(current_solution_copy,current_used_flow,allocation,current_built_length_list)
        print("neighbor_solution",neighbor_solution)
        
        neighbor_route,neighbor_score,neighbor_used_flow,neighbor_built_length_list = evaluate_design(neighbor_solution)
        print("soil_amount",soil_amount)
        # 新しい解が良い場合、悪くても確率的に受け入れる
        if (neighbor_score < current_score) or (random.random() < np.exp(-( np.abs(neighbor_score - current_score) ) / temperature)):
            current_solution = neighbor_solution
            current_score = neighbor_score
            current_route = neighbor_route
            current_used_flow = neighbor_used_flow
            current_built_length_list = neighbor_built_length_list

            # ベスト解を更新
        if current_score < best_score:
            best_solution = current_solution
            best_score = current_score
            best_route = current_route
            best_built_length_list = current_built_length_list
            best_used_flow = current_used_flow
            best_score_loop = _
        best_solution_flow.append([best_score, best_solution])
        current_solution_flow.append([current_score, current_solution])
        neighbor_solution_flow.append([neighbor_score, neighbor_solution])        
        best_solution_flow.append([best_score, best_solution])
        current_solution_flow.append([current_score, current_solution])
        neighbor_solution_flow.append([neighbor_score, neighbor_solution])
        print("neighbor_solution",neighbor_solution)
        print("neighbor_score",neighbor_score)
        print("current_solution",current_solution)
        print("current_score",current_score)
        print("best_solution",best_solution)
        print("best_score",best_score)
        print("\n")
        # 温度を下げる

    return best_solution, best_score, best_route,best_solution_flow,current_solution_flow,neighbor_solution_flow,best_score_loop

def plot_route( path_list,temp_solution,cut_indices,fill_indices):  #結果の可視化（経路）
    # 格子点の座標を生成
    x = [i for i in range(grid_size_x)]
    y = [i for i in range(grid_size_y)]
    X, Y = np.meshgrid(x, y)
    current_temp = []
    all_temp = []
    for temp in temp_solution:
        current_temp.append([[temp[0][0],0],[temp[0][1],0]])
        all_temp.append([[temp[0][0],1],[temp[0][1],1]])
    # プロット用の格子点を設定
    x_coords = X.flatten()
    y_coords = Y.flatten()

    # 土量マトリックスを作成（仮に色付けのためのデータを用意）
    soil_amount = np.zeros((grid_size_y, grid_size_x))
    for [(i, j),k] in cut_indices:
        soil_amount[int(j), int(i)] = 1
    for [(i, j),k] in fill_indices:
        soil_amount[int(j),int(i)] = 0
    soil_amount_copy = copy.deepcopy(soil_amount)

    soil_amount_real = np.zeros((grid_size_y, grid_size_x))
    for [(i, j),k] in cut_indices:
        soil_amount_real[int(j), int(i)] = k
    for [(i, j),k] in fill_indices:
        soil_amount_real[int(j),int(i)] = -k
    soil_amount_real_copy = copy.deepcopy(soil_amount_real)

    def init_animate (): #初期化関数(これがないとanimate関数のi=0が2回繰り返される)
        pass

    def temp_before_step(current_temps,step,solution):
        current_temps_copy = copy.deepcopy(current_temps)
        new_temp = []
        for temp in solution:
            new_temp.append([[temp[0][0],0],[temp[0][1],0]])
        for i,(pair,timing) in enumerate(solution):
            if timing[step] == 1:
                if current_temps_copy[i][0][1] == 0:
                    new_temp[i][0][1] = 1
                if current_temps_copy[i][1][1] == 0:
                    new_temp[i][1][1] = 1
                current_temps_copy[i][0][1] = 1
                current_temps_copy[i][1][1] = 1
        return current_temps_copy,new_temp

    def draw_temporary_roads_in_step(ax,current_road,start_point,end_point):
            # 仮設道路の描画
        current_road_copy = copy.deepcopy(current_road)
        if current_road_copy!=None:
            if start_point != None and end_point != None:
                for i,temp in enumerate(current_road_copy):
                    current_list = temp[0]
                    next_list = temp[1]
                    current = current_list[0]
                    next = next_list[0]
                    if current == start_point or current == end_point:
                        current_road_copy[i][0][1] = 0
                    if next == start_point or next == end_point:
                        current_road_copy[i][1][1] = 0
            for temp in current_road_copy:
                    current_list = temp[0]
                    next_list = temp[1]
                    current = current_list[0]
                    next = next_list[0]
                    adjusted_current = (current[0] + 0.5, current[1] + 0.5)
                    adjested_next = (next[0] + 0.5, next[1] + 0.5)
                    adjusted_middle  = ((current[0] + next[0])/2 + 0.5, (current[1] + next[1])/2 + 0.5)
                    if current_list[1] == 1:
                        ax.plot(
                            [adjusted_current[0], adjusted_middle[0]],
                            [adjusted_current[1], adjusted_middle[1]],
                            color="grey",
                            linewidth=10,
                            alpha=0.7,
                        )
                    if next_list[1] == 1:
                        ax.plot(
                                [adjusted_middle[0], adjested_next[0]],
                                [adjusted_middle[1], adjested_next[1]],
                                color="grey",
                                linewidth=10,
                                alpha=0.7,
                            )

            return current_road_copy
    def animate(i):
        nonlocal current_temp_copy
        ax.clear()
        # 最終フレーム：全てを描画
        if i >= len(path_list) * 2:
            draw_temporary_roads_in_step(ax, all_temp, None, None)
            # path_list内のすべての矢印を描画
            for step, arrows_i in enumerate(path_list):
                for k, (start_point, end_point) in enumerate(zip(arrows_i[:-1], arrows_i[1:])):
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
                    ax.text(mid_point[0], mid_point[1] + 0.1, f'({step + 1})', fontsize=12, ha='center', color='grey')
        # 偶数フレーム：仮設道路のみを描画
        elif i % 2 == 0:
            current_temp_copy,new_temp = temp_before_step(current_temp_copy, i // 2, temp_solution)  # temp_before_stepを適用
            draw_temporary_roads_in_step(ax, current_temp_copy, None, None)  # 仮設道路を描画
        
        # 奇数フレーム：仮設道路 + 矢印 + 土量の描画
        else:
            step = i // 2
            if step < len(path_list):
                arrows_i = path_list[step]
                start_point = arrows_i[0]
                end_point = arrows_i[-1]
                # 仮設道路の描画
                current_temp_copy = draw_temporary_roads_in_step(ax, current_temp_copy, start_point, end_point)
                # 土量の変更
                soil_amount_real_copy[int(start_point[1]), int(start_point[0])] -= 1
                soil_amount_real_copy[int(end_point[1]), int(end_point[0])] += 1
                if soil_amount_real_copy[int(start_point[1]), int(start_point[0])] == 0:
                    soil_amount_copy[int(start_point[1]), int(start_point[0])] = 0.5
                if soil_amount_real_copy[int(end_point[1]), int(end_point[0])] == 0:
                    soil_amount_copy[int(end_point[1]), int(end_point[0])] = 0.5
                
                # 矢印を描画
                for start_point, end_point in zip(arrows_i[:-1], arrows_i[1:]):
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
                    ax.text(mid_point[0], mid_point[1] + 0.1, f'({step + 1})', fontsize=12, ha='center', color='grey')
        # 共通の描画処理
        ax.pcolormesh(soil_amount_copy, edgecolors='gray', linewidth=2, cmap='viridis', shading='flat', alpha=0.2)
        ax.scatter(x_coords + 0.5, y_coords + 0.5, color='blue', marker='o')  # 格子点のプロット
        # 格子点のラベルを表示
        for x_val, y_val in zip(x_coords, y_coords):
            ax.text(x_val + 0.5, y_val + 0.4, f'{int(soil_amount_real_copy[y_val][x_val])}', fontsize=12, ha='center', va='top')

    current_temp_copy = copy.deepcopy(current_temp)
    print("current_temp_copy",current_temp_copy)
    # アニメーションの準備
    fig, ax = plt.subplots(figsize=(8, 6))
    # アニメーションの実行
    ani = animation.FuncAnimation(fig, animate,init_func=init_animate, frames=2*len(path_list)+1 , interval=1500, repeat=False,blit=False)
    
    # GIFや動画として保存したい場合
    ani.save('animation_all.gif', writer='Pillow')

    soil_amount_real_copy = soil_amount_real_copy.copy()
    soil_amount_copy = soil_amount_copy.copy()
    current_temp_copy = copy.deepcopy(current_temp)
    # アニメーションを表示
    plt.show()



cut_indices = [[(1, 0),1],[(1, 2),1],[(2, 0),1],[(2, 1),1],[(2, 2),1],[(2, 3),1],[(3, 0),1],[(3, 1),1],[(3, 3),2]]
#盛土の座標と土量
fill_indices = [[(0, 0),2],[(0, 1),1],[(0, 2),1],[(0, 3),1],[(1, 3),2],[(1, 1),1],[(3, 2),2]]

soil_amount = np.zeros((grid_size_x, grid_size_y))
for [(i, j),k] in cut_indices:
    soil_amount[int(i), int(j)] = k
for [(i, j),k] in fill_indices:
    soil_amount[int(i),int(j)] = -k

solution = [[[(2, 1), (3, 0)], [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]], [[(0, 0), (1, 0)], [0, 0, 1, 1, 0, 0, 0, 0, 0, 0]], [[(2, 2), (1, 2)], [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]], [[(3, 1), (3, 2)], [1, 1, 0, 0, 1, 0, 1, 1, 1, 0]], [[(3, 2), (3, 3)], [0, 0, 1, 1, 0, 0, 0, 0, 0, 0]], [[(0, 1), (1, 1)], [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]], [[(1, 2), (0, 2)], [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]], [[(1, 3), (2, 3)], [1, 1, 0, 0, 1, 0, 1, 1, 1, 0]], [[(0, 3), (1, 2)], [0, 0, 1, 1, 0, 0, 0, 0, 0, 0]], [[(3, 3), (2, 3)], [1, 0, 0, 0, 0, 0, 0, 0, 0, 0]]]

start_time = time.time()
allocation = earth_allocation(cut_indices,fill_indices)
# print("allocation",allocation)
# allocation = [((1,0),(2,1)),((1,0),(2,1))]

print(evaluate_design(solution))
# # パラメータの設定
# initial_temprature = 1000
# # final_temprature = 1
# alpha = 0.99
# max_iter = 10


# best_solution, best_score, best_route,best_solution_flow,current_solution_flow,neighbor_solution_flow,best_score_loop = simulated_annealing(allocation,solution)
# end_time = time.time()
# print("best_solution",best_solution)
# print("best_score",best_score)
# print("best_route",best_route)
# print("best_score_loop",best_score_loop)
# print("time",end_time-start_time)

# plot_route(best_route,best_solution,cut_indices,fill_indices)