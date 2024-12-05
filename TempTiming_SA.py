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
# from function_ver2 import plot_route



DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
grid_size_x = 4
grid_size_y = 4
temp_eff = 0.5

def a_star(start_goal_pairs,temporary_roads,grid_size_x,grid_size_y,temp_eff):
    

    """A*アルゴリズムでstartからgoalへの最小コスト経路を探索"""
def astar(start, goal,temp,soil_amount):
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
    


# 評価関数
def evaluate_design(temporary_roads,soil_amount):
    temp_copy1 = copy.deepcopy(temporary_roads)
    temp_copy2 = copy.deepcopy(temporary_roads)
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
    route,cost1 = a_star(allocation,temp_copy1,grid_size_x,grid_size_y,temp_eff,soil_amount)
    print("運搬距離コスト",cost1)
    # print("cost1",cost1)
    soil_volume =  grid_length**2 * depth_unit # 1ブロックの土量(m3)
    # print("soil_volume",soil_volume)
    time_average = cost1 * grid_length / (speed_rough_road*1000) # 1往復のみと仮定した時のn台トラックの所要時間(h) 
    # print("time_average",time_average)
    cost_construction = cost_hour * soil_volume/(truck_num * volume_unit_truck) * time_average / work_eff # 作業コスト($)
    "仮設道路の建設にかかるコスト"
    #定数の値
    construction_temp = 17.5 # 仮設道路の建設単位長さあたりのコスト($/m)
    #コスト計算
    cost2 = temp_length(temp_copy2,soil_amount)
    print("仮設道路の長さ",cost2)
    cost_construction_temp = cost2 * construction_temp * grid_length# 仮設道路の建設コスト($)
    #総コスト
    total_cost = cost_construction + cost_construction_temp

    print("cost_construction:$",cost_construction)
    print("cost_construction_temp:$",cost_construction_temp)
    print("total_cost:$",total_cost)
    return route,total_cost



def generate_neighbor(route,temp,soil_amount):
    #新たな解を生成
    return temp_copy

def simulated_annealing(allocation,temp,soil_amount):
    current_solution = temp
    current_route,current_score = evaluate_design(current_solution)
    best_solution = current_solution
    best_score = current_score
    best_route = current_route
    temperature = initial_temprature
    best_score_loop = 0
    best_solution_flow = []
    current_solution_flow = []
    neighbor_solution_flow = []

    for _ in range(max_iter):
        print("simulated anealing loop",_)
        print("soil_amount",soil_amount)
        temperature *= alpha
        current_route_copy = copy.deepcopy(current_route)
        current_solution_copy = copy.deepcopy(current_solution)
        neighbor_solution= generate_neighbor(current_route_copy,current_solution_copy,soil_amount)
        neighbor_route,neighbor_score = evaluate_design(neighbor_solution,soil_amount)


        # 新しい解が良い場合、または確率的に受け入れる
        if (neighbor_score < current_score) or (random.random() < np.exp(-( np.abs(neighbor_score - current_score) ) / temperature)):
            print("neighbor")
            current_solution = neighbor_solution
            current_score = neighbor_score
            current_route = neighbor_route

            # ベスト解を更新
        if current_score < best_score:
            best_solution = current_solution
            best_score = current_score
            best_route = current_route
            best_score_loop = _
        
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


    return best_solution, best_score, best_route ,best_solution_flow,current_solution_flow,neighbor_solution_flow,best_score_loop



# テスト
temporary_roads = [
    {(1, 2): {6}, (2, 2): {1}},
    {(1, 0): {7}, (2, 1): {0}}
]
# allocation = [[(0, 3), (3, 2)], [(1, 0), (2, 0)], [(0, 0), (3, 1)]]
allocation = [[(1, 0), (3, 1)], [(0, 0), (3, 1)]]
best_schedule, best_cost, best_construction_steps = simulated_annealing(
    allocation, temporary_roads)

# パラメータの設定
initial_temprature = 1000
# final_temprature = 1
alpha = 0.95
max_iter = 2000