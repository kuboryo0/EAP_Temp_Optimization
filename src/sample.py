import numpy as np
import random
import pulp
import time
import math
import copy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq

DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
grid_size_x = 4
grid_size_y = 4
temp_eff = 0.5

cut_indices = [[(1, 0),1],[(1, 2),1],[(2, 0),1],[(2, 1),1],[(2, 2),1],[(2, 3),1],[(3, 0),1],[(3, 1),1],[(3, 3),2]]
#盛土の座標と土量
fill_indices = [[(0, 0),2],[(0, 1),1],[(0, 2),1],[(0, 3),1],[(1, 3),2],[(1, 1),1],[(3, 2),2]]

soil_amount = np.zeros((grid_size_x, grid_size_y))
for [(i, j),k] in cut_indices:
    soil_amount[int(i), int(j)] = k
for [(i, j),k] in fill_indices:
    soil_amount[int(i),int(j)] = -k

# ペアから元の形式に戻す関数
def convert_to_directions(pairs_list, directions):
    temporary_roads = []
    # 辞書で座標ごとの方向を管理
    for pairs in pairs_list:
        # print("pairs",pairs)
        temporary_roads_in_pair = []
        coord_to_directions = {}
        for pair in pairs:
            (x1, y1), (x2, y2) = pair
            # (x1, y1) から (x2, y2) への方向を計算
            dx1, dy1 = x2 - x1, y2 - y1
            direction1 = directions.index((dx1, dy1))
            # (x2, y2) から (x1, y1) への方向を計算
            dx2, dy2 = x1 - x2, y1 - y2
            direction2 = directions.index((dx2, dy2))
            # (x1, y1) に方向を追加
            if (x1, y1) not in coord_to_directions:
                coord_to_directions[(x1, y1)] = set()
            coord_to_directions[(x1, y1)].add(direction1)
            # (x2, y2) に方向を追加
            if (x2, y2) not in coord_to_directions:
                coord_to_directions[(x2, y2)] = set()
            coord_to_directions[(x2, y2)].add(direction2)
        temporary_roads_in_pair.append(coord_to_directions)
        # print("temporary_roads_in_pair",temporary_roads_in_pair)
    # データ構造を temporary_roads の形式に変換
        temporary_roads.append(temporary_roads_in_pair)

    return temporary_roads

def convert_to_pairs(temporary_roads, directions):
    connected_pairs = []
    for road_set in temporary_roads:
        connected_pairs_in_set = []
        for (x, y), connected_directions in road_set.items():
            for direction in connected_directions:
                dx, dy = directions[direction]
                neighbor = (x + dx, y + dy)
                # Add the pair in sorted order to avoid duplicates
                pair = tuple(sorted([(x, y), neighbor]))
                connected_pairs_in_set.append(pair)
        # Remove duplicate pairs by converting to a set and back to a list
        connected_pairs_in_set = list(set(connected_pairs_in_set))
        connected_pairs.append(connected_pairs_in_set)
    
    return connected_pairs


def single_temp_length(single_temp,soil_amount):
    single_length = 0
    # print("single_temp",single_temp)
    if single_temp is not None:
        for cell, directions in single_temp.items():
            for direction in directions:
                single_length += ((DIRECTIONS[direction][0]**2 + DIRECTIONS[direction][1]**2)**0.5) /2
    return single_length
def temp_length(temps,soil_amount):
    """仮設道路の長さを計算"""
    length = 0
    # print("temps",temps)
    for temp_index,single_temp in enumerate(temps):
        length += single_temp_length(single_temp,soil_amount)
    return length
def temp_length_diff(temp1,temp2,soil_amount):
    """仮設道路の長さの差分を計算"""
    # print("temp1",temp1)
    # print("temp2",temp2)
    return np.abs(single_temp_length(temp1,soil_amount) - single_temp_length(temp2,soil_amount))
      
def temp_in_step(current_temps,step,solution):

    # print("current_temp",current_temp)
    # print("temp",temp)
    # print("step",step)
    # print("solution",solution)
    # print("current_temp before tempinstep",current_temps)
    current_temps_copy = copy.deepcopy(current_temps)
    built_road_set = []
    for temp in solution:
        # print("temp",temp)
        built_road_set_in_temp = []
        for coord_list in temp:
            # print("coord_list",coord_list)
            coord_pair = coord_list[0]
            coord_built_step_list = coord_list[1]
            # print("coord_pair",coord_pair)
            # print("coord_built_step_list",coord_built_step_list)
            if coord_built_step_list[step] == 1:
                coord1 = coord_pair[0]
                coord2 = coord_pair[1]
                coord1_direction = DIRECTIONS.index((coord2[0] - coord1[0], coord2[1] - coord1[1])) 
                coord2_direction = DIRECTIONS.index((coord1[0] - coord2[0], coord1[1] - coord2[1]))
                built_road_set_in_temp.append({coord1: {coord1_direction}, coord2: {coord2_direction}})
        #おなじ座標のsetをマージ
        merged_data = {}
        for dictionary in built_road_set_in_temp:
            for key, value in dictionary.items():
                if key not in merged_data:
                    merged_data[key] = set()
                merged_data[key].update(value)
        # print("merged_data",merged_data)
        built_road_set.append(merged_data)
    # print("built_road_set",built_road_set)
    built_length = 0
    for i in range(len(current_temps)):
        # print("built_length before i",built_length)
        current_temp = current_temps[i]
        # print("current_temp",current_temp)
        if current_temp is None:
            current_temps_copy[i] = built_road_set[i]
            # print("current_temp is None")
            # print("add length ",single_temp_length(built_road_set[i],soil_amount))
            built_length += single_temp_length(built_road_set[i],soil_amount)
        else:
            for coord in built_road_set[i]:
                # print("coord",coord)
                coord_directions = built_road_set[i][coord]
                # print("coord_directions",coord_directions)
                for coord_direction in coord_directions:
                    if coord not in current_temp:
                        # print("coord_direction",coord_direction)
                        # print("coord not in current_temp",coord)
                        # print("add length ",((DIRECTIONS[coord_direction][0]**2 + DIRECTIONS[coord_direction][1]**2)**0.5) /2)
                        built_length += ((DIRECTIONS[coord_direction][0]**2 + DIRECTIONS[coord_direction][1]**2)**0.5) /2
                        current_temps_copy[i][coord] = {coord_direction}
                        # current_temps_copy[i][coord].add(DIRECTIONS.index[coord_direction])
                    elif coord_direction not in current_temp[coord]:
                        # print("coord_direction not in current_temp",coord)
                        # print("add length ",((DIRECTIONS[coord_direction][0]**2 + DIRECTIONS[coord_direction][1]**2)**0.5) /2)
                        built_length += ((DIRECTIONS[coord_direction][0]**2 + DIRECTIONS[coord_direction][1]**2)**0.5) /2
                        # print("coord_direction",coord_direction)
                        current_temps_copy[i][coord].add(coord_direction)
    # print("current_temp after",current_temps_copy)
    # print("\n")
    # print("current_temp after tempinstep",current_temps_copy)
    return current_temps_copy,built_length

def generate_neighbor(current_solution,change_prob):
    current_solution_copy = copy.deepcopy(current_solution)
    all_pairs = [
        (i, j, k) 
        for i, temp in enumerate(current_solution)
        for j, pair in enumerate(temp)
        for k in range(len(pair[1]))
    ]
    total_pairs = len(all_pairs)
    
    # Randomly choose a pair and a timing index
    random_index = random.randint(0, total_pairs - 1)
    temp_index, pair_index, timing_index = all_pairs[random_index]
    print("temp_index",temp_index,"pair_index",pair_index,"timing_index",timing_index)
    # Flip the selected timing
    current_solution_copy[temp_index][pair_index][1][timing_index] ^= 1

    # Randomly flip additional timings with probability change_prob
    for i in range(len(current_solution_copy[temp_index][pair_index][1])):
        if random.random() < change_prob:
            current_solution_copy[temp_index][pair_index][1][i] ^= 1

    return current_solution_copy

def a_star_alogrithm(start_goal_pairs,temp,soil_amount,solution):
    #timingに基づくタイミングで道路を建設した場合のコストを計算する
    temporary_roads = copy.deepcopy(temp)
    def single_temp_length(single_temp,soil_amount):
        single_length = 0
        # print("single_temp",single_temp)
        if single_temp is not None:
            for cell, directions in single_temp.items():
                for direction in directions:
                    single_length += ((DIRECTIONS[direction][0]**2 + DIRECTIONS[direction][1]**2)**0.5) /2
        return single_length
    def temp_length(temps,soil_amount):
        """仮設道路の長さを計算"""
        length = 0
        # print("temps",temps)
        for temp_index,single_temp in enumerate(temps):
            length += single_temp_length(single_temp,soil_amount)
        return length
    def temp_length_diff(temp1,temp2,soil_amount):
        """仮設道路の長さの差分を計算"""
        # print("temp1",temp1)
        # print("temp2",temp2)
        return np.abs(single_temp_length(temp1,soil_amount) - single_temp_length(temp2,soil_amount))
    def temp_in_step(current_temps,step,solution):
        # print("current_temp",current_temp)
        # print("temp",temp)
        # print("step",step)
        # print("solution",solution)
        # print("current_temp before tempinstep",current_temps)
        current_temps_copy = copy.deepcopy(current_temps)
        built_road_set = []
        for temp in solution:
            # print("temp",temp)
            built_road_set_in_temp = []
            for coord_list in temp:
                # print("coord_list",coord_list)
                coord_pair = coord_list[0]
                coord_built_step_list = coord_list[1]
                # print("coord_pair",coord_pair)
                # print("coord_built_step_list",coord_built_step_list)
                if coord_built_step_list[step] == 1:
                    coord1 = coord_pair[0]
                    coord2 = coord_pair[1]
                    coord1_direction = DIRECTIONS.index((coord2[0] - coord1[0], coord2[1] - coord1[1])) 
                    coord2_direction = DIRECTIONS.index((coord1[0] - coord2[0], coord1[1] - coord2[1]))
                    built_road_set_in_temp.append({coord1: {coord1_direction}, coord2: {coord2_direction}})
            #おなじ座標のsetをマージ
            merged_data = {}
            for dictionary in built_road_set_in_temp:
                for key, value in dictionary.items():
                    if key not in merged_data:
                        merged_data[key] = set()
                    merged_data[key].update(value)
            # print("merged_data",merged_data)
            built_road_set.append(merged_data)
        # print("built_road_set",built_road_set)
        built_length = 0
        for i in range(len(current_temps)):
            # print("built_length before i",built_length)
            current_temp = current_temps[i]
            # print("current_temp",current_temp)
            if current_temp is None:
                current_temps_copy[i] = built_road_set[i]
                # print("current_temp is None")
                # print("add length ",single_temp_length(built_road_set[i],soil_amount))
                built_length += single_temp_length(built_road_set[i],soil_amount)
            else:
                for coord in built_road_set[i]:
                    # print("coord",coord)
                    coord_directions = built_road_set[i][coord]
                    # print("coord_directions",coord_directions)
                    for coord_direction in coord_directions:
                        if coord not in current_temp:
                            # print("coord_direction",coord_direction)
                            # print("coord not in current_temp",coord)
                            # print("add length ",((DIRECTIONS[coord_direction][0]**2 + DIRECTIONS[coord_direction][1]**2)**0.5) /2)
                            built_length += ((DIRECTIONS[coord_direction][0]**2 + DIRECTIONS[coord_direction][1]**2)**0.5) /2
                            current_temps_copy[i][coord] = {coord_direction}
                            # current_temps_copy[i][coord].add(DIRECTIONS.index[coord_direction])
                        elif coord_direction not in current_temp[coord]:
                            # print("coord_direction not in current_temp",coord)
                            # print("add length ",((DIRECTIONS[coord_direction][0]**2 + DIRECTIONS[coord_direction][1]**2)**0.5) /2)
                            built_length += ((DIRECTIONS[coord_direction][0]**2 + DIRECTIONS[coord_direction][1]**2)**0.5) /2
                            # print("coord_direction",coord_direction)
                            current_temps_copy[i][coord].add(coord_direction)
        # print("current_temp after",current_temps_copy)
        # print("\n")
        # print("current_temp after tempinstep",current_temps_copy)
        return current_temps_copy,built_length


    def get_distance(a, b,soil):
        """current と neighbor の3次元距離を計算"""
        # print("a",a)
        # print("b",b)
        # horizontal_distance = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        # vertical_distance = abs(soil[int(a[0])][int(a[1])] - soil[int(b[0])][int(b[1])])
        # return (horizontal_distance**2 + vertical_distance**2)**0.5
        return  ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

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
        for temp_index,temp in enumerate(temporary_roads):
            if direction_index_current in temp.get(current, set()):
                current_distance *=  temp_eff
            if direction_index_neighbor in temp.get(neighbor, set()):
                neighbor_distance *=  temp_eff 
        
        return current_distance + neighbor_distance
    
    def hasUsedRoad(path,temp):
        """経路上で仮設道路を利用したかどうか"""
        # print("path",path)
        # print("temporary_roads",temp)
        for i in range(len(path) - 1):
            current = path[i]
            next = path[i + 1]
            direction = (next[0] - current[0], next[1] - current[1])
            direction_index = DIRECTIONS.index(direction)
            if direction_index in temp.get(current, set()):
                return True
        return False

    def remove_temporary_roads(start, goal,temps):
        """指定されたセル（start, goal）から仮設道路を削除"""
        for cell in [start, goal]:
            for temp in temps:
                if temp is not None:
                    if cell in temp:
                        del temp[cell]
    
    def change_soil(start,goal,soil_amount):
        """指定されたセル（start, goal）から土量を変更"""
        soil_amount[int(start[0])][int(start[1])] -= 1
        soil_amount[int(goal[0])][int(goal[1])] += 1

    def astar(start, goal,temp,soil_amount,used_temp_list):
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
            # print("current",current)
            # print("came_from",came_from)
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()

        #経路上で仮設道路を利用したかどうか
        used_temp_list_inStep = []
        for temp_index,temp in enumerate(temp_copy):
            # print("temp_index",temp_index)
            # print("temp",temp)
            # print("path",path)
            # print("hasUsedRoad",hasUsedRoad(path,temp))
            if temp is not None:
                # print("hasUsedRoad",hasUsedRoad(path,temp))
                if hasUsedRoad(path,temp):
                    used_temp_list_inStep.append(temp_index)
            
        used_temp_list.append(used_temp_list_inStep)
        # 仮設道路を削除
        remove_temporary_roads(start, goal,temp_copy)
        for i,temp in enumerate(temp_copy):
            # print("temp",temp)
            if temp is not None:
                if len(temp) == 0:
                   temp_copy[i] = None
        change_soil(start,goal,soil_amount_copy)
        # print(f"Temporary Roads After Removal: {temp_copy}")

        return path, cost_so_far[goal],temp_copy,soil_amount_copy
    
    total_cost = 0
    path_list = []
    # print("temporary_roads in astar",temporary_roads)
    current_temporary = list(None for _ in range(len(temporary_roads)))
    # print("current_temporary",current_temporary)
    temp_deepcopy = copy.deepcopy(temporary_roads)
    soil_amount_deepcopy = copy.deepcopy(soil_amount)
    total_built_length = 0
    used_temp_list = []
    for i,(start, goal) in enumerate(start_goal_pairs):
        # print("i",i)
        # print("soil_amount before",soil_amount_deepcopy)
        # print("timing",timing)
        current_temporary,built_length = temp_in_step(current_temporary,i,solution)
        # print("current_temporary before",current_temporary)
        # print("built_length",built_length)
        total_built_length += built_length
        path, cost,current_temporary,soil_amount_deepcopy = astar(start, goal,current_temporary,soil_amount_deepcopy,used_temp_list)
        total_cost += cost
        path_list.append(path)
        # print("current_temporary after",current_temporary)
        # print("current_temporary after",current_temporary)
        # print(f"Start: {start}, Goal: {goal}")

        # print(f"Path: {path}")
        # print(f"Cost: {cost}\n")
        # print("\n")
    # print(f"Total Cost: {total_cost}")

    # print("used_temp_list",used_temp_list)
    return path_list,total_cost,used_temp_list,total_built_length




def evaluate_design(solution):
    temp_copy1 = copy.deepcopy(temporary_roads)
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
    route,cost1,used_road_flow,road_length = a_star_alogrithm(allocation,temp_copy1,soil_amount,solution)
    # print("運搬距離コスト",cost1)
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
    cost2 = road_length
    # print("仮設道路の長さ",cost2)
    cost_construction_temp = cost2 * construction_temp * grid_length# 仮設道路の建設コスト($)
    #総コスト
    total_cost = cost_construction + cost_construction_temp

    print("cost_construction:$",cost_construction)
    print("cost_construction_temp:$",cost_construction_temp)
    print("total_cost:$",total_cost)
    return route,total_cost,used_road_flow



temporary_roads = [
    {(1, 2): {6, 4, 7}, (2, 2): {1}, (1, 3): {3}, (2, 3): {0}},
    {(1, 0): {7}, (2, 1): {0}},
]
allocation = [[(1, 2), (2, 3)], [(1, 0), (2, 1)],[(0, 0), (3, 2)]]
total_step = len(allocation)
temporary_roads_pairs =[[[(1, 2), (2, 3)], [(1, 2), (1, 3)], [(1, 2), (2, 2)]], [[(1, 0), (2, 1)]]]
# temporary_roads = convert_to_directions(temporary_roads_pairs, DIRECTIONS)
solution = [
            ]
# for temporary_roads_pair in temporary_roads_pairs:
#     solution_in_pair = []
#     for pair in temporary_roads_pair:
#         solution_in_pair.append([pair,[1]*total_step])
#     solution.append(solution_in_pair)

print("solution",solution)

# current_temp = [None] * len(temporary_roads)
# current_temp[1] = {(1, 0): {7}}
# print("current_temp",current_temp)
# total_built_length = 0
# for i in range(2):
#     current_temp,built_length_i = temp_in_step(current_temp,i,solution)
#     total_built_length += built_length_i
#     print("current_temp",current_temp)
#     print("built_length",total_built_length)
chage_prob = 0.6
for i in range(10):
    print("step",i)
    solution = generate_neighbor(solution,chage_prob)
    chage_prob *= 0.9
    route,total_cost,used_road_flow = evaluate_design(solution)
    print("solution",solution,)
    print("route",route)
    print("total_cost",total_cost)
    print("used_road_flow",used_road_flow,"\n")