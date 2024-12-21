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
# from function_ver2 import plot_route



DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
grid_size_x = 4
grid_size_y = 4
temp_eff = 0.5

def temp_in_step(current_temps,step,solution,built_length_list):
        def pair_length(pair,soil_amount):
            return ((pair[0][0]-pair[1][0])**2 + (pair[0][1]-pair[1][1])**2)**0.5
        current_temps_copy = copy.deepcopy(current_temps)
        built_length = 0
        print("solution in tempinstep",solution)
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
        print("current_temp after tempinstep",current_temps_copy)
        return current_temps_copy,built_length


def a_star_alogrithm(start_goal_pairs,soil_amount,solution):
    #timingに基づくタイミングで道路を建設した場合のコストを計算する
    # temp_pair = [solution[i][0] for i in range(len(solution))]
    # print("temp_pair",temp_pair)
    # temporary_roads = convert_to_directions(temp_pair)
    # print("temporary_roads",temporary_roads)

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

    def get_cost(current, neighbor,soil_amount,temporary_roads):
        """移動コストを計算"""
        distance = get_distance(current, neighbor,soil_amount)
        # direction = (neighbor[0] - current[0], neighbor[1] - current[1])
        # direction_index_current = DIRECTIONS.index(direction)
        # direction_index_neighbor = DIRECTIONS.index((-direction[0], -direction[1]))
        current_distance = distance/2
        neighbor_distance = distance/2
        # print("temporary_roads",temporary_roads)
        # print("current",current)
        # print("neighbor",neighbor)
        for temp in temporary_roads:
            # print("temp[0][0]",temp[0][0])
            # print("temp[1][0]",temp[1][0])
            # print("current",current)
            # print("temp[1][0]==current",temp[1][0]==current)
            # print("temp[0][0]==current",temp[0][0]==current)
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
        # print(f"current:{current}, neighbor:{neighbor}, distance:{current_distance + neighbor_distance}")
        return current_distance + neighbor_distance
    
    def hasUsedRoad(path,temp):
        """経路上で仮設道路を利用したかどうか"""
        # print("path",path)
        # print("temporary_roads",temp)
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
                    # print("coord",coord)
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
            # print("current",current)
            # print("came_from",came_from)
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()

        #経路上で仮設道路をどれくらい利用したかを記録
        used_temp_list_inStep = [0]*len(temp_copy)
        print("temp_copy",temp_copy)
        for temp_index,temp in enumerate(temp_copy):
            # print("temp_index",temp_index)
            # print("temp",temp)
            # print("path",path)
            # print("hasUsedRoad",hasUsedRoad(path,temp))
            if temp[0][1] == 1 or temp[1][1] == 1:
                # print("hasUsedRoad",hasUsedRoad(path,temp))
                if hasUsedRoad(path,temp):
                    print("temp_index",temp_index)
                    # print("temp[0][0][0]",temp[0][0][0])
                    # print("temp[1][0][0]",temp[1][0][0])
                    # print("temp[0][0][1]",temp[0][0][1])
                    # print("temp[1][0][1]",temp[1][0][1])
                    used_distance = ((temp[0][0][0] - temp[1][0][0])**2 + (temp[0][0][1] - temp[1][0][1])**2)**0.5
                    print("temp[0][1]",temp[0][1],"temp[1][1]",temp[1][1])
                    used_distance *= 0.5 if not (temp[0][1] == 1 and temp[1][1] == 1) else 1
                    print("\n","used_distance",used_distance,"\n")
                    used_temp_list_inStep[temp_index] = used_distance
            
        used_temp_list.append(used_temp_list_inStep)
        # 仮設道路を削除
        remove_temporary_roads(start, goal,temp_copy)
        for i,temp in enumerate(temp_copy):
            # print("temp",temp)
            if temp is not None:
                if len(temp) == 0:
                   temp_copy[i] = None
        change_soil(start,goal,soil_amount_copy)
        print(f"Temporary Roads After Removal: {temp_copy}")

        return path, cost_so_far[goal],temp_copy,soil_amount_copy
    
    total_cost = 0
    path_list = []
    # print("temporary_roads in astar",temporary_roads)
    print("solution",solution)
    current_temporary = [[[solution[i][0][0],0],[solution[i][0][1],0]] for i in range(len(solution))]
    print("current_temporary",current_temporary)
    # temp_deepcopy = copy.deepcopy(temporary_roads)
    soil_amount_deepcopy = copy.deepcopy(soil_amount)
    total_built_length = 0
    used_temp_list = []
    built_length_list = [0]*len(solution)
    for i,(start, goal) in enumerate(start_goal_pairs):
        print("i",i)
        # print("soil_amount before",soil_amount_deepcopy)
        # print("timing",timing)
        print("current_temporary before",current_temporary)
        current_temporary,built_length = temp_in_step(current_temporary,i,solution,built_length_list)
        print(f"current_temporary after temp_in_step {i}",current_temporary)
        print("built_length",built_length)
        total_built_length += built_length
        path, cost,current_temporary,soil_amount_deepcopy = astar(start, goal,current_temporary,soil_amount_deepcopy,used_temp_list)
        total_cost += cost
        path_list.append(path)
        print("current_temporary after",current_temporary)
        print(f"Start: {start}, Goal: {goal}")
        print(f"Path: {path}")
        print(f"Cost: {cost}\n")
        # print("\n")
    # print(f"Total Cost: {total_cost}")
    print("built_length_list",built_length_list)
    # print("used_temp_list",used_temp_list)
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
    cost2 = road_length
    print("仮設道路の長さ",cost2)
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
    print("temp_lists",temp_lists)
    print("current_solution before",current_solution,"\n")
    print("temp_usage_flow",temp_usage_flow)
    print("allocation",allocation)
    current_solution_copy = copy.deepcopy(current_solution)
    # 仮設道路の使用量を計算
    usage_weights = list([current_solution[i],0] for i in range(len(current_solution)))
    print("built_length_list",built_length_list)
    print("usage_weights before",usage_weights)
    for i in range(len(usage_weights)):
        for j in range(len(temp_usage_flow)):
            usage_weights[i][1] += temp_usage_flow[j][i]
        # print("usage_weights[i][1]",usage_weights[i][1])
        if built_length_list[i] != 0:
            usage_weights[i][1] /= built_length_list[i]
        else:
            usage_weights[i][1] = 0
    print("usage_weights after",usage_weights)
    
    # 使用量が低い道路のリストを作成
    low_usage_roads = sorted(usage_weights, key=lambda x: x[1])  # 使用量が低い順にソート
    print("low_usage_roads",low_usage_roads)
    # # 使用量が0の道路を削除
    # low_usage_roads = [road for road in low_usage_roads if road[1] > 0]
    # print("low_usage_roads after filtering:", low_usage_roads)
    # # tempから使用量が0だった道路を削除
    # remaining_roads = [road for road, _ in low_usage_roads]  # low_usage_roadsに残った道路を抽出
    # temp_copy = [road for road in temp_copy if road in remaining_roads]
    # print("temp after filtering:", temp_copy)
    random_num = random.random()
    print("random_num",random_num)
    if len(low_usage_roads) == 0 or random_num < 0.15 :  # 仮設道路なしor15%の確率で道路を追加
        print("道路を追加")
        while True:
            coord1 = (random.randint(0, grid_size_x), random.randint(0, grid_size_y))
            dx, dy = random.choice(DIRECTIONS)
            coord2 = (coord1[0] + dx, coord1[1] + dy)
            isSame = any((coord1 in temp_list and coord2 in temp_list) for temp_list in temp_lists)
            if not isSame and (0 <= coord2[0] < grid_size_x and 0 <=coord2[1] < grid_size_y):
                break
        print("coord1",coord1)
        print("coord2",coord2)
        new_timing = []
        for i in range(total_step_num):
            new_timing.append(random.randint(0,1)) 
        print("new_timing",new_timing)
        current_solution_copy.append([[coord1,coord2],new_timing])
    else:
        total_weight = sum(1 / (usage + 1e-6) for _, usage in low_usage_roads)  # 小さい値を追加してゼロ除算を防止
        probabilities = [(1 / (usage + 1e-6)) / total_weight for _, usage in low_usage_roads]
        print("total_weight",total_weight)
        print("probabilities",probabilities)
        # 確率に基づいて選択
        road_index = np.random.choice(len(low_usage_roads), p=probabilities)
        print("road_index",road_index)
        if len(low_usage_roads) == 1 or random_num < 0.60:
            if random_num < 0.3:
                print("タイミングを変更")
                print("current_solution_copy[road_index][1]",current_solution_copy[road_index][1])
                timing_choice = random.randint(0,total_step_num-1)
                current_solution_copy[road_index][1][timing_choice] ^= 1 
                for i in range(len(current_solution_copy[road_index][1])):
                    if random.random() < 0.2:
                        current_solution_copy[road_index][1][i] ^= 1
                print("current_solution_copy[road_index][1] after",current_solution_copy[road_index][1])
            elif random_num < 0.45:
                print("道路を削除")
                current_solution_copy.remove(low_usage_roads[road_index][0])
                print("current_solution_copy after remove",current_solution_copy)
            else:
                print("道路を変更")
                while True:
                    coord1 = (random.randint(0, grid_size_x), random.randint(0, grid_size_y))
                    dx, dy = random.choice(DIRECTIONS)
                    coord2 = (coord1[0] + dx, coord1[1] + dy)
                    isSame = any((coord1 in temp_list and coord2 in temp_list) for temp_list in temp_lists)
                    if not isSame and (0 <= coord2[0] < grid_size_x and 0 <=coord2[1] < grid_size_y):
                        break
                print("coord1",coord1)
                print("coord2",coord2)
                current_solution_copy[road_index][0] = [coord1,coord2]
                print("current_solution_copy after change",current_solution_copy)
        else:
            print("2つ目の道路を選択")
            while True:
                road_index2 = np.random.choice(len(low_usage_roads))
                if road_index2 != road_index:
                    break
            print("road_index2",road_index2)
            if random_num < 0.75:
                print("点を交換")
                random_index1 = random.randint(0,1)
                random_index2 = random.randint(0,1)
                for i in range(2):
                    for j in range(2):
                        if current_solution_copy[road_index][0][i] == current_solution_copy[road_index2][0][j]:
                            random_index1,random_index2 = 1-i,1-j
                            break
                current_solution_copy[road_index][0][random_index1],current_solution_copy[road_index2][0][random_index2] = current_solution_copy[road_index2][0][random_index2],current_solution_copy[road_index][0][random_index1]
                print("current_solution_copy after change",current_solution_copy)
            elif random_num < 0.9:
                print("タイミングを交換")
                current_solution_copy[road_index][1],current_solution_copy[road_index2][1] = current_solution_copy[road_index2][1],current_solution_copy[road_index][1]
                print("current_solution_copy after change",current_solution_copy)
            else:
                print("タイミングを同化")
                current_solution_copy[road_index][1] = current_solution_copy[road_index2][1]
                print("current_solution_copy after change",current_solution_copy)
    
    return current_solution_copy


def simulated_annealing(allocation,soil_amount,timing):
    current_solution = timing
    current_route,current_score,current_used_flow,current_built_length_list = evaluate_design(current_solution)
    best_solution = current_solution
    best_score = current_score
    best_route = current_route
    best_used_flow = current_used_flow
    temperature = initial_temprature
    best_score_loop = 0
    best_solution_flow = []
    current_solution_flow = []
    neighbor_solution_flow = []
    print("\n")

    for _ in range(max_iter):
        print("simulated anealing loop",_)
        print("soil_amount",soil_amount)
        temperature *= alpha
        current_route_copy = copy.deepcopy(current_route)
        current_solution_copy = copy.deepcopy(current_solution)
        neighbor_solution= generate_neighbor(current_solution_copy,current_used_flow,allocation,current_built_length_list)
        print("neighbor_solution",neighbor_solution)
        
        neighbor_route,neighbor_score,neighbor_used_flow,neighbor_built_length_list = evaluate_design(neighbor_solution)
        # print("neighbor_route",neighbor_route)
        # print("neighbor_score",neighbor_score)
        print("neighbor_used_flow",neighbor_used_flow)

        # 新しい解が良い場合、または確率的に受け入れる
        if (neighbor_score < current_score) or (random.random() < np.exp(-( np.abs(neighbor_score - current_score) ) / temperature)):
            print("neighbor")
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

    # for i in range(len(temporary_roads)):
    #     best_solution = moderate_solution(best_solution,best_used_flow,i)
    #     best_route,best_score,best_used_flow = evaluate_design(best_solution)

        # return best_solution, best_score, best_route ,best_solution_flow,current_solution_flow,neighbor_solution_flow,best_score_loop
    return best_solution, best_score, best_route,best_solution_flow,current_solution_flow,neighbor_solution_flow,best_score_loop


cut_indices = [[(1, 0),1],[(1, 2),1],[(2, 0),1],[(2, 1),1],[(2, 2),1],[(2, 3),1],[(3, 0),1],[(3, 1),1],[(3, 3),2]]
#盛土の座標と土量
fill_indices = [[(0, 0),2],[(0, 1),1],[(0, 2),1],[(0, 3),1],[(1, 3),2],[(1, 1),1],[(3, 2),2]]

soil_amount = np.zeros((grid_size_x, grid_size_y))
for [(i, j),k] in cut_indices:
    soil_amount[int(i), int(j)] = k
for [(i, j),k] in fill_indices:
    soil_amount[int(i),int(j)] = -k
# print("soil_amount",soil_amount)


current_temps = [
                [[(1, 2),0], [(2, 3),1]], [[(1, 2),0], [(1, 3),0]],[[(1, 2),0], [(2, 2),0]], 
                [[(1, 0),0], [(2, 1),0]],
                ]

solution = [

            ]

total_cost = 0
# for i in range(2):
#     current_temps,cost = temp_in_step(current_temps,i,solution)
#     print("current_temps",current_temps)
#     print("cost",cost)
#     total_cost += cost
# print("total_cost",total_cost)

allocation = [((1, 2), (3, 3)),((0, 2),(3, 3)),((0, 0),(3, 2))]
# print(a_star_alogrithm(allocation,soil_amount,solution))
temp_usage_flow = [[1.4142135623730951, 0], [1.4142135623730951, 0],[0, 1]]
built_length_list = [2.121320343559643, 1.0]


# for i in range(10):
#     generate_neighbor(solution,temp_usage_flow,allocation,built_length_list)

# パラメータの設定
initial_temprature = 1000
# final_temprature = 1
alpha = 0.95
max_iter = 10000

# for i in range(15):
#     generate_neighbor(solution,temp_usage_flow,allocation,built_length_list)
# simulated_annealing(allocation,soil_amount,solution)
start_time = time.time()
best_solution, best_score, best_route,best_solution_flow,current_solution_flow,neighbor_solution_flow,best_score_loop = simulated_annealing(allocation,soil_amount,solution)
end_time = time.time()
print("best_solution",best_solution)
print("best_score",best_score)
print("best_route",best_route)
# for i in range(len(best_solution)):
#     print("best_score flow",best_solution[i][0])

print("best_score_loop",best_score_loop)
print("time",end_time-start_time)