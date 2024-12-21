import copy
import numpy as np
import heapq
import random
grid_size_x = 4
grid_size_y = 4
built_road_set = {(1, 2): {4, 6, 7}, (2, 3): {0}, (1, 3): {3}, (2, 2): {1}, (1, 0): {7}, (2, 1): {0}}
DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
temp_eff = 0.5
current_temps = [[(1, 2), (2, 3)], [(1, 2), (1, 3)]]

current_temps = [
                [[(1, 2),0], [(2, 3),1]], [[(1, 2),0], [(1, 3),0]],[[(1, 2),0], [(2, 2),0]], 
                [[(1, 0),0], [(2, 1),0]],
                ]
cut_indices = [[(1, 0),1],[(1, 2),1],[(2, 0),1],[(2, 1),1],[(2, 2),1],[(2, 3),1],[(3, 0),1],[(3, 1),1],[(3, 3),2]]
#盛土の座標と土量
fill_indices = [[(0, 0),2],[(0, 1),1],[(0, 2),1],[(0, 3),1],[(1, 3),2],[(1, 1),1],[(3, 2),2]]

soil_amount = np.zeros((grid_size_x, grid_size_y))
for [(i, j),k] in cut_indices:
    soil_amount[int(i), int(j)] = k
for [(i, j),k] in fill_indices:
    soil_amount[int(i),int(j)] = -k
def temp_in_step(current_temps,step,solution):
        def pair_length(pair,soil_amount):
            return ((pair[0][0]-pair[1][0])**2 + (pair[0][1]-pair[1][1])**2)**0.5
        current_temps_copy = copy.deepcopy(current_temps)
        built_length = 0

        for i,(pair,timing) in enumerate(solution):
            if timing[step] == 1:
                if current_temps[i][0][1] == 0:
                     built_length += pair_length(pair,soil_amount)/2
                     current_temps_copy[i][0][1] = 1
                if current_temps[i][1][1] == 0:
                     built_length += pair_length(pair,soil_amount)/2
                     current_temps_copy[i][1][1] = 1
        print("current_temp after tempinstep",current_temps_copy)
        return current_temps_copy,built_length


# ペアから元の形式に戻す関数
def convert_to_directions(pairs_list):
    temporary_roads = []
    coord_to_directions = {}
    # 辞書で座標ごとの方向を管理
    for pair in pairs_list:
        (x1, y1), (x2, y2) = pair
        # (x1, y1) から (x2, y2) への方向を計算
        dx1, dy1 = x2 - x1, y2 - y1
        direction1 = DIRECTIONS.index((dx1, dy1))
        # (x2, y2) から (x1, y1) への方向を計算
        dx2, dy2 = x1 - x2, y1 - y2
        direction2 = DIRECTIONS.index((dx2, dy2))
        # (x1, y1) に方向を追加
        if (x1, y1) not in coord_to_directions:
            coord_to_directions[(x1, y1)] = set()
        coord_to_directions[(x1, y1)].add(direction1)
        # (x2, y2) に方向を追加
        if (x2, y2) not in coord_to_directions:
            coord_to_directions[(x2, y2)] = set()
        coord_to_directions[(x2, y2)].add(direction2)
    temporary_roads.append(coord_to_directions)
    # print("temporary_roads_in_pair",temporary_roads_in_pair)
    # データ構造を temporary_roads の形式に変換
    return temporary_roads

def a_star_alogrithm(start_goal_pairs,soil_amount,solution):
    #timingに基づくタイミングで道路を建設した場合のコストを計算する
    temp_pair = [solution[i][0] for i in range(len(solution))]
    # print("temp_pair",temp_pair)
    temporary_roads = convert_to_directions(temp_pair)
    print("temporary_roads",temporary_roads)
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
        print("temporary_roads",temporary_roads)
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
                    # print("temp_index",temp_index)
                    # print("temp[0][0][0]",temp[0][0][0])
                    # print("temp[1][0][0]",temp[1][0][0])
                    # print("temp[0][0][1]",temp[0][0][1])
                    # print("temp[1][0][1]",temp[1][0][1])
                    used_distance = ((temp[0][0][0] - temp[1][0][0])**2 + (temp[0][0][1] - temp[1][0][1])**2)**0.5
                    # print("\n","used_distance",used_distance,"\n")
                    used_distance *= 0.5 if not (temp[0][1] == 1 and temp[1][1] == 1) else 1
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
    current_temporary = [[[solution[i][0][0],0],[solution[i][0][1],0]] for i in range(len(solution))]
    print("current_temporary",current_temporary)
    temp_deepcopy = copy.deepcopy(temporary_roads)
    soil_amount_deepcopy = copy.deepcopy(soil_amount)
    total_built_length = 0
    used_temp_list = []
    for i,(start, goal) in enumerate(start_goal_pairs):
        print("i",i)
        # print("soil_amount before",soil_amount_deepcopy)
        # print("timing",timing)
        print("current_temporary before",current_temporary)
        current_temporary,built_length = temp_in_step(current_temporary,i,solution)
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

    # print("used_temp_list",used_temp_list)
    return path_list,total_cost,used_temp_list,total_built_length



def generate_neighbor(current_solution,temp_usage_flow,allocation):
    #新たな解を生成
    print("current_solution",current_solution)
    print("temp_usage_flow",temp_usage_flow)
    print("allocation",allocation)
    current_solution_copy = copy.deepcopy(current_solution)
    # 仮設道路の使用量を計算
    usage_weights = list([temp_copy[i],0] for i in range(len(current_solution)))
    print("usage_weights before",usage_weights)
    for i in range(len(current_solution)):
        for j in range(len(temp_usage_flow[i])):
            usage_weights[i][1] += temp_usage_flow[i][j]
        # print("temp[i]",temp[i])
    print("usage_weights after",usage_weights)
    # 使用量が低い道路のリストを作成
    low_usage_roads = sorted(usage_weights, key=lambda x: x[1])  # 使用量が低い順にソート
    # print("low_usage_roads",low_usage_roads)
    # 使用量が0の道路を削除
    low_usage_roads = [road for road in low_usage_roads if road[1] > 0]
    # print("low_usage_roads after filtering:", low_usage_roads)
    # tempから使用量が0だった道路を削除
    remaining_roads = [road for road, _ in low_usage_roads]  # low_usage_roadsに残った道路を抽出
    temp_copy = [road for road in temp_copy if road in remaining_roads]
    print("temp after filtering:", temp_copy)
    
    print("current_solution_copy after random change",current_solution_copy)

    # current_solution_copy = moderate_solution(current_solution_copy,temp_usage_flow,temp_choice_index)
    # print("current_solution_copy after ",current_solution_copy)

    return current_solution_copy

solution = [
            [[(1, 2), (2, 3)], [1, 0]] ,
            [[(1, 2), (1, 3)], [1, 0]],
            ]

total_cost = 0
for i in range(2):
    current_temps,cost = temp_in_step(current_temps,i,solution)
    print("current_temps",current_temps)
    print("cost",cost)
    total_cost += cost
print("total_cost",total_cost)

allocation = [((1, 0), (1, 2)),((0, 2),(3, 3))]
print(a_star_alogrithm(allocation,soil_amount,solution))
temp_usage_flow = [[], [0]]