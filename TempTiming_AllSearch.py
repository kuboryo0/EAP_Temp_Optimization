import heapq
import copy
from itertools import product
DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
temp_eff = 0.5  # 仮設道路を通るときのコスト減少効果
grid_size_x = 4
grid_size_y = 4
#仮設道路の建設タイミングを総当たり法で最適化するプログラム


def astar(start, goal, temporary_roads,soil_amount):
    def get_distance(a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def heuristic(a, b):
        return temp_eff * get_distance(a, b)

    def get_cost(current, neighbor, temporary_roads):
        # print("temporary_roads",temporary_roads)
        distance = get_distance(current, neighbor)
        direction = (neighbor[0] - current[0], neighbor[1] - current[1])
        direction_index_current = DIRECTIONS.index(direction)
        direction_index_neighbor = DIRECTIONS.index((-direction[0], -direction[1]))
        current_distance = distance / 2
        neighbor_distance = distance / 2
        if not len(temporary_roads) == 0:
            for temps in temporary_roads:
                # print("temps",temps)
                temp = temps[0]
                # print("temp",temp)
                if direction_index_current in temp.get(current, set()):
                    current_distance *= temp_eff
                if direction_index_neighbor in temp.get(neighbor, set()):
                    neighbor_distance *= temp_eff
        return current_distance + neighbor_distance
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

            if not (0 <= neighbor[0] < grid_size_x and 0 <= neighbor[1] < grid_size_y):
                continue

            new_cost = cost_so_far[current] + get_cost(current, neighbor, temporary_roads)

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(goal, neighbor)
                heapq.heappush(open_set, (priority, neighbor))
                came_from[neighbor] = current

    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path, cost_so_far[goal]


def optimize_construction(allocation, temp):
    def path_on_road(path_list, road):  #経路が仮設道路を通っているかを判定
        if len(road) == 0:
            return True
        is_on_road = False
        for i in range(len(path_list)-1):
            current = path_list[i]
            next = path_list[i+1]
            direction = (next[0] - current[0], next[1] - current[1])
            direction_index_current = DIRECTIONS.index(direction)
            direction_index_next = DIRECTIONS.index((-direction[0], -direction[1]))
            if direction_index_current in road.get(current, set()):
                is_on_road = True
                break
            if direction_index_next in road.get(next, set()):
                is_on_road = True
                break
        return is_on_road
    n = len(temp)  # 仮設道路の本数
    print("temp",temp)
    best_cost = float('inf')
    best_schedule = None
    best_construction_steps = []  # 各ステップで建設された仮設道路を記録
    temp_copy = copy.deepcopy(temp)
    temp_index_list = list(range(n)) #　仮設道路が建設されているかを判定するためのリスト
    print("temp_index_list",temp_index_list)
    print("temp_copy",temp_copy)
    # 仮設道路のすべての建設パターンを生成
    all_patterns = list(product([0, 1], repeat=n)) 
    print("all_patterns",all_patterns)
    construction_pattern_index = [[[],[],0]]# 建設状況を記録(建設の推移,最新の建設状況,コスト) 
    #建設の推移:各ステップで建てた道路のindex, 最新の建設状況:[道路配置,道路index,完全に建てられているかのbool]のリスト, コスト:それまでのステップのコスト
    print("first construction_pattern_index",construction_pattern_index)
    for step, (start, goal) in enumerate(allocation):
        binary_list = list(product([0, 1], repeat=n)) 
        all_patterns_step = list([binary_list[i],True]for i in range(len(binary_list))) #stepごとのパターン

        print("all_patterns_step",all_patterns_step)
        construction_step_pattern = []  # 各ステップで建設された仮設道路を記録
        for k in range(len(construction_pattern_index)):
            print("現在扱っているパターン",construction_pattern_index[k])
            current_temp_pattern = copy.deepcopy(construction_pattern_index[k]) 
            construction_pattern_flow  = copy.deepcopy(current_temp_pattern[0]) #建設の推移
            constrution_step = copy.deepcopy(current_temp_pattern[1]) #現在の建設状況
            construction_cost = copy.deepcopy(current_temp_pattern[2]) #現在のコスト
            print("constrution_step",constrution_step) 
            print("construction_pattern",construction_pattern_flow)
            for j in range(len(all_patterns_step)):
                print("step",step)
                print("start:",start,"goal",goal)   

                current_cost = construction_cost
                print("current_cost",current_cost)
                current_pattern_flow = copy.deepcopy(construction_pattern_flow)
                print("current_pattern",current_pattern_flow)
                current_temp = copy.deepcopy(constrution_step) #道路配置,道路index,完全に建てられてるかのboolのリスト
                print("current_temp",current_temp)
                pattern = all_patterns_step[j][0] #新たに建てる仮設道路のパターン
                print("pattern",pattern)
                new_temp = [copy.deepcopy(temp[i]) for i in range(n) if pattern[i] == 1]
                new_temp_index = [i for i in range(n) if pattern[i] == 1]
                new_temp2 = [[copy.deepcopy(temp[i]),i] for i in range(n) if pattern[i] == 1]
                print("new_temp",new_temp)
                print("new_temp_index",new_temp_index)
                print("new_temp2",new_temp2)
                
                current_built_list = [current_temp[i][1] for i in range(len(current_temp)) if current_temp[i][2] == True]
                print("current_built_list",current_built_list)
                # 一本でも仮設道路が建設済みなら無効
                if not len(current_temp) == 0:
                    if any([i in new_temp_index for i in current_built_list]):                        
                        print("already built")
                        continue
                print("not built yet")

                # このパターンで建設された仮設道路を追加
                for new_road,new_index in new_temp2:
                    isInCurrentTemp = False #新たに建てる仮設道路がすでにcurrent_tempに存在するか
                    for j in range(len(current_temp)):
                        if new_index == current_temp[j][1]:
                            current_temp[j][0].update(new_road)
                            current_temp[j][2] = True
                            isInCurrentTemp = True
                            break
                    if not isInCurrentTemp:
                        current_temp.append([new_road,new_index,True])
                    
                            

                print("current_temp after built",current_temp)
                # current_temp = [copy.deepcopy(temp[i]) for i in current_temp]
                
                # A*で最短経路を計算
                path, cost = astar(start, goal, current_temp)
                print("path",path)
                print("cost",cost)

                # 建てた仮設道路を通っていない場合は無効
                isUsed = True
                for i in range(len(new_temp)):
                    if not path_on_road(path, new_temp[i]):
                        print(f"{new_temp[i] } not used")
                        isUsed = False
                        break
                if not isUsed:
                    print("建てた仮設道路が利用されていない")
                    all_patterns_step[j][1] = False
                    continue
                else:
                    print("建てた仮設道路が全て利用されている")

                current_cost += cost
                print("current_cost",current_cost)

                current_temp2 = copy.deepcopy(current_temp)
                # start,goal上の仮設道路を削除
                for i,temp_road in enumerate(current_temp2):
                    print("i",i)
                    print("temp_road",temp_road)
                    road = temp_road[0]
                    if start in road:
                        print("road",road)
                        del temp_road[0][start]
                        print("road after",road)
                        temp_road[2] = False
                    if goal in road:
                        print("road",road)
                        del temp_road[0][goal]
                        print("road after",road)
                        temp_road[2] = False
                print("current_temp2",current_temp2)
                filtered_temp2 = []
                for element in current_temp2:
                    # 空の値を持つキーを削除
                    element[0] = {k: v for k, v in element[0].items() if v}
                    # 要素の先頭部分が空でない場合のみ保持
                    if element[0]:
                        filtered_temp2.append(element)
                current_temp = filtered_temp2
                print("current_temp after filter",current_temp)

                current_pattern_flow.append(pattern)
                construction_step_pattern.append([current_pattern_flow,current_temp,current_cost])
                print("construction_step_pattern",construction_step_pattern)
                print("\n")
            print("\n")
        print("construction_step_pattern after pattern loop",construction_step_pattern)
        construction_pattern_index = construction_step_pattern
        # print("construction_pattern_index",construction_pattern_index)
        print("\n")

    for i in range(len(construction_pattern_index)):
        if construction_pattern_index[i][2] < best_cost:
            best_cost = construction_pattern_index[i][2]
            best_schedule = construction_pattern_index[i][0]
            best_construction_steps = construction_pattern_index[i][1]
    # print("best_cost",best_cost)
    # print("best_schedule",best_schedule)
    # print("best_construction_steps",best_construction_steps)
    return best_schedule, best_cost, best_construction_steps

# テスト
#切土の座標と土量
cut_indices = [[(1, 0),1],[(1, 2),1],[(2, 0),1],[(2, 1),1],[(2, 2),1],[(2, 3),1],[(3, 0),1],[(3, 1),1],[(3, 3),2]]
#盛土の座標と土量
fill_indices = [[(0, 0),2],[(0, 1),1],[(0, 2),1],[(0, 3),1],[(1, 3),2],[(1, 1),1],[(3, 2),2]]


cut_indices_float = []
for i in range(len(cut_indices)):
    new_coords = (float(cut_indices[i][0][0]), float(cut_indices[i][0][1]))  # 新しいタプルを作成
    cut_indices_float.append([new_coords, cut_indices[i][1]])  # 新しいリストに追加

fill_indices_float = []
for i in range(len(fill_indices)):
    new_coords = (float(fill_indices[i][0][0]), float(fill_indices[i][0][1]))  # 新しいタプルを作成
    fill_indices_float.append([new_coords, fill_indices[i][1]])  # 新しいリストに追加

sum_cut = sum(cut_indices[i][1] for i in range(len(cut_indices)))
sum_fill = sum(fill_indices[i][1] for i in range(len(fill_indices)))
# 土量の合計が一致しているか確認
if (sum_cut != sum_fill):
    print("input error:切土と盛土の土量が違います")
    exit() 

soil_amount = np.zeros((grid_size_x, grid_size_y))
for [(i, j),k] in cut_indices:
    soil_amount[int(i), int(j)] = k
for [(i, j),k] in fill_indices:
    soil_amount[int(i),int(j)] = -k
# print("soil_amount",soil_amount)

temporary_roads = [
    {(1, 2): {6}, (2, 2): {1}},
    {(1, 0): {7}, (2, 1): {0}}
]
# allocation = [[(0, 3), (3, 2)], [(1, 0), (2, 0)], [(0, 0), (3, 1)]]
allocation = [[(1, 0), (3, 1)], [(0, 0), (3, 1)]]
best_schedule, best_cost, best_construction_steps = optimize_construction(
    allocation, temporary_roads,soil_amount)

print("最適なスケジュール:", best_schedule)
print("最小コスト:", best_cost)
print("各ステップで建設された仮設道路:", best_construction_steps)
