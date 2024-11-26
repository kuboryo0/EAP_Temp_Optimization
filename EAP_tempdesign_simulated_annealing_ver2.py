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
from function_ver2 import earth_allocation
from function_ver2 import a_star
from function_ver2 import plot_route
from function_ver2 import temp_length
from function_ver2 import calculate_temporary_road_usage


DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


# 評価関数
def evaluate_design(temporary_roads):
    temp_copy1 = copy.deepcopy(temporary_roads)
    temp_copy2 = copy.deepcopy(temporary_roads)
    "工事作業（時間依存）のコスト"
    #定数の値
    grid_length =150 # 1マスの１辺の長さ(m)
    volume_unit_truck = 40 # トラック1台あたりの土砂輸送量(m3)
    depth_unit = 1 #土量1単位の深さ(m)
    speed_rough_road = 24 # 未舗装道路の速度(km/h)
    speed_paved_road = 36 # 舗装道路の速度(km/h)
    temp_eff = speed_rough_road/speed_paved_road # 仮設道路の効率
    work_eff = 0.75 # 作業効率
    truck_num = 1 # トラックの台数
    cost_hour = 2000# トラック1台1時間あたりのコスト($/h)
    #コスト計算
    route,cost1 = a_star(allocation,temp_copy1,grid_size_x,grid_size_y,temp_eff)
    print("運搬距離コスト",cost1)
    # print("cost1",cost1)
    soil_volume =  grid_length**2 * depth_unit # 運搬する土砂の総量(m3)
    # print("soil_volume",soil_volume)
    time_average = cost1*grid_length / (speed_rough_road*1000) # 1台トラックの所要時間(h) 
    # print("time_average",time_average)
    cost_construction = cost_hour * soil_volume/(truck_num * volume_unit_truck) * time_average / work_eff # 作業コスト($)
    "仮設道路の建設にかかるコスト"
    #定数の値
    construction_temp = 17.5 # 仮設道路の建設単位長さあたりのコスト($/m)
    #コスト計算
    cost2 = temp_length(temp_copy2)
    cost_construction_temp = cost2 * construction_temp * grid_length# 仮設道路の建設コスト($)
    #総コスト
    total_cost = cost_construction + cost_construction_temp

    print("cost_construction:$",cost_construction)
    print("cost_construction_temp:$",cost_construction_temp)
    print("total_cost:$",total_cost)
    return route,total_cost



def generate_neighbor(route,temp):
    # 新規仮設道路のデザインを生成

    filtered_temp = [road for road in temp if road]
    temp = filtered_temp
    print("temp before",temp)
    
    # 仮設道路の使用量を計算
    usage_weights = []
    for i in range(len(temp)):
        usage = calculate_temporary_road_usage(route, temp[i])
        # print("temp[i]",temp[i])
        usage_weights.append((temp[i], usage))
    print("usage_weights",usage_weights)
    # 使用量が低い道路のリストを作成
    low_usage_roads = sorted(usage_weights, key=lambda x: x[1])  # 使用量が低い順にソート
    # print("low_usage_roads",low_usage_roads)

    # 使用量が0の道路を削除
    low_usage_roads = [road for road in low_usage_roads if road[1] > 0]
    print("low_usage_roads after filtering:", low_usage_roads)

    # tempから使用量が0だった道路を削除
    remaining_roads = [road for road, _ in low_usage_roads]  # low_usage_roadsに残った道路を抽出
    temp = [road for road in temp if road in remaining_roads]
    print("temp after filtering:", temp)

    if len(temp) == 0 or random.random() < 0.15:  # 仮設道路なしor15%の確率で要素を追加
        print("道路を追加")
        new_temporary_road = {}
        coord = (random.randint(0, 3), random.randint(0, 3))  # ランダムな座標
        while True:
            new_direction_index = random.randint(0, len(DIRECTIONS) - 1)
            # print("new_direction_index",new_direction_index)
            # print("DIRECTIONS[new_direction_index]",DIRECTIONS[new_direction_index])
            # print("new_coord",coord[0]+  DIRECTIONS[new_direction_index][0], coord[1]+  DIRECTIONS[new_direction_index][1])
            if  0 <= coord[0]+  DIRECTIONS[new_direction_index][0] <= grid_size_x-1 and 0 <= coord[1]+ DIRECTIONS[new_direction_index][1] <= grid_size_y-1:
                new_neighbor_coord = (coord[0]+  DIRECTIONS[new_direction_index][0], coord[1]+  DIRECTIONS[new_direction_index][1])
                break
        new_temporary_road[coord] = {new_direction_index}
        neighbor_index = DIRECTIONS.index((-DIRECTIONS[new_direction_index][0], -DIRECTIONS[new_direction_index][1]))
        new_temporary_road[new_neighbor_coord] = {neighbor_index}
        # print("new_temporary_road",new_temporary_road)
        temp.append(new_temporary_road)

    else:
        # 重みを考慮してランダムに道路を選択
        total_weight = sum(1 / (usage + 1e-6) for _, usage in low_usage_roads)  # 小さい値を追加してゼロ除算を防止
        probabilities = [(1 / (usage + 1e-6)) / total_weight for _, usage in low_usage_roads]
        print("total_weight",total_weight)
        print("probabilities",probabilities)
        # 確率に基づいて選択
        road_index = np.random.choice(len(low_usage_roads), p=probabilities)
        print("road_index",road_index)
        selected_road = low_usage_roads[road_index][0]
        print("selected_road",selected_road)
        road_probability_list =[]
        for i in range(len(temp)):
            road_probability_list.append([low_usage_roads[i],probabilities[i]])
        
        print("neighbor temp",temp)
        if random.random() < 0.15: # 15%の確率で要素を削除
            print("道路を削除")
            selected_road = random.choice(temp)
            print("selected_road",selected_road)
            temp.remove(selected_road)

        else:  # 残りの確率で既存の要素をランダムに変更
            selected_road = random.choice(temp)
            print("selected_road",selected_road)
            selected_coord = list(selected_road.keys())[0] if random.choice([True, False]) else list(selected_road.keys())[-1]
            print("selected_coord",selected_coord)
            if random.choice([True, False]):
                # 端点を削除
                print("端点を削除")
                if len(selected_road[selected_coord]) > 0:
                    selected_road[selected_coord].remove(random.choice(list(selected_road[selected_coord])))
                # 値が空の場合はキー（座標）を削除
                if not selected_road[selected_coord]:  # 座標のセットが空ならTrue
                    del selected_road[selected_coord]

            else:
                # 新しい点を追加
                print("端点を追加")
    #             new_direction = random.randint(0, len(DIRECTIONS) - 1)
    #             print("new_direction",new_direction)
    #             if new_direction not in selected_road[selected_coord]:
    #                 selected_road[selected_coord].add(new_direction)
    #                 # 新しい座標を計算
    #                 dx, dy = DIRECTIONS[new_direction]
    #                 new_coord = (selected_coord[0] + dx, selected_coord[1] + dy)
    #                 # 新しい座標の方向を追加
    #                 reverse_direction = DIRECTIONS.index((-dx, -dy))
    #                 if new_coord not in selected_road:
    #                     selected_road[new_coord] = set()
    #                 selected_road[new_coord].add(reverse_direction)
                while True:
                    new_direction_index = random.randint(0, len(DIRECTIONS) - 1)
                    # print("new_direction_index",new_direction_index)
                    # print("DIRECTIONS[new_direction_index]",DIRECTIONS[new_direction_index])
                    # print("new_coord",coord[0]+  DIRECTIONS[new_direction_index][0], coord[1]+  DIRECTIONS[new_direction_index][1])
                    if  0 <= selected_coord[0]+  DIRECTIONS[new_direction_index][0] <= grid_size_x-1 and 0 <= selected_coord[1]+ DIRECTIONS[new_direction_index][1] <= grid_size_y-1:
                        new_neighbor_coord = (selected_coord[0]+  DIRECTIONS[new_direction_index][0], selected_coord[1]+  DIRECTIONS[new_direction_index][1])
                        break
                print("selected_coord",selected_coord)
                print("new_neighbor_coord",new_neighbor_coord)
                selected_road[selected_coord].add(new_direction_index)
                neighbor_index = DIRECTIONS.index((-DIRECTIONS[new_direction_index][0], -DIRECTIONS[new_direction_index][1]))
                selected_road[new_neighbor_coord] = {neighbor_index}
    print("new temp",temp)
    return temp

def simulated_annealing(temp):
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
    # selected_road_flow = []
    # road_probability_list = []
    for _ in range(max_iter):
        print("simulated anealing loop",_)
        temperature *= alpha
        current_route_copy = copy.deepcopy(current_route)
        current_solution_copy = copy.deepcopy(current_solution)
        neighbor_solution= generate_neighbor(current_route_copy,current_solution_copy)
        neighbor_route,neighbor_score = evaluate_design(neighbor_solution)


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




# 仮設道路のデザイン


# temporary_roads = {
# }
grid_size_x = 4
grid_size_y = 4
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



start_time = time.time()

#土砂の分配を計画(固定)
allocation = earth_allocation(cut_indices_float, fill_indices_float)

print("allocation",allocation)

temporary_roads = [
    {
    (1, 3): {6},
    (2, 3):{1}
    }
]
# temporary_roads = [
# ]
# 初期解の評価
route_first,cost_first = evaluate_design(temporary_roads)

print("初期ルート",route_first)
print("初期解のコスト",cost_first)
print("初期解",temporary_roads)
print("\n")
# plot_route(grid_size_x,grid_size_y,route_first,temporary_roads, cut_indices, fill_indices)

# パラメータの設定
initial_temprature = 1000
# final_temprature = 1
alpha = 0.95
max_iter = 1000


# 最適化の実行
optimized_solution, optimized_score, optimized_route,best_solution_flow,current_solution_flow,neighbor_solution_flow,best_score_loop = simulated_annealing(temporary_roads)

end_time = time.time()
print("最適化されたデザイン:", optimized_solution)
print("最適スコア:", optimized_score)
print("最適ルート:", optimized_route)
print("最適化されたデザインが出たループ",best_score_loop)
print("処理時間:", end_time - start_time, "秒")

plot_route(grid_size_x,grid_size_y,optimized_route,optimized_solution, cut_indices, fill_indices)

# for i in range(len(best_solution_flow)):
#     print(f"{i+1}回目")
#     print("best_solution",best_solution_flow[i][1])
#     print("best_solution_score",best_solution_flow[i][0])
#     print("neighbor_solution",current_solution_flow[i][1])
#     print("neighbor_solution_score",current_solution_flow[i][0])
#     print("current_solution",current_solution_flow[i][1])
#     print("current_solution_score",current_solution_flow[i][0])
#     print("\n")

grid_size_x = 4
grid_size_y = 4 

current_score_list = [current_solution_flow[i][0] for i in range(len(current_solution_flow))]
# Best Solution Scoreをリスト化
best_score_list = [best_solution_flow[i][0] for i in range(len(best_solution_flow))]

# グラフのプロット
plt.figure(figsize=(10, 6))  # グラフサイズを設定
plt.plot(current_score_list, marker='x', label="Current Solution Score", color='blue')  # Current Solution Scoreをプロット
plt.plot(best_score_list, marker='o', label="Best Solution Score", color='orange') 
plt.xlabel("Index")  # 横軸ラベル
plt.ylabel("Score")  # 縦軸ラベル
plt.title("Solution Score")  # タイトル
plt.grid()  # グリッド線を追加
plt.show()




