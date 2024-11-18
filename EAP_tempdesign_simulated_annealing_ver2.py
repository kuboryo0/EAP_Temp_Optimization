#simulated annealingを用いた仮設道路デザインの最適化
#地形変化も考慮。土砂の割り当て方は事前に計算してから仮設道路の最適化を行う
import numpy as np
import random
import pulp
import time
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from function_ver2 import earth_allocation
from function_ver2 import a_star
from function_ver2 import plot_route
from function_ver2 import temp_length
from function_ver2 import temp_usage

temp_eff = 0.7
v =  1/temp_eff
tan_alpha = math.sqrt(v**2-1)
sin_alpha = math.sqrt(v**2-1)/v
distance = 0




# 評価関数
def evaluate_design(allocation,temporary_roads,grid_size_x,grid_size_y):
    # 道路の長さと移動経路に関する評価関数
    solution,cost = a_star(allocation,temporary_roads,grid_size_x,grid_size_y)
    temp_cost = temp_length(temporary_roads)
    total_cost = cost + temp_cost
    return solution,total_cost



def generate_neighbor(route,temp):


    # usage_weights = [(road, temp_usage(route, road)) for road in temp]
    # # print("usage_weights",usage_weights)
    # # 使用量が低い道路のリストを作成
    # low_usage_roads = sorted(usage_weights, key=lambda x: x[1])  # 使用量が低い順にソート
    # # print("low_usage_roads",low_usage_roads)
    # # 重みを考慮してランダムに道路を選択
    # total_weight = sum(1 / (usage + 1e-6) for _, usage in low_usage_roads)  # 小さい値を追加してゼロ除算を防止
    # probabilities = [(1 / (usage + 1e-6)) / total_weight for _, usage in low_usage_roads]
    # # print("total_weight",total_weight)
    # # print("probabilities",probabilities)
    # # 確率に基づいて選択
    # road_index = np.random.choice(len(low_usage_roads), p=probabilities)
    # # print("road_index",road_index)
    # selected_road = low_usage_roads[road_index][0]
    # # print("selected_road",selected_road)
    # road_probability_list =[]
    # for i in range(len(temp)):
    #     road_probability_list.append([low_usage_roads[i],probabilities[i]])
        
    selected_road = random.choice(temp)
    if random.random() < 0.2:  # 20%の確率で追加または削除
        if len(temp) > 1 and random.random() < 0.5:  # 10%の確率で削除
            # 道路を削除
            temp.remove(selected_road)
        else:
            # 新しい道路を追加
            new_road = [(random.randint(0, 3), random.randint(0, 3)), 
                        (random.randint(0, 3), random.randint(0, 3))]  # 適当な範囲
            temp.append(new_road)
    else:
            # ランダムに一本の道路を選択.その道路の端点を削除or追加
        road_index_change = temp.index(selected_road)
        # 端点をランダムに選択
        if random.choice([True, False]):
            new_direction = random.randint(0, 7)  # 適当な範囲
            # 始点を削除
            if random.random() < 0.5 or new_direction in selected_road[0]:
                selected_road.pop(0)  # 選択した道路の始点を削除
            #始点に追加
            else:
                selected_road.insert(0, new_direction)

                
        else:
            # 終点をランダムに変更
            new_end = (random.randint(0, 3), random.randint(0, 3))  # 適当な範囲
            temp[road_index_change] = [selected_road[0], new_end]
    print("new temp",temp)
    # return temp,selected_road,road_probability_list
    return temp,selected_road

def simulated_annealing(temp, initial_temp, final_temp, alpha, max_iter, cut_indices_float, fill_indices_float):
    current_solution = temp
    current_score,current_route = evaluate_design(current_solution,cut_indices_float, fill_indices_float)
    best_solution = current_solution
    best_score = current_score
    best_route = current_route
    temperature = initial_temp

    best_solution_flow = []
    current_solution_flow = []
    neighbor_solution_flow = []
    selected_road_flow = []
    road_probability_list = []
    for _ in range(max_iter):
        print("simulated anealing loop",_)
        temperature *= alpha
        neighbor_solution,selected_road,road_probability = generate_neighbor(current_route.copy(),current_solution.copy())
        neighbor_score,neighbor_route = evaluate_design(neighbor_solution,cut_indices_float, fill_indices_float)

        # 新しい解が良い場合、または確率的に受け入れる
        if (neighbor_score < current_score) or (random.random() < np.exp(-( np.abs(neighbor_score - current_score) ) / temperature)):
            current_solution = neighbor_solution
            current_score = neighbor_score

            # ベスト解を更新
        if current_score < best_score:
            best_solution = current_solution
            best_score = current_score
            best_route = neighbor_route
        
        best_solution_flow.append([best_score, best_solution])
        current_solution_flow.append([current_score, current_solution])
        neighbor_solution_flow.append([neighbor_score, neighbor_solution])
        selected_road_flow.append(selected_road)
        road_probability_list.append(road_probability)
        print("current_solution",current_solution)
        print("current_score",current_score)
        print("best_solution",best_solution)
        print("best_score",best_score)
    # 温度を下げる


    return best_solution, best_score, best_route ,best_solution_flow,current_solution_flow,neighbor_solution_flow,selected_road_flow,road_probability_list




# 仮設道路のデザイン

temporary_roads = [
    {
    (2, 0): {2},  
    (1, 1): {2,5},  
    (0, 2): {5}
    },
    {
    (1, 2): {2},
    (0, 3):{5}
    }
]
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

#土砂の分配を計画
allocation = earth_allocation(cut_indices_float, fill_indices_float)

# 初期解の評価
solution_first,cost_first = evaluate_design(allocation,temporary_roads,grid_size_x,grid_size_y)

print("初期解",solution_first)
print("初期解のコスト",cost_first)

plot_route(grid_size_x,grid_size_y,solution_first,temporary_roads, cut_indices, fill_indices)

# パラメータの設定
# initial_temp = 1000
# final_temp = 1
# alpha = 0.95
# max_iter = 50


# # 最適化の実行
# optimized_solution, optimized_score, optimized_route,best_solution_flow,current_solution_flow,neighbor_solution_flow,selected_road_flow,road_probability_list = simulated_annealing(temp, initial_temp, final_temp, alpha, max_iter, cut_indices_float, fill_indices_float)

# end_time = time.time()
# print("最適化されたデザイン:", optimized_solution)
# print("最適スコア:", optimized_score)
# print("最適ルート:", optimized_route)
# print("処理時間:", end_time - start_time, "秒")


# for i in range(len(best_solution_flow)):
#     print(f"{i+1}回目")
#     print("best_solution",best_solution_flow[i][1])
#     print("best_solution_score",best_solution_flow[i][0])
#     print("neighbor_solution",current_solution_flow[i][1])
#     print("neighbor_solution_score",current_solution_flow[i][0])
#     print("road_probability",road_probability_list[i])
#     print("selected_road",selected_road_flow[i])
#     print("current_solution",current_solution_flow[i][1])
#     print("current_solution_score",current_solution_flow[i][0])
#     print("\n")

# grid_size_x = 4
# grid_size_y = 4 

# plot(grid_size_x,grid_size_y,optimized_route,optimized_solution, cut_indices, fill_indices)
# # ani.save('animation.gif', writer='imagemagick')
# current_score_list = []
# for i in range(len(best_solution_flow)):
#     current_score_list.append(current_solution_flow[i][0])
# # グラフのプロット
# plt.plot(current_score_list, marker='x',label="Current Solution Score")  # marker='x' で点も表示
# plt.xlabel("Index")  # 横軸ラベル
# plt.ylabel("Score")  # 縦軸ラベル
# plt.title("Solution Score")  # タイトル
# plt.grid()  # グリッド線を追加
# plt.show()




