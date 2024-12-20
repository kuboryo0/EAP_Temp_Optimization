#simulated annealingを用いた仮設道路デザインの最適化

import numpy as np
import random
import pulp
import time
import math
import matplotlib.pyplot as plt
from temp_placement.function import calculate_cost
from temp_placement.function import plot
from temp_placement.function import temp_length
from temp_placement.function import temp_usage
temp_eff = 0.7
v =  1/temp_eff
tan_alpha = math.sqrt(v**2-1)
sin_alpha = math.sqrt(v**2-1)/v
distance = 0




# 評価関数
def evaluate_design(temp,cut_indices_float, fill_indices_float):
    # 道路の交差や長さなどの評価を行う
    costs,route_matrix = calculate_cost(cut_indices_float, fill_indices_float,temp)
    # 評価基準に基づいてスコアを計算（例: 道路の長さ）
    # print("costs",costs)
    # for i in range(len(route_matrix)):
    #     print("route_matrix",route_matrix[i])
# 問題の設定
    prob = pulp.LpProblem("土砂運搬最適化", pulp.LpMinimize)

    # 変数の定義
    T = sum_cut  # ステップ数
    num_fill = len(fill_indices_float)
    num_cut = len(cut_indices_float)

    # 変数xはステップtで切土cから盛土fに運んだ時、x[t][c][f]=1となる。それ以外は0
    x_vars = pulp.LpVariable.dicts("x", (range(T), range(num_cut), range(num_fill)), cat='Binary')
    objective = pulp.LpAffineExpression()

    #目的関数
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
    #盛土地点から次のステップの切土地点への移動コストを追加
    for t in range(T-1):
        objective += costs[m[t+1]][n[t]]

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

    route_list = []
    for t in range(T):
        # print(f"\nステップ {t+1}:")
        for f in range(num_fill):
            for c in range(num_cut):
                if pulp.value(x_vars[t][c][f]) > 0.5:
                    # print(f"  切土地点 ({cut_indices[c][0]}) から 盛土地点 ({fill_indices[f][0]}) に土が運ばれました")
                    index = c * len(fill_indices) + f
                    route_list.append(route_matrix[index])                    
    # print("最適なコスト:", pulp.value(prob.objective))
    # print("仮設道路の長さ",temp_length(temp))
    score = pulp.value(prob.objective) + temp_length(temp)
    return score,route_list



def generate_neighbor(route,temp):


    usage_weights = [(road, temp_usage(route, road)) for road in temp]
    print("usage_weights",usage_weights)
    # 使用量が低い道路のリストを作成
    low_usage_roads = sorted(usage_weights, key=lambda x: x[1])  # 使用量が低い順にソート
    print("low_usage_roads",low_usage_roads)
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
        

    if random.random() < 0.3:  # 30%の確率で追加または削除
        if len(temp) > 1 and random.random() < 0.5:  # 15%の確率で削除
            # 道路を削除
            temp.remove(selected_road)
        else:
            # 新しい道路を追加
            new_road = [(random.randint(0, 3), random.randint(0, 3)), 
                        (random.randint(0, 3), random.randint(0, 3))]  # 適当な範囲
            temp.append(new_road)
    else:
            # ランダムに一本の道路を選択し、その始点または終点を変更
        road_index_change = temp.index(selected_road)
        # 始点または終点をランダムに選択
        if random.choice([True, False]):
            # 始点をランダムに変更
            new_start = (random.randint(0, 3), random.randint(0, 3))  # 適当な範囲
            temp[road_index_change] = [new_start, selected_road[1]]
        else:
            # 終点をランダムに変更
            new_end = (random.randint(0, 3), random.randint(0, 3))  # 適当な範囲
            temp[road_index_change] = [selected_road[0], new_end]
    print("new temp",temp)
    return temp,selected_road,road_probability_list

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
temp = [[(1,3),(2,3)],[(3,0),(3,1)],[ (0, 0), (0, 1)]]

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
# 初期解の評価
initial_score = evaluate_design(temp,cut_indices_float, fill_indices_float)
print("初期スコア:", initial_score)



# パラメータの設定
initial_temp = 1000
final_temp = 1
alpha = 0.95
max_iter = 50


# 最適化の実行
optimized_solution, optimized_score, optimized_route,best_solution_flow,current_solution_flow,neighbor_solution_flow,selected_road_flow,road_probability_list = simulated_annealing(temp, initial_temp, final_temp, alpha, max_iter, cut_indices_float, fill_indices_float)

end_time = time.time()
print("最適化されたデザイン:", optimized_solution)
print("最適スコア:", optimized_score)
print("最適ルート:", optimized_route)
print("処理時間:", end_time - start_time, "秒")


for i in range(len(best_solution_flow)):
    print(f"{i+1}回目")
    print("best_solution",best_solution_flow[i][1])
    print("best_solution_score",best_solution_flow[i][0])
    print("neighbor_solution",current_solution_flow[i][1])
    print("neighbor_solution_score",current_solution_flow[i][0])
    print("road_probability",road_probability_list[i])
    print("selected_road",selected_road_flow[i])
    print("current_solution",current_solution_flow[i][1])
    print("current_solution_score",current_solution_flow[i][0])
    print("\n")

grid_size_x = 4
grid_size_y = 4 

plot(grid_size_x,grid_size_y,optimized_route,optimized_solution, cut_indices, fill_indices)
# ani.save('animation.gif', writer='imagemagick')
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