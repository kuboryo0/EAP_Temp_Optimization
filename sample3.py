import pulp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import time
import copy
# セルの土量設定
# 土量 = np.array([-1000, -4000, -5000, 550, -500, 800, 450, 6700, 2000]).reshape((3, 3))
temp_eff = 0.5
v =  1/temp_eff
tan_alpha = math.sqrt(v**2-1)
sin_alpha = math.sqrt(v**2-1)/v
distance = 0





def calculate_cost(cut_indices, fill_indices,temp):

    def on_segment(p:tuple, q:tuple, r:tuple) -> bool:
            """点rが線分pq上にあるかどうかを確認する関数"""
            if min(p[0], q[0]) < r[0] < max(p[0], q[0]) and min(p[1], q[1]) < r[1] < max(p[1], q[1]):
                return True
            return False


    def is_intersect(p1:float, p2:float, q1:float, q2:float) -> bool:
        def orientation(p:tuple, q:tuple, r:tuple) -> int:
            """3点の並び順を計算。0: コリニア（同一直線上）, 1: 時計回り, 2: 反時計回り"""
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if val == 0:
                return 0  # コリニア
            elif val > 0:
                return 1  # 時計回り
            else:
                return 2  # 反時計回り

        

        # 4つの点 p1, p2, q1, q2 の並びを取得
        o1 = orientation(p1, p2, q1)
        o2 = orientation(p1, p2, q2)
        o3 = orientation(q1, q2, p1)
        o4 = orientation(q1, q2, p2)

        # 交差判定
        # 線分が交差する一般的な場合
        if o1 != o2 and o3 != o4:
            return True

        # 特殊なケース：線分がコリニアで、交差する場合
        if o1 == 0 and on_segment(p1, p2, q1):
            return True
        if o2 == 0 and on_segment(p1, p2, q2):
            return True
        if o3 == 0 and on_segment(q1, q2, p1):
            return True
        if o4 == 0 and on_segment(q1, q2, p2):
            return True

        return False

    def calculate_distance(i:float, j:float, k:float, l:float)->float:
        return np.sqrt((i - k)**2 + (j - l)**2)

    def calculate_intersection(p1:float, p2:float, q1:float, q2:float) -> tuple:
        """p1, p2: 1つ目の仮設道路, q1, q2: 2つ目の仮設道路の端点"""
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = q1
        x4, y4 = q2

        # 行列式を使って交点を計算
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denominator == 0:
            print("denominator == 0")
            return None  # 交差しない（平行または同一線上）

        intersect_x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
        intersect_y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator

        return (intersect_x, intersect_y)


    #ある点(x,y)から仮設道路(temp_i)に角度alphaで下した点の座標を求める
    #また、その点から仮設道路までの距離を返す
    def shortest_point_to_road(x:float,y:float,temp_i:list):
        if not (isinstance(temp_i, list) and len(temp_i) == 2 and
            all(isinstance(point, tuple) and len(point) == 2 and 
                all(isinstance(coord, (int, float)) for coord in point) 
                for point in temp_i)):
            print("shortest_point_to_road error:temp_iは[(p1,q1),(p2,q2)]の型で入力してください")
            return None
        point = []
        if temp_i[0][0] == temp_i[1][0]: #仮設道路が垂直の場合
            a = temp_i[0][0]
            d = abs(x-a)
            if(x != a):
                point1_x = a
                point1_y = y + d/tan_alpha
                point2_x = a
                point2_y = y - d/tan_alpha
                point.append((point1_x,point1_y))
                point.append((point2_x,point2_y))
                distance = d/sin_alpha
            else:
                point.append((x,y))
                distance = 0
        else:
            m = (temp_i[1][1]-temp_i[0][1])/(temp_i[1][0]-temp_i[0][0]) 
            n = (temp_i[0][1]*temp_i[1][0]-temp_i[0][0]*temp_i[1][1])/(temp_i[1][0]-temp_i[0][0])
            beta = math.atan(m)
            d = abs(m*x-y+n)/math.sqrt(m**2+1) 
            #(x,y)から仮設道路に角度alphaで下した2つの点(point1とpoint2)の座標を求める
            if (y > m*x+n): #(x,y)が仮設道路(y = mx+n)より上にある場合
                point1_x = x + d*(math.cos(beta)/tan_alpha+math.sin(beta))
                point1_y = y - d*(math.cos(beta)-math.sin(beta)/tan_alpha)
                point2_x = x - d*(math.cos(beta)/tan_alpha-math.sin(beta))
                point2_y = y - d*(math.cos(beta)+math.sin(beta)/tan_alpha)
                point.append((point1_x,point1_y))
                point.append((point2_x,point2_y))
                distance = calculate_distance(x,y,point1_x,point1_y)
            elif (y < m*x+n): #(x,y)が仮設道路(y = mx+n)より下にある場合
                point1_x = x + d*(math.cos(beta)/tan_alpha-math.sin(beta))
                point1_y = y + d*(math.cos(beta)+math.sin(beta)/tan_alpha)
                point2_x = x - d*(math.cos(beta)/tan_alpha+math.sin(beta))
                point2_y = y + d*(math.cos(beta)-math.sin(beta)/tan_alpha)
                point.append((point1_x,point1_y))
                point.append((point2_x,point2_y))
                distance = calculate_distance(x,y,point1_x,point1_y)
            else: # (x,y)が仮設道路上にある場合
                point.append((x,y))
                distance = 0
        #pointに含まれている座標が仮設道路上にあるかどうかを確認
        for i in range(len(point)):
            if not(on_segment(temp_i[0],temp_i[1],point[i])):
                point[i] = None
        return point,distance

    # 重複を取り除く関数
    def remove_duplicates(temp_list):
        unique_points = set()  # ユニークな座標を保持するセット
        cleaned_temp_list = []

        for sublist in temp_list:
            cleaned_sublist = []
            for item in sublist:
                if isinstance(item, tuple):  # 座標がタプルの場合
                    if item not in unique_points:
                        unique_points.add(item)
                        cleaned_sublist.append(item)
                elif isinstance(item, list):  # 内部リストの場合
                    unique_inner_points = []
                    for inner_item in item:
                        if isinstance(inner_item, tuple) and inner_item not in unique_points:
                            unique_points.add(inner_item)
                            unique_inner_points.append(inner_item)
                    cleaned_sublist.append(unique_inner_points)  # ユニークな内側のリストを追加
                else:
                    cleaned_sublist.append(item)  # Noneや他の値はそのまま追加
            cleaned_temp_list.append(cleaned_sublist)

        return cleaned_temp_list


    distance = 0
    

    temp_number = len(temp)
    temp_list = [] #各仮設道路の端点、交点の座標を格納するリスト(交点がない場合はNoneを格納)
    #交点の存在の確認
    for i in range(temp_number):
        distance_list_tempi = [temp[i][0],temp[i][1]]
        for k in range(temp_number):
            distance_list_tempi.insert(-1,None)
        for j in range(temp_number):
            if(i!= j):
                if is_intersect(temp[i][0],temp[i][1],temp[j][0],temp[j][1]): 
                    distance_list_tempi[j+1] = calculate_intersection(temp[i][0],temp[i][1],temp[j][0],temp[j][1])
        temp_list.append(distance_list_tempi)


    print(temp_list,"\n")

    #各仮設道路の端点、交点から別の仮設道路に角度alphaで下した点を下す
    additional_point = [[] for _ in range(temp_number)] #各仮設道路の端点・交点から別の仮設道路に角度alphaで下した点を格納するリスト
    for i in range(temp_number):
        for j in range(len(temp_list[i])):
            if temp_list[i][j] is not None and isinstance(temp_list[i][j], tuple):
                for k in range(temp_number):
                    if k != i: 
                        point,distance = shortest_point_to_road(temp_list[i][j][0],temp_list[i][j][1],temp[k])
                        # print(f"{i}番目の仮設道路の{j}番目の座標{temp_list[i][j]}から{k}番目の仮設道路に下した座標:",point)
                        for l in range(len(point)):
                            if point[l] != None and distance != 0:
                                # additional_point_tempi.append(point[l]) #下した点を追加
                                additional_point[k].append(point[l])
    # print("additional_point",additional_point)
    additional_point_cleaned = remove_duplicates(additional_point)
    # print("additional_point_cleaned",additional_point_cleaned)
    for k in range(len(temp_list)):
        if isinstance(additional_point_cleaned[k], (list,tuple)) and len(additional_point_cleaned[k]) != 0:
            for l in range(len(additional_point_cleaned[k])):
                temp_list[k].insert(-1,additional_point[k][l])


    # for i in range(len(temp_list)):
    #     print(f"temp_list[{i}])",temp_list[i],"\n")

    temp_list_cleaned = [list(filter(lambda x: x is not None, sublist)) for sublist in temp_list]
    print("temp_list_cleaned",temp_list_cleaned)


    #startとgoalを加えて最短コストをWarshall-Floydを使って計算

    def calculate_distance(i:float, j:float, k:float, l:float)->float:
        return np.sqrt((i - k)**2 + (j - l)**2)


    # Warshall-Floyd法によって仮設道路を用いた最短距離を計算最短
    def Warshall_Floyd(temp:list):
        point_number = sum([len(temp[i]) for i in range(len(temp))])
        distance = np.zeros((point_number,point_number))  
        for i in range(len(temp)):
            for j in range(len(temp[i])):
                index_ij = sum([len(temp[k]) for k in range(i)]) + j
                for k in range(len(temp)):
                    for l in range(len(temp[k])):
                        index_kl = sum([len(temp[m]) for m in range(k)]) + l
                        if i == k:
                            # distance[index_ij][index_kl] = math.sqrt((temp[i][j][0] - temp[k][l][0])**2 + (temp[i][j][1] - temp[k][l][1])**2) * temp_eff
                            distance[index_ij][index_kl] = calculate_distance(temp[i][j][0],temp[i][j][1],temp[k][l][0],temp[k][l][1]) * temp_eff
                        else:
                            distance[index_ij][index_kl] = calculate_distance(temp[i][j][0],temp[i][j][1],temp[k][l][0],temp[k][l][1])
        
        pre_point = np.full((point_number, point_number), [-1])  # 経由点を記録する配列
        for i in range(point_number):
            for j in range(point_number):
                pre_point[i][j] = i
        # for i in range(len(distance)):
        #      print(f"distance[{i}]",distance[i])
        # print("\n")
        for k in range(point_number):
            for i in range(point_number):
                for j in range(point_number):
                    if distance[i][j] > distance[i][k] + distance[k][j]:
                        pre_point[i][j] = pre_point[k][j]
                    distance[i][j] = min(distance[i][j], distance[i][k] + distance[k][j])

        # for i in range(len(distance)):
        #      print(f"distance[{i}]",distance[i])
        return distance[0][-1],pre_point



    #全ての切土、盛土の組み合わせに対して最短経路とそのコストを計算
    def shortest_route_search(cut_indicies, fill_indices,temp_list_cleaned):
        cost = np.zeros((len(cut_indices),len(fill_indices)))
        pre_point_matrix = []  # 経由点を記録する配列
        for i in range(len(cut_indices)):
            for j in range(len(fill_indices)):
                temp_list_cleaned_copy_ij = copy.deepcopy(temp_list_cleaned)
                temp_list_cleaned_copy_ij.insert(0,[cut_indices[i][0]])
                temp_list_cleaned_copy_ij.append([fill_indices[j][0]])
                print("temp_list_cleaned",temp_list_cleaned)
                print("temp_list_cleaned_copy_ij",temp_list_cleaned_copy_ij)
                cost[i][j],pre_point_matrix_value = Warshall_Floyd(temp_list_cleaned_copy_ij)
                print("pre_point_matrix_value",pre_point_matrix_value)
                pre_point_matrix.append(path_search(pre_point_matrix_value,temp_list_cleaned_copy_ij))
        return cost,pre_point_matrix
    # print("temp_list_cleaned after",temp_list_cleaned)


    #経路のindexのリストであるpre_pointを座標リストに整理
    def path_search(pre_point,temp_list_cleaned_copy_ij):
        print("pre_point",pre_point)
        print(len(pre_point))
        print("\n")

        def find_path(pre_point, start, end):
            path = [start]
            while start != end:
                path.insert(1,end)
                end = pre_point[start][end]
            return path
        def clear_online(route):
            delta = 0.000001
            route_copy = copy.deepcopy(route)
            
            i = 1  # 最初の点は残すのでインデックス1から始めます
            while i < len(route_copy) - 1:
                # 点 A, B, C を取得
                A = route_copy[i - 1]
                B = route_copy[i]
                C = route_copy[i + 1]

                # ベクトル AB の傾きと BC の傾きを比較
                if abs((B[1] - A[1]) * (C[0] - B[0]) - (B[0] - A[0]) * (C[1] - B[1])) < delta:
                    # 点 B が A と C の間にある直線上にあれば削除
                    route_copy.pop(i)
                else:
                    # 点 B が直線上にない場合は次の点へ進む
                    i += 1
            
            return route_copy
        path = find_path(pre_point, 0, len(pre_point)-1)
        print("path",path)
        print(f"0番目の点から{len(pre_point)-1}番目の点までの経路:", path)

        temp_list_cleaned2 = []
        route = []
        for i in range(len(temp_list_cleaned_copy_ij)):
            for j in range(len(temp_list_cleaned_copy_ij[i])):
                temp_list_cleaned2.append(temp_list_cleaned_copy_ij[i][j])
        print("temp_list_cleaned2",temp_list_cleaned2)
        for i in range(len(path)):
            route.append(temp_list_cleaned2[int(path[i])])
        print("route",route)
        route = clear_online(route)
        print("route_after",route)
        return route
       
    cost,pre_point_matrix = shortest_route_search(cut_indices, fill_indices,temp_list_cleaned)
    for i in range(len(cost)):
        print(f"cost[{i}]",cost[i])
    print("\n")
    print("temp_list_cleaned",temp_list_cleaned)


# 0番目の点から14番目の点までの経路を探す

    return cost,pre_point_matrix

#切土の座標と土量
cut_indices = [[(1, 0),1],[(1, 2),1],[(2, 0),1], [(2, 1),1], [(2, 2),1],[(3, 0),1],[(3, 1),1]]
#盛土の座標と土量
fill_indices = [[(0, 0),1], [(0, 1),1], [(0, 2),1], [(1, 1),2],[(3, 2),2]]

cut_indices_float = []
for i in range(len(cut_indices)):
    new_coords = (float(cut_indices[i][0][0]), float(cut_indices[i][0][1]))  # 新しいタプルを作成
    cut_indices_float.append([new_coords, cut_indices[i][1]])  # 新しいリストに追加

fill_indices_float = []
for i in range(len(fill_indices)):
    new_coords = (float(fill_indices[i][0][0]), float(fill_indices[i][0][1]))  # 新しいタプルを作成
    fill_indices_float.append([new_coords, fill_indices[i][1]])  # 新しいリストに追加


# 仮設道路のデザイン
temp = [[(0, 0), (0, 1)],[(1, 1),(2, 2)],[(1, 2),(2, 1)]]

start_time = time.time()


sum_cut = sum(cut_indices[i][1] for i in range(len(cut_indices)))
sum_fill = sum(fill_indices[i][1] for i in range(len(fill_indices)))
# 土量の合計が一致しているか確認
if (sum_cut != sum_fill):
    print("input error:切土と盛土の土量が違います")
    exit() 


# コスト行列の作成
costs,route_matrix = calculate_cost(cut_indices_float, fill_indices_float,temp)

print("route_matrix",route_matrix)
print("len(route_matrix)",len(route_matrix))
print("\n")


# 問題の設定
prob = pulp.LpProblem("土砂運搬最適化", pulp.LpMinimize)

# 変数の定義
T = sum_cut  # ステップ数
num_fill = len(fill_indices)
num_cut = len(cut_indices)

# 変数xはステップtで切土cから盛土fに運んだ時、x[t][c][f]=1となる。それ以外は0
x_vars = pulp.LpVariable.dicts("x", (range(T), range(num_cut), range(num_fill)), cat='Binary')
# 変数yはステップtで切土cから盛土fに運んだ土量
# y_vars = pulp.LpVariable.dicts("y", (range(T), range(num_cut), range(num_fill)), lowBound=0, cat='Continuous')

# 目的関数の設定
# objective = pulp.lpSum(costs[c][f] * x_vars[t][c][f] for t in range(T) for c in range(num_cut) for f in range(num_fill))
# prob += objective
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
          objective += costs[c][f] * x_vars[t][c][f]
#盛土地点から次のステップの切土地点への移動コストを追加
for t in range(T-1):
    objective += costs[m[t+1]][n[t]]

prob += objective




# 制約条件
# 各ステップでちょうど1つの切土地点と盛土地点が選ばれる
for t in range(T):
    prob += pulp.lpSum(x_vars[t][c][f] for c in range(num_cut) for f in range(num_fill)) == 1

# 運搬量の制約
# for t in range(T):
#     for c in range(num_cut):
#         for f in range(num_fill):
#             # 運搬量が、土量制約に従う
#             prob += y_vars[t][c][f] <= cut_soil[c] * x_vars[t][c][f]
#             prob += y_vars[t][c][f] <= fill_soil[f] * x_vars[t][c][f]
            
    # 運搬量が、次のステップでの土量に基づく制約
    # if t < T - 1:
    #     for c in range(num_cut):
    #         for f in range(num_fill):
    #             prob += y_vars[t][c][f] == pulp.lpSum(y_vars[t_prime][c][f] for t_prime in range(t + 1))

# 最終ステップで全ての土量がゼロになるように制約
for c in range(num_cut):
    prob += pulp.lpSum(x_vars[t][c][f] for t in range(T) for f in range(num_fill)) == cut_indices[c][1]

for f in range(num_fill):
    prob += pulp.lpSum(x_vars[t][c][f] for t in range(T) for c in range(num_cut)) == fill_indices[f][1]
# 問題の解決
prob.solve()
end_time = time.time()

# #検算
# for t in range(T):
#     x = np.zeros((num_cut,num_fill))
#     for c in range(num_cut):
#         for f in range(num_fill):
#             x[c][f] =pulp.value(x_vars[t][c][f])
#     print(f"xvars[{t}]:\n{x}")

# for c in range(num_cut):
#     a=0
#     for t in range(T):
#         for f in range(num_fill):
#             a += pulp.value(x_vars[t][c][f])
#     print(f"切土{c}の掘削量は{a}")

# for f in range(num_fill):
#     a=0
#     for t in range(T):
#         for c in range(num_cut):
#             a += pulp.value(x_vars[t][c][f])
#     print(f"盛土{f}の積載量は{a}")  


# 結果の表示
print("ステータス:", pulp.LpStatus[prob.status])
print("最小コスト:", pulp.value(prob.objective))
print(f"Calculation Time: {end_time - start_time:.6f} seconds")
# 複数の矢印を描画するための例
arrows = []
route_arrows = []

# 解の表示
for t in range(T):
    print(f"\nステップ {t+1}:")
    for f in range(num_fill):
        for c in range(num_cut):
            if pulp.value(x_vars[t][c][f]) > 0.5:
                print(f"  切土地点 ({cut_indices[c][0]}) から 盛土地点 ({fill_indices[f][0]}) に土が運ばれました")
                arrows.append((cut_indices[c][0],fill_indices[f][0]))
                index = c * len(fill_indices) + f
                route_arrows.append(route_matrix[index])

print("arrows",arrows)
print("route_arrows",route_arrows)
#結果の可視化

# グリッドのサイズ
grid_size_x = 4  # x方向のサイズ
grid_size_y = 3  # y方向のサイズ

# 格子点の座標を生成
x = [i for i in range(grid_size_x)]
y = [i for i in range(grid_size_y)]
X, Y = np.meshgrid(x, y)

# プロット用の格子点を設定
x_coords = X.flatten()
y_coords = Y.flatten()

# 土量マトリックスを作成（仮に色付けのためのデータを用意）
soil_amount = np.zeros((3, 4))
max_soil = max(max(cut_indices[i][1] for i in range(len(cut_indices))),max(fill_indices[i][1] for i in range(len(fill_indices))))
print("max_soil",max_soil)
for [(i, j),k] in cut_indices:
    soil_amount[int(j), int(i)] = 0.5 + k/max_soil/2
for [(i, j),k] in fill_indices:
    soil_amount[int(j),int(i)] = 0.5 - k/max_soil/2

print("soil_amount",soil_amount)

def init_animate (): #初期化関数(これがないとanimate関数のi=0が2回繰り返される)
    pass

def animate(i):
    # 前のフレームの矢印をクリア
    ax.clear()

    print("i:",i)



    #土量を更新
    if i % 2 == 0:
        route_arrows_i = route_arrows[i//2]
        start_point = route_arrows_i[0]
        end_point = route_arrows_i[-1]
        print("start_point", start_point)
        print("end_point", end_point)
        soil_amount[int(start_point[1]), int(start_point[0])] -= 1/max_soil/2  # 開始点の土量を減少
        soil_amount[int(end_point[1]), int(end_point[0])] += 1/max_soil/2 # 終了点の土量を増加
        print("soil_amount",soil_amount)

    # グリッドの描画（背景）
    ax.pcolormesh(soil_amount, edgecolors='gray', linewidth=2, cmap='viridis', shading='flat', alpha=0.2)
    ax.scatter(x_coords+0.5, y_coords+0.5, color='blue', marker='o')  # 格子点のプロット

    # 格子点のラベルを表示（x, y方向に0.5ずらす）
    for x_val, y_val in zip(x_coords, y_coords):
        ax.text(x_val + 0.5, y_val + 0.4, f'({x_val},{y_val})', fontsize=12, ha='center', va='top')
    
    print("temp",temp)
    #仮設道路を描画
    for k in range(len(temp)):
        start_point, end_point = temp[k]
        adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
        adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
        ax.plot([adjusted_start[0], adjusted_end[0]], [adjusted_start[1], adjusted_end[1]], 
                    color='black', linewidth=8, alpha=0.5)
    


    #矢印を描画
    if i <=2 or i == 2*len(route_arrows)-1: #最初と最後の矢印の描画
        for j in range(i + 1):
            # print("j", j)
            #切土から盛土への矢印
            if(j % 2 == 0):
                route_arrows_j = route_arrows[j//2]
                # print("j%2==0")
                # print("route_arrows_j", route_arrows_j)
                if len(route_arrows_j) == 2:
                    start_point, end_point = route_arrows_j
                    adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                    adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                    ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                    arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))

                elif len(route_arrows_j) > 2: #中継点がある場合
                    for k in range(len(route_arrows_j) - 1):
                        start_point = route_arrows_j[k]
                        end_point = route_arrows_j[k+1]
                        adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                        adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                        ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                                    arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))
                # 矢印の番号を矢印の中心に表示
                mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                ax.text(mid_point[0], mid_point[1]+0.1, f'{j//2+1}', fontsize=12, ha='center', color='black')


            #盛土から切土までの矢印
            elif (j%2 == 1 and j//2 <= len(route_arrows)-2):
                # print("j%2==1")
                start_point = route_arrows[j//2][-1]  # 現在の矢印の終了点
                end_point = route_arrows[j//2 + 1][0]  # 次の矢印の開始点
                # print("start_point", start_point)
                # print("end_point", end_point)
                adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                            arrowprops=dict(facecolor='blue', edgecolor='black', linewidth=2, alpha=0.4, shrink=0.05))
                # 矢印の番号を矢印の中心に表示
                mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                ax.text(mid_point[0], mid_point[1]+0.1, f'({j//2+1})', fontsize=12, ha='center', color='grey')




    else: #i>2での矢印の描画(描画される矢印が2つまでにする)
        for j in range(i - 1,i + 1):
            # print("j", j)
            #切土から盛土への矢印
            if(j % 2 == 0):
                route_arrows_j = route_arrows[j//2]
                # print("j%2==0")
                # print("route_arrows_j", route_arrows_j)
                if len(route_arrows_j) == 2:
                    start_point, end_point = route_arrows_j
                    adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                    adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                    ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                            arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))
                elif len(route_arrows_j) > 2:
                    for k in range(len(route_arrows_j) - 1):
                        start_point = route_arrows_j[k]
                        end_point = route_arrows_j[k+1]

                        adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                        adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                        ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                            arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))
                # 矢印の番号を矢印の中心に表示
                mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                ax.text(mid_point[0], mid_point[1]+0.1, f'{j//2+1}', fontsize=12, ha='center', color='black')
                
            #盛土から切土までの矢印
            elif (j%2 == 1 and j//2 <= len(route_arrows)-2):
                # print("j%2==1")
                start_point = route_arrows[j//2][-1]  # 現在の矢印の終了点
                end_point = route_arrows[j//2 + 1][0]  # 次の矢印の開始点
                # print("start_point", start_point)
                # print("end_point", end_point)
                adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                            arrowprops=dict(facecolor='blue', edgecolor='black', linewidth=2, alpha=0.4, shrink=0.05))
                # 矢印の番号を矢印の中心に表示
                mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                ax.text(mid_point[0], mid_point[1]+0.1, f'({j//2+1})', fontsize=12, ha='center', color='grey')
    

            
    
    # グリッド線の描画
    for k in np.arange(-1.0, grid_size_y + 1, 1.0):
        ax.axhline(y=k, color='gray', linestyle='--', linewidth=0.5)
    for k in np.arange(-1.0, grid_size_x + 1, 1.0):
        ax.axvline(x=k, color='gray', linestyle='--', linewidth=0.5)
    
    ax.set_xlim(-0.5, grid_size_x + 0.5)
    ax.set_ylim(-0.5, grid_size_y + 0.5)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title('4x3 Grid Animation with Route Arrows')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')

# アニメーションの準備
fig, ax = plt.subplots(figsize=(8, 6))

# アニメーションの実行
ani = animation.FuncAnimation(fig, animate,init_func=init_animate, frames=2*len(route_arrows) , interval=1000, repeat=False)

# GIFや動画として保存したい場合
# ani.save('animation.gif', writer='imagemagick')

# アニメーションを表示
plt.show()

