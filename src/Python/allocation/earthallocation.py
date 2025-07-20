import numpy as np
import time
import pulp
#問題設定
# グリッドサイズ
grid_size_x = 8
grid_size_y = 4
#仮説道路上を走る際のスピード効率(0~1の間)
temp_eff = 0.25

# #切土の座標と土量
# cut_indices = [[(1, 2),0],[(2, 0),0],[(2, 1),0],[(2, 2),0],[(2, 3),0],[(3, 0),0],[(3, 1),0],[(3, 3),0]]
# #盛土の座標と土量
# fill_indices = [[(0, 0),0],[(0, 1),0],[(0, 2),0],[(0, 3),0],[(1, 0),0],[(1, 3),0],[(1, 1),0],[(3, 2),0]]

# #切土の座標と土量
# cut_indices = [[(1, 2),2],[(2, 0),1],[(2, 1),1],[(2, 2),1]]
# #盛土の座標と土量
# fill_indices = [[(0, 0),1],[(0, 1),1],[(0, 2),1],[(1, 0),1],[(1, 1),1]]
# 切土の座標と土量
cut_indices = [
    [(2, 0), 3700], [(3, 0), 9000], [(4, 0), 9000], [(5, 0), 8000],
    [(1, 1), 22500], [(2, 1), 33800], [(3, 1), 36000], [(4, 1), 23000], [(5, 1), 22200], [(6, 1), 8100], 
    [(1, 2), 22500], [(2, 2), 28100], [(3, 2), 23000], [(4, 2), 24300], [(5, 2), 14200],
    [(2, 3), 2300], [(3, 3), 1200], [(4, 3), 9000]
]

# 盛土の座標と土量
fill_indices = [
    [(0, 0), 126200], [(1, 0), 3700], [(6, 0), 1000], [(7, 0), 11200], 
    [(0, 1), 62600], [(7, 1), 24800],
    [(0, 2), 3700], [(6, 2), 12400], [(7, 2), 34400],
    [(1, 3), 1400], [(5, 3), 5900], [(6, 3), 9900], [(7, 3), 2700]
]

# ゼロの座標
zero_indices = [

    [(0, 3), 0], 
]

num_cut = len(cut_indices)
num_fill = len(fill_indices)

total_cut = sum(cut_indices[i][1] for i in range(len(cut_indices)))
total_fill = sum(fill_indices[i][1] for i in range(len(fill_indices)))
print(total_cut)
print(total_fill)

soil_amount = np.zeros((grid_size_x, grid_size_y))
for [(i, j),k] in cut_indices:
    soil_amount[int(i), int(j)] = k
for [(i, j),k] in fill_indices:
    soil_amount[int(i),int(j)] = -k
for [(i, j),k] in zero_indices:
    soil_amount[int(i),int(j)] = 0


def calculate_distance(a,b,soil_amount):
    return np.sqrt(((a[0]-b[0])*150)**2+((a[1]-b[1])*150)**2+((soil_amount[a[0],a[1]]-soil_amount[b[0],b[1]])/150/150)**2)

# 移動コストの初期化
# 各セル間の移動コストを無限大で初期化
all_cost = np.full((grid_size_x * grid_size_y, grid_size_x * grid_size_y), np.inf)

# セル間の基本コストを設定 (1ユニットとする)
for i in range(grid_size_x):
    for j in range(grid_size_y):
        index = i * grid_size_y + j
        all_cost[index][index] = 0  # 自分自身へのコストは0

        # 隣接するセルのコストを設定
        if i + 1 < grid_size_x:  # 右
            all_cost[index][(i + 1) * grid_size_y + j] = calculate_distance((i,j),(i+1,j),soil_amount)
        if j + 1 < grid_size_y:  # 下
            all_cost[index][i * grid_size_y + (j + 1)] = calculate_distance((i,j),(i,j+1),soil_amount)
        if i - 1 >= 0:  # 左
            all_cost[index][(i - 1) * grid_size_y + j] = calculate_distance((i,j),(i-1,j),soil_amount)
        if j - 1 >= 0:  # 上
            all_cost[index][i * grid_size_y + (j - 1)] = calculate_distance((i,j),(i,j-1),soil_amount)

        # 斜め方向のセルのコストを設定 (√2)
        if i + 1 < grid_size_x and j + 1 < grid_size_y:  # 右下
            all_cost[index][(i + 1) * grid_size_y + (j + 1)] = calculate_distance((i,j),(i+1,j+1),soil_amount)
        if i + 1 < grid_size_x and j - 1 >= 0:  # 右上
            all_cost[index][(i + 1) * grid_size_y + (j - 1)] = calculate_distance((i,j),(i+1,j-1),soil_amount)
        if i - 1 >= 0 and j + 1 < grid_size_y:  # 左下
            all_cost[index][(i - 1) * grid_size_y + (j + 1)] = calculate_distance((i,j),(i-1,j+1),soil_amount)
        if i - 1 >= 0 and j - 1 >= 0:  # 左上
            all_cost[index][(i - 1) * grid_size_y + (j - 1)] = calculate_distance((i,j),(i-1,j-1),soil_amount)

# Warshall-Floyd法を用いて全頂点間の最短コストを計算
for k in range(grid_size_x * grid_size_y):
    for i in range(grid_size_x * grid_size_y):
        for j in range(grid_size_x * grid_size_y):
            index_i_x = i //grid_size_x
            index_i_y = i % grid_size_x
            index_j_x = j //grid_size_x
            index_j_y = j % grid_size_x            
            all_cost[i][j] = min(all_cost[i][j], all_cost[i][k] + all_cost[k][j])

cost = np.zeros((num_cut,num_fill))

for i in range(len(cut_indices)):
    for j in range(len(fill_indices)):
        cost[i][j] = all_cost[cut_indices[i][0][0]*grid_size_y + cut_indices[i][0][1]][fill_indices[j][0][0]*grid_size_y + fill_indices[j][0][1]]
print(cost)

# 問題の設定
prob = pulp.LpProblem("土砂運搬最適化", pulp.LpMinimize)

# # 変数の定義
# 変数xは切土cから盛土fに運んだ土量を表す
x_vars = pulp.LpVariable.dicts("x", (range(num_cut), range(num_fill)), cat="Integer", lowBound=0)

# # 目的関数
objective = pulp.LpAffineExpression()
#各ステップで切土から盛土に土を運んだ時のコスト
for f in range(num_fill):
    for c in range(num_cut):
        objective += cost[c][f] * x_vars[c][f]/40
prob += objective

# # 制約条件
# 全ての区間で初めにある土量分運ばれるように制約
for c in range(num_cut):
    prob += pulp.lpSum(x_vars[c][f] for f in range(num_fill)) == cut_indices[c][1]

for f in range(num_fill):
    prob += pulp.lpSum(x_vars[c][f] for c in range(num_cut)) == fill_indices[f][1]
# 問題の解決
prob.solve()

# 結果の表示
# print("ステータス:", pulp.LpStatus[prob.status])
# print("最小コスト:", pulp.value(prob.objective))
# 複数の矢印を描画するための例


# 土砂分配をリスト化
allocation = []
for f in range(num_fill):
    for c in range(num_cut):
        if pulp.value(x_vars[c][f]) != 0:
            print(f"  切土地点 ({cut_indices[c][0]}) から 盛土地点 ({fill_indices[f][0]}) に土が{pulp.value(x_vars[c][f])}運ばれました")
            allocation.append((cut_indices[c][0],fill_indices[f][0],pulp.value(x_vars[c][f])))

def calculate_distance_2D(a,b):
    return np.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2)
def further_point_distance(allocation): 
    further = calculate_distance_2D(allocation[0],entrance)
    nearer = calculate_distance_2D(allocation[1],entrance)
    if further<nearer:
       further,nearer = nearer,further
    return further,nearer

entrance = (0,0)
# 全ての allocation の要素について further_point_distance を計算し、大きい順に並べる
further_allocation_list = []
for pair in allocation:
    distance = further_point_distance(pair)
    further_allocation_list.append((pair, distance))

# 距離の1番目が大きい順、1番目が同じ場合は2番目が大きい順にソート
further_allocation_list.sort(key=lambda x: (x[1][0], x[1][1]), reverse=True)

allocation = [x[0] for x in further_allocation_list]
# print(allocation)

    #allocationを表示.すべて{{切土のx座標,切土のy座標},{盛土のx座標,盛土のy座標},{土量}}}の形式
print("{")
for cut, fill, amount in allocation:
    print(f"  {{{{{cut[0]}, {cut[1]}}}, {{{fill[0]}, {fill[1]}}}, {amount}}},")
print("}")