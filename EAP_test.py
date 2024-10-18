import pulp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# セルの土量設定
# 土量 = np.array([-1000, -4000, -5000, 550, -500, 800, 450, 6700, 2000]).reshape((3, 3))

# 距離コストの計算
def calculate_distance(i, j, k, l):
    return np.sqrt((i - k)**2 + (j - l)**2)

def calculate_cost(dist,temp):
    # コスト行列の初期化（距離そのものをコストとして使用）
    if len(temp) != 0 :
        cost = dist.copy()
    # 仮設道路の効果を反映
        temp_eff = 0.7  # 仮設道路でコストが30%減少すると仮定
        for (cut_idx, fill_idx) in temp:
            cost[cut_idx][fill_idx] = dist[cut_idx][fill_idx] * temp_eff
        return cost
    elif len(temp) == 0:
        return dist
    else:
        print("temprory road does not exit ")
        exit()

    
def animate(i):
    # 前のフレームの矢印をクリア
    ax.clear()
    
    # グリッドの描画（背景）
    ax.pcolormesh(soil_amount, edgecolors='gray', linewidth=2, cmap='viridis', shading='flat', alpha=0.2)
    ax.scatter(x_coords+0.5, y_coords+0.5, color='blue', marker='o')  # 格子点のプロット

    # 格子点のラベルを表示（x, y方向に0.5ずらす）
    for x_val, y_val in zip(x_coords, y_coords):
        ax.text(x_val + 0.5, y_val + 0.4, f'({x_val},{y_val})', fontsize=12, ha='center', va='top')
    
    # 現在のステップまでの矢印を描画
    for j in range(i + 1):
        if(j % 2 == 0):
            start_point, end_point = arrows[j//2]
            adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
            adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
            ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                        arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))
            # 矢印の番号を矢印の中心に表示
            mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
            ax.text(mid_point[0], mid_point[1]+0.1, f'{j//2+1}', fontsize=12, ha='center', color='black')
        # elif (j%2 == 1 and j<= len(arrows)-1):
        #     start_point = arrows[j][1]  # 現在の矢印の終了点
        #     end_point = arrows[j + 1][0]  # 次の矢印の開始点
        #     adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
        #     adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
        #     ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
        #                 arrowprops=dict(facecolor='blue', 
        #                                 edgecolor='black', 
        #                                 linewidth=3, 
        #                                 alpha=0.3,  # 透明度を設定
        #                                 shrink=0.05))
        #     mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, 
        #                 (adjusted_start[1] + adjusted_end[1]) / 2)
        #     ax.text(mid_point[0], mid_point[1]+0.1, f'({i+1})', fontsize=12, ha='center', color='grey')
        elif (j%2 == 1 and j//2 <= len(arrows)-2):
            start_point = arrows[j//2][1]  # 現在の矢印の終了点
            end_point = arrows[j//2 + 1][0]  # 次の矢印の開始点
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
    ax.set_title('4x3 Grid Animation')
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')


 

# セルの座標リスト
cut_indices = [(1, 0), (1, 2), (2, 0), (2, 1), (2, 2),(3, 0),(3, 1)]
fill_indices = [(0, 0), (0, 1), (0, 2), (1, 1),(3, 2)]

# セルの土量
# cut_soil = [550, 800, 450, 6700, 2000]
# fill_soil = [1000, 4000, 5000, 500]
cut_soil = [1, 1, 1, 1, 1, 1, 1]
fill_soil = [1, 1, 1, 2, 2]
if (sum(cut_soil)!=sum(fill_soil)):
    print("input error:切土と盛土の土量が違います")
    exit() 
#土を運ぶ際、切土から盛土で往復する回数
num_transport = 5
#仮設道路のリスト(切土の番号、盛り土の番号)
temp_road = [(0,0)]

#0番目の切土から0番目の盛り土に仮設道路があることを表す
# コスト行列の作成
dist = np.array([[calculate_distance(c[0], c[1], f[0], f[1]) for f in fill_indices] for c in cut_indices])
costs = calculate_cost(dist,temp_road)
# 問題の設定
prob = pulp.LpProblem("土砂運搬最適化", pulp.LpMinimize)

# 変数の定義
T = sum(cut_soil)
num_fill = len(fill_indices)
num_cut = len(cut_indices)

# 変数xはステップtで切土cから盛土fに運んだ時、x[t][c][f]=1となる。それ以外は0
x_vars = pulp.LpVariable.dicts("x", (range(T), range(num_cut), range(num_fill)), cat='Binary')

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
          objective += num_transport* costs[c][f] * x_vars[t][c][f]
#盛土地点から次のステップの切土地点への移動コストを追加
for t in range(T-1):
    objective += costs[m[t+1]][n[t]]

prob += objective

# #仮設道路の建設コストを追加
# temp_cost_per_road = 5
# temp_cost = 0
# for (cut_idx,fill_idx) in temp_road:
#     temp_cost += temp_cost_per_road * calculate_distance(cut_indices[cut_idx][0],cut_indices[cut_idx][1],fill_indices[fill_idx][0],fill_indices[fill_idx][1])
# print(f"temp_cost:{temp_cost}")
# # prob += temp_cost


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
    prob += pulp.lpSum(x_vars[t][c][f] for t in range(T) for f in range(num_fill)) == cut_soil[c]

for f in range(num_fill):
    prob += pulp.lpSum(x_vars[t][c][f] for t in range(T) for c in range(num_cut)) == fill_soil[f]
# 問題の解決
prob.solve()

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

# 複数の矢印を描画するための例
arrows = []

# 解の表示
for t in range(T):
    print(f"\nステップ {t+1}:")
    for f in range(num_fill):
        for c in range(num_cut):
            if pulp.value(x_vars[t][c][f]) > 0.5:
                print(f"  切土地点 ({cut_indices[c]}) から 盛土地点 ({fill_indices[f]}) に土が運ばれました")
                arrows.append((cut_indices[c],fill_indices[f]))


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
for (i, j) in cut_indices:
    soil_amount[j, i] = 1
for (i, j) in fill_indices:
    soil_amount[j, i] = 0





# アニメーションの準備
fig, ax = plt.subplots(figsize=(8, 6))

# アニメーションの実行
ani = animation.FuncAnimation(fig, animate, frames=2*len(arrows), interval=1000, repeat=False)

# GIFや動画として保存したい場合
# ani.save('animation.gif', writer='imagemagick')

# アニメーションを表示
plt.show()