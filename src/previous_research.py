import numpy as np
import time
#問題設定
# グリッドサイズ
grid_size_x = 10
grid_size_y = 10
#仮説道路上を走る際のスピード効率(0~1の間)
temp_eff = 0.8
if (temp_eff<0 or temp_eff >=1):
    print("tempory road's efficiency must be between 0 and 1")
    exit() 

def get_decimal_part(value):
    return value - int(value)

def calculate_distance(i, j, k, l):
    return np.sqrt((i - k)**2 + (j - l)**2)

def line_points(x1, y1, x2, y2):
    """直線上の整数座標を返す"""
    points = []
    dx = x2 - x1
    dy = y2 - y1
    steps = max(abs(dx), abs(dy))

    for i in range(steps + 1):
        if (abs(dx)>abs(dy)):
            decimal_part = get_decimal_part(i*dy/steps)
            if (decimal_part!=0.5):
                x = round(x1 + i * dx / steps)
                y = round(y1 + i * dy / steps)
                points.append((x, y))
            else:
                x_1 = x1 + i * dx // steps
                y_1 = y1 + i * dy // steps
                y_2 = y1 + i * dx // steps+1
                points.append((x, y_1))
                points.append((x, y_2))  

        elif (abs(dy)>abs(dx)):
            decimal_part = get_decimal_part(i*dx/steps)
            if (decimal_part!=0.5):
                x = round(x1 + i * dx / steps)
                y = round(y1 + i * dy / steps)
                points.append((x, y))            
            else:
                x_1 = x1 + i * dx // steps
                x_2 = x1 + i * dx // steps+1
                y = y1 + i * dy // steps
                points.append((x_1, y))
                points.append((x_2, y)) 
        # print(points)
    return points

# 切土から盛土に仮設道路がある場合のリスト
temp_road = [((0, 0), (0, 1)),((1, 1),(2, 2))]  # (切土の始点, 切土の終点)
# 移動コストの初期化
# 各セル間の移動コストを無限大で初期化
cost = np.full((grid_size_x * grid_size_y, grid_size_x * grid_size_y), np.inf)

# セル間の基本コストを設定 (1ユニットとする)
for i in range(grid_size_x):
    for j in range(grid_size_y):
        index = i * grid_size_y + j
        cost[index][index] = 0  # 自分自身へのコストは0

        # 隣接するセルのコストを設定
        if i + 1 < grid_size_x:  # 右
            cost[index][(i + 1) * grid_size_y + j] = 1
        if j + 1 < grid_size_y:  # 下
            cost[index][i * grid_size_y + (j + 1)] = 1
        if i - 1 >= 0:  # 左
            cost[index][(i - 1) * grid_size_y + j] = 1
        if j - 1 >= 0:  # 上
            cost[index][i * grid_size_y + (j - 1)] = 1

        # 斜め方向のセルのコストを設定 (√2)
        if i + 1 < grid_size_x and j + 1 < grid_size_y:  # 右下
            cost[index][(i + 1) * grid_size_y + (j + 1)] = np.sqrt(2)
        if i + 1 < grid_size_x and j - 1 >= 0:  # 右上
            cost[index][(i + 1) * grid_size_y + (j - 1)] = np.sqrt(2)
        if i - 1 >= 0 and j + 1 < grid_size_y:  # 左下
            cost[index][(i - 1) * grid_size_y + (j + 1)] = np.sqrt(2)
        if i - 1 >= 0 and j - 1 >= 0:  # 左上
            cost[index][(i - 1) * grid_size_y + (j - 1)] = np.sqrt(2)
        

start_time = time.time()

# 仮設道路を考慮したコストを更新
# for i in range(grid_size_x):
#     for j in range(grid_size_y):
#         index = i * grid_size_y + j
#         for i2 in range(grid_size_x):
#             for j2 in range(grid_size_y):
#                 if (i, j) != (i2, j2):
#                     if in_temp_road(i, j, i2, j2):
#                         cost[index][i2 * grid_size_y + j2] = min(cost[index][i2 * grid_size_y + j2], 0.7)  # 仮設道路を使う場合のコスト
# print(cost)

#同一仮説道路上のセル間の距離コストを埋める
for temp in temp_road:
    index_i = temp[0][0] * grid_size_y+ temp[0][1]
    index_j = temp[1][0] * grid_size_y+ temp[1][1]
    # cost[index_i][index_j] = temp_eff *calculate_distance(temp[0][0],temp[0][1],temp[1][0],temp[1][1])
    cost[index_i][index_j] *= temp_eff 

# for i in range(grid_size_x):
#     for j in range(grid_size_y):
#         index = i * grid_size_y + j
#         print(f"Cell ({i}, {j}): ", cost[index])
# print(cost)


# Warshall-Floyd法を用いて全頂点間の最短コストを計算
for k in range(grid_size_x * grid_size_y):
    for i in range(grid_size_x * grid_size_y):
        for j in range(grid_size_x * grid_size_y):
            index_i_x = i //grid_size_x
            index_i_y = i % grid_size_x
            index_j_x = j //grid_size_x
            index_j_y = j % grid_size_x            
            cost[i][j] = min(cost[i][j], cost[i][k] + cost[k][j])

end_time = time.time()
np.set_printoptions(formatter={'float': '{:.2f}'.format})
# for i in range(grid_size_x):
#     for j in range(grid_size_y):
#         index = i * grid_size_y + j
#         print(f"Cell ({i}, {j}): ", cost[index])
print(cost)
print(f"Calculation Time: {end_time - start_time:.6f} seconds")