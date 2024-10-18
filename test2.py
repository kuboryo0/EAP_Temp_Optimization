import numpy as np
import time
# グリッドサイズ
grid_size_x = 3
grid_size_y = 3

def get_decimal_part(value):
    return value - int(value)

def calculate_distance(i, j, k, l):
    return np.sqrt((i - k)**2 + (j - l)**2)

# 仮設道路のコストを設定
def in_temp_road(x1, y1, x2, y2):
    for (start, end) in temp_road:
        (x_start, y_start) = start
        (x_end, y_end) = end
        # 仮設道路の直線に含まれるかチェック
        if (x1, y1) in line_points(x_start, y_start, x_end, y_end) and (x2, y2) in line_points(x_start, y_start, x_end, y_end):
            return True
    return False

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
temp_road = [((0, 0), (1, 2))]  # (切土の始点, 切土の終点)
temp_list = []
for ((x1,y1),(x2,y2)) in temp_road:
    # print (f"x1:{x1},y1:{y1},x2:{x2},y2:{y2}")
    temp_list.append(line_points(x1,y1,x2,y2))
print(temp_list)
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
        if i - 1 >= 0:  # 上
            cost[index][(i - 1) * grid_size_y + j] = 1
        if j - 1 >= 0:  # 左
            cost[index][i * grid_size_y + (j - 1)] = 1


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

#同一仮説道路上のセル間の距離を埋める
for temp in temp_list:
    for i in range(len(temp)):
        index_i = temp[i][0] * grid_size_y+ temp[i][1]
        for j in range(len(temp)):
            index_j = temp[j][0] * grid_size_y+ temp[j][1]
            if(i!=j):
                cost[index_i][index_j] = 0.7*calculate_distance(temp[i][0],temp[i][1],temp[j][0],temp[j][1])

for i in range(grid_size_x):
    for j in range(grid_size_y):
        index = i * grid_size_y + j
        print(f"Cell ({i}, {j}): ", cost[index])
# print(cost)
# Warshall-Floyd法を用いて全頂点間の最短コストを計算
#要修正
for k in range(grid_size_x * grid_size_y):
    for i in range(grid_size_x * grid_size_y):
        for j in range(grid_size_x * grid_size_y):
            cost[i][j] = min(cost[i][j], cost[i][k] + cost[k][j])

end_time = time.time()
# 結果を表示
for i in range(grid_size_x):
    for j in range(grid_size_y):
        index = i * grid_size_y + j
        print(f"Cell ({i}, {j}): ", cost[index])
# 計算時間を表示  
print(f"Calculation Time: {end_time - start_time:.6f} seconds")