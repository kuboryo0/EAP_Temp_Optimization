import pulp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
#土量を表示するためのコード

# セルの土量設定
# 土量 = np.array([-1000, -4000, -5000, 550, -500, 800, 450, 6700, 2000]).reshape((3, 3))

# 距離コストの計算
def calculate_distance(i, j, k, l):
    return np.sqrt((i - k)**2 + (j - l)**2)



#切土の座標と土量
cut_indices = [[(1, 0),1],[(1, 2),1],[(2, 0),1],[(2, 1),1],[(2, 2),1],[(2, 3),1],[(3, 0),1],[(3, 1),1],[(3, 3),2]]
#盛土の座標と土量
fill_indices = [[(0, 0),2],[(0, 1),1],[(0, 2),1],[(0, 3),1],[(1, 3),2],[(1, 1),1],[(3, 2),2]]

#結果の可視化

# グリッドのサイズ
grid_size_x = 4  # x方向のサイズ
grid_size_y = 4  # y方向のサイズ

# 格子点の座標を生成
x = [i for i in range(grid_size_x)]
y = [i for i in range(grid_size_y)]
X, Y = np.meshgrid(x, y)

# プロット用の格子点を設定
x_coords = X.flatten()
y_coords = Y.flatten()

# 土量マトリックスを作成（仮に色付けのためのデータを用意）
soil_amount = np.zeros((4, 4))
for [(i, j),k] in cut_indices:
    soil_amount[j, i] = 1
for [(i, j),k] in fill_indices:
    soil_amount[j, i] = 0


temp = []

# プロットの準備
plt.figure(figsize=(8, 6))  # サイズを変更

plt.pcolormesh(soil_amount, edgecolors='gray', linewidth=2, cmap='viridis', shading='flat',alpha=0.4)
plt.scatter(x_coords+0.5, y_coords+0.5, color='blue', marker='o')  # 格子点のプロット

for k in range(len(temp)):
        start_point, end_point = temp[k]
        adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
        adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
        plt.plot([adjusted_start[0], adjusted_end[0]], [adjusted_start[1], adjusted_end[1]], 
                    color='black', linewidth=5, alpha=0.5)

# # 格子点のラベルを表示（x, y方向に0.5ずらす）
# for x_val, y_val in zip(x_coords, y_coords):
#     plt.text(x_val + 0.5, y_val + 0.4, f'({x_val},{y_val})', fontsize=12, ha='center', va='top')

# 切土地点の座標に切土量をラベルとして表示
for [(i, j),k] in cut_indices:
    plt.text(i + 0.5, j + 0.5, f'Cut: {k}', fontsize=12, ha='center', color='black')

# 盛土地点の座標に盛土量をラベルとして表示
for [(i, j),k] in fill_indices:
    plt.text(i + 0.5, j + 0.5, f'Fill: {k}', fontsize=12, ha='center', color='black')



# グリッド線の描画
for i in np.arange(-1.0, grid_size_y+1, 1.0):
    plt.axhline(y=i, color='gray', linestyle='--', linewidth=0.5)  # 横方向のグリッド線
for i in np.arange(-1.0, grid_size_x+1, 1.0):
    plt.axvline(x=i, color='gray', linestyle='--', linewidth=0.5)  # 縦方向のグリッド線

# プロットのカスタマイズ
plt.xlim(-0.5, grid_size_x + 0.5)
plt.ylim(-0.5, grid_size_y + 0.5)
plt.gca().set_aspect('equal', adjustable='box')
plt.title(f'{grid_size_x}x{grid_size_y} Grid with Custom Gridlines')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')

# プロットの表示
plt.show()

