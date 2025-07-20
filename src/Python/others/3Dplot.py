import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# グリッドサイズの指定
x_size, y_size = 5, 5

# xとyの位置を設定
x = np.arange(x_size)
y = np.arange(y_size)
x, y = np.meshgrid(x, y)

min_height = -5  # 最小高さ
max_height = 5   # 最大高さ
# 各グリッドごとの高さ z を指定（ランダムに-10から10の範囲の整数値）
z = np.random.randint(min_height, max_height + 1, size=(x_size, y_size))

# プロットの設定
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 各グリッドのブロックを描画
# min_
for i in range(x_size):
    for j in range(y_size):
        # 各ブロックの高さによる色分け
        if z[i, j] > 0:
            color = 'blue'   # プラスの高さは青
        elif z[i, j] < 0:
            color = 'red'    # マイナスの高さは赤
        else:
            color = 'gray'   # 高さゼロは灰色
        
        # 各ブロックを bar3d で描画
        ax.bar3d(x[i, j], y[i, j], 0, 1, 1, z[i, j]-min_height, color=color, shade=True)

# 軸ラベル
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
