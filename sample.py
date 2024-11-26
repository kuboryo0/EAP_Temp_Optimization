import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import copy

# 初期データ
grid_size_x = 4
grid_size_y = 4
path_list = [
    [(3.0, 0.0), (2.0, 0.0), (1.0, 0.0), (0.0, 0.0), (0.0, 1.0)],
    [(3.0, 3.0), (2.0, 3.0), (1.0, 3.0)],
    [(2.0, 0.0), (1.0, 0.0), (0.0, 0.0)],
]
temporary_roads = [{(1, 3): {6}, (2, 3): {1}}, {(0, 3): {5}, (1, 2): {2, 6}}]
cut_indices = [[(1, 0), 1], [(1, 2), 1], [(2, 0), 1]]
fill_indices = [[(0, 0), 2], [(0, 1), 1], [(1, 3), 2]]

def plot_3d_routes(grid_size_x, grid_size_y, path_list, temporary_roads, cut_indices, fill_indices):
    DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    # 土量の初期設定
    soil_amount = np.zeros((grid_size_y, grid_size_x))
    for [(i, j), k] in cut_indices:
        soil_amount[int(j), int(i)] = k
    for [(i, j), k] in fill_indices:
        soil_amount[int(j), int(i)] = -k

    soil_amount_real_copy = soil_amount.copy()

    # アニメーションの初期化
    def init_animate():
        pass

    # アニメーション関数
    def animate(i):
        ax.clear()
        ax.set_xlim(0, grid_size_x)
        ax.set_ylim(0, grid_size_y)
        ax.set_zlim(-3, 3)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        if i < len(path_list):
            arrows = path_list[i]
            start = arrows[0]
            end = arrows[-1]

            # 更新土量
            soil_amount_real_copy[int(start[1]), int(start[0])] -= 1
            soil_amount_real_copy[int(end[1]), int(end[0])] += 1

            # ブロックの描画
            for x in range(grid_size_x):
                for y in range(grid_size_y):
                    h = soil_amount_real_copy[y, x]
                    if h > 0:
                        ax.bar3d(x, y, 0, 1, 1, h, color="brown", alpha=0.8)
                    elif h < 0:
                        ax.bar3d(x, y, h, 1, 1, -h, color="blue", alpha=0.8)

            # 仮設道路の描画
            for road in temporary_roads:
                for start, directions in road.items():
                    for direction in directions:
                        dx, dy = DIRECTIONS[direction]
                        ax.plot(
                            [start[0] + 0.5, start[0] + 0.5 + dx],
                            [start[1] + 0.5, start[1] + 0.5 + dy],
                            [
                                soil_amount_real_copy[start[1], start[0]] + 0.5,
                                soil_amount_real_copy[start[1] + dy, start[0] + dx] + 0.5,
                            ],
                            color="grey",
                            linewidth=2,
                            alpha=0.7,
                        )

            # 矢印の描画
            for k, (start_point, end_point) in enumerate(zip(arrows[:-1], arrows[1:])):
                start_z = soil_amount_real_copy[int(start_point[1]), int(start_point[0])]
                end_z = soil_amount_real_copy[int(end_point[1]), int(end_point[0])]
                ax.quiver(
                    start_point[0] + 0.5,
                    start_point[1] + 0.5,
                    start_z + 0.5,
                    end_point[0] - start_point[0],
                    end_point[1] - start_point[1],
                    end_z - start_z,
                    color="red",
                    arrow_length_ratio=0.3,
                )
        else:
            # 最後の状態を描画
            for x in range(grid_size_x):
                for y in range(grid_size_y):
                    h = soil_amount_real_copy[y, x]
                    if h > 0:
                        ax.bar3d(x, y, 0, 1, 1, h, color="brown", alpha=0.8)
                    elif h < 0:
                        ax.bar3d(x, y, h, 1, 1, -h, color="blue", alpha=0.8)

    # アニメーション作成
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ani = animation.FuncAnimation(
        fig, animate, init_func=init_animate, frames=len(path_list) + 1, interval=1500
    )

    # GIF保存（必要に応じて）
    ani.save("3d_animation.gif", writer="Pillow")

    # アニメーション表示
    plt.show()


plot_3d_routes(grid_size_x, grid_size_y, path_list, temporary_roads, cut_indices, fill_indices)
