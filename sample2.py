import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import copy

# 初期データ
grid_size_x = 3
grid_size_y = 3
path_list = [
    [(0.0, 0.0), (0.0, 1.0),],

]
temporary_roads = [{(1, 2): {6}, (2, 2): {1}}, {(0, 2): {5}, (1, 1): {2, 6}}]
cut_indices = [[(0, 2), 2],[(1, 0), 1], [(1, 2), 1], [(2, 0), 1]]
fill_indices = [[(0, 0), 1], [(0, 1), 1],[(1, 1), 1],[(2, 1), 1], [(2, 2), 1]]

def plot_3d_routes(grid_size_x, grid_size_y, path_list, temporary_roads, cut_indices, fill_indices):
    DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    # 土量の初期設定
    soil_amount = np.zeros((grid_size_x, grid_size_y))
    for [(i, j), k] in cut_indices:
        soil_amount[int(i), int(j)] = k
    for [(i, j), k] in fill_indices:
        soil_amount[int(i), int(j)] = -k

    # soil_amount_real_copy = copy.deepcopy(soil_amount)

    # アニメーションの初期化
    def init_animate():
        pass

    # アニメーション関数
    def animate(i):
        print("i",i)
        ax.clear()
        ax.set_xlim(0, grid_size_x)
        ax.set_ylim(0, grid_size_y)
        ax.set_zlim(-3, 3)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        # if i < len(path_list):
        arrows = path_list[i]
        start = arrows[0]
        end = arrows[-1]
        soil_amount_real_copy = copy.deepcopy(soil_amount)
        # 更新土量
        soil_amount_real_copy[int(start[0]), int(start[1])] -= 1
        soil_amount_real_copy[int(end[0]), int(end[1])] += 1
        print("soil_amount_real_copy",soil_amount_real_copy)
        # ブロックの描画
        for x in range(grid_size_x):
            for y in range(grid_size_y):
                h = soil_amount_real_copy[x, y]
                if h > 0:
                    ax.bar3d(x, y, 0, 1, 1, h, color="blue", alpha=0.3)
                elif h < 0:
                    ax.bar3d(x, y, h, 1, 1, -h, color="red", alpha=0.3)
        # 仮設道路の描画
        for road in temporary_roads:
            for start, directions in road.items():
                for direction in directions:
                    # dx = DIRECTIONS[direction][0]/2
                    # dy = DIRECTIONS[direction][1]/2
                    dx, dy = DIRECTIONS[direction]
                    print("start",start)
                    # print("dx",dx)
                    # print("dy",dy)
                    print("DIRECTIONS[direction]",DIRECTIONS[direction])
                    middle_soil = (soil_amount_real_copy[int(start[0]), int(start[1])] +soil_amount_real_copy[int(start[0] + dx), int(start[1] + dy)])/2
                    print("middle_point",(start[0] + dx/2,start[1] + dy/2))
                    print("middle_soil",middle_soil)
                    if soil_amount_real_copy[start[0], start[1]] > 0:
                        ax.plot(
                            [start[0] + 0.5, start[0] + 0.5 + dx/2],
                            [start[1] + 0.5, start[1] + 0.5 + dy/2],
                            [
                                soil_amount_real_copy[start[0], start[1]] - 0.5,
                                middle_soil,
                            ],
                            color="grey",
                            linewidth=7,
                            alpha=0.9,
                        )
                    elif soil_amount_real_copy[start[0], start[1]] <0:
                        ax.plot(
                            [start[0] + 0.5, start[0] + 0.5 + dx/2],
                            [start[1] + 0.5, start[1] + 0.5 + dy/2],
                            [
                                soil_amount_real_copy[start[0], start[1]] + 0.5,
                                middle_soil,
                            ],
                            color="grey",
                            linewidth=7,
                            alpha=0.9,
                        )
                    else:
                        ax.plot(
                            [start[0] + 0.5, start[0] + 0.5 + dx/2],
                            [start[1] + 0.5, start[1] + 0.5 + dy/2],
                            [
                                soil_amount_real_copy[start[0], start[1]],
                                middle_soil,
                            ],
                            color="grey",
                            linewidth=7,
                            alpha=0.9,
                        )
                    print("\n")
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ani = animation.FuncAnimation(
        fig, animate, init_func=init_animate, frames=len(path_list) , interval=1500,repeat=False
    )
    # GIF保存（必要に応じて）
    ani.save("3d_animation.gif", writer="Pillow")
    print("after save")
    # アニメーション表示
    plt.show()
    print("after show")


plot_3d_routes(grid_size_x, grid_size_y, path_list, temporary_roads, cut_indices, fill_indices)