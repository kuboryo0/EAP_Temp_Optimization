import numpy as np
import copy
import matplotlib.pyplot as plt
import matplotlib.animation as animation

grid_size_x = 12
grid_size_y = 4
def plot_route(path_list,temp_solution,cut_indices,fill_indices,zero_indices):  #結果の可視化（経路）
    # 格子点の座標を生成
    x = [i for i in range(grid_size_x)]
    y = [i for i in range(grid_size_y)]
    X, Y = np.meshgrid(x, y)
    current_temp = []
    all_temp = []
    for temp in temp_solution:
        current_temp.append([[temp[0][0],0],[temp[0][1],0]])
        all_temp.append([[temp[0][0],1],[temp[0][1],1]])
    # プロット用の格子点を設定
    x_coords = X.flatten()
    y_coords = Y.flatten()

    # 土量マトリックスを作成（仮に色付けのためのデータを用意）
    soil_amount_color = np.zeros((grid_size_y, grid_size_x))
    for [(i, j),k] in cut_indices:
        soil_amount_color[int(j), int(i)] = 1
    # for [(i, j),k] in fill_indices:
    #     soil_amount_color[int(j),int(i)] = 0
    for [(i, j),k] in zero_indices:
        soil_amount_color[int(j),int(i)] = 0.5
    soil_amount_real = np.zeros((grid_size_y, grid_size_x))
    for [(i, j),k] in cut_indices:
        soil_amount_real[int(j), int(i)] = k
    for [(i, j),k] in fill_indices:
        soil_amount_real[int(j),int(i)] = -k
    # for [(i, j),k] in zero_indices:
    #     soil_amount_real[int(j),int(i)] = 0

    def init_animate (): #初期化関数(これがないとanimate関数のi=0が2回繰り返される)
        pass

    def temp_before_step(current_temps,step,solution):
        current_temps_copy = copy.deepcopy(current_temps)
        new_temp = []
        # print("current_temps_copy before ",current_temps_copy)
        # for temp in solution:
        #     new_temp.append([[temp[0][0],0],[temp[0][1],0]])
        for i,(pair,timing) in enumerate(solution):
            new_temp_i_binary_0 = 0
            new_temp_i_binary_1 = 0
            if timing[step] == 1:
                if current_temps_copy[i][0][1] == 0:
                    new_temp_i_binary_0 = 1
                    current_temps_copy[i][0][1] = 1
                if current_temps_copy[i][1][1] == 0:
                    new_temp_i_binary_1 = 1
                    current_temps_copy[i][1][1] = 1
            new_temp.append([[pair[0],new_temp_i_binary_0],[pair[1],new_temp_i_binary_1]])
        # print("new_temp",new_temp)  
        # print("current_temps_copy after",current_temps_copy,"\n")
        return current_temps_copy,new_temp

    def draw_temporary_roads_in_step(ax, current_road, start_point, end_point, new_temp): 
        # 仮設道路の描画
        current_road_copy = copy.deepcopy(current_road)
        
        if current_road_copy is not None:
            if start_point is not None and end_point is not None:
                for i, temp in enumerate(current_road_copy):
                    current_list = temp[0]
                    next_list = temp[1]
                    current = current_list[0]
                    next = next_list[0]
                    if current == start_point or current == end_point:
                        current_road_copy[i][0][1] = 0
                    if next == start_point or next == end_point:
                        current_road_copy[i][1][1] = 0
            
            for i, temp in enumerate(current_road_copy):
                current_list = temp[0]
                next_list = temp[1]
                current = current_list[0]
                next = next_list[0]
                adjusted_current = (current[0] + 0.5, current[1] + 0.5)
                adjusted_next = (next[0] + 0.5, next[1] + 0.5)
                adjusted_middle = ((current[0] + next[0]) / 2 + 0.5, (current[1] + next[1]) / 2 + 0.5)
                # print("new_temp",new_temp)
                # Check if the road is new
                color_current = "grey"
                color_next = "grey"
                if new_temp is not None:
                    if new_temp[i][0][1] == 1:
                        color_current = "blue"
                    if new_temp[i][1][1] == 1:
                        color_next = "blue"

                if current_list[1] == 1:
                    ax.plot(
                        [adjusted_current[0], adjusted_middle[0]],
                        [adjusted_current[1], adjusted_middle[1]],
                        color=color_current,
                        linewidth=10,
                        alpha=0.7,
                    )
                if next_list[1] == 1:
                    ax.plot(
                        [adjusted_middle[0], adjusted_next[0]],
                        [adjusted_middle[1], adjusted_next[1]],
                        color=color_next,
                        linewidth=10,
                        alpha=0.7,
                    )

        return current_road_copy
    def animate(i):
        nonlocal current_temp_copy
        nonlocal soil_amount_real_copy
        nonlocal soil_amount_color_copy

        ax.clear()
        # 最終フレーム：全てを描画
        if i >= len(path_list) * 2:
            draw_temporary_roads_in_step(ax, all_temp, None, None, None)  # 仮設道路を描画
            # path_list内のすべての矢印を描画
            for step, (arrows_i,_) in enumerate(path_list):
                for (start_point, end_point) in zip(arrows_i[:-1], arrows_i[1:]):
                    adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                    adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                    ax.annotate(
                        "",
                        xy=adjusted_end,
                        xytext=adjusted_start,
                        arrowprops=dict(facecolor="red", edgecolor="black", linewidth=2, alpha=0.7, shrink=0.05),
                    )
                    # 矢印の番号を矢印の中心に表示
                    mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                    ax.text(mid_point[0], mid_point[1] + 0.1, f'({step + 1})', fontsize=12, ha='center', color='grey')
        # 偶数フレーム：仮設道路のみを描画
        elif i % 2 == 0:
            current_temp_copy,new_temp = temp_before_step(current_temp_copy, i // 2, temp_solution) # 仮設道路の更新
            draw_temporary_roads_in_step(ax, current_temp_copy, None, None,new_temp)  # 仮設道路を描画
        
        # 奇数フレーム：仮設道路 + 矢印 + 土量の描画
        else:
            step = i // 2
            if step < len(path_list):
                arrows_i = path_list[step][0]
                start_point = arrows_i[0]
                end_point = arrows_i[-1]
                # 仮設道路の描画
                current_temp_copy = draw_temporary_roads_in_step(ax, current_temp_copy, start_point, end_point,None)
                # 土量の変更
                soil_amount_real_copy[int(start_point[1]), int(start_point[0])] -= path_list[step][1]
                soil_amount_real_copy[int(end_point[1]), int(end_point[0])] += path_list[step][1]
                if soil_amount_real_copy[int(start_point[1]), int(start_point[0])] == 0:
                    soil_amount_color_copy[int(start_point[1]), int(start_point[0])] = 0.5
                if soil_amount_real_copy[int(end_point[1]), int(end_point[0])] == 0:
                    soil_amount_color_copy[int(end_point[1]), int(end_point[0])] = 0.5
                
                # 矢印を描画
                for start_point, end_point in zip(arrows_i[:-1], arrows_i[1:]):
                    adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                    adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                    ax.annotate(
                        "",
                        xy=adjusted_end,
                        xytext=adjusted_start,
                        arrowprops=dict(facecolor="red", edgecolor="black", linewidth=2, alpha=0.7, shrink=0.05),
                    )
                    # 矢印の番号を矢印の中心に表示
                    mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                    ax.text(mid_point[0], mid_point[1] + 0.1, f'({step + 1})', fontsize=12, ha='center', color='grey')
        # 共通の描画処理
        ax.pcolormesh(soil_amount_color_copy, edgecolors='gray', linewidth=2, cmap='viridis', shading='flat', alpha=0.2)
        ax.scatter(x_coords + 0.5, y_coords + 0.5, color='blue', marker='o')  # 格子点のプロット
        # 格子点のラベルを表示
        for x_val, y_val in zip(x_coords, y_coords):
            ax.text(x_val + 0.5, y_val + 0.4, f'{int(soil_amount_real_copy[y_val][x_val])}', fontsize=12, ha='center', va='top')

    # アニメーションの準備
    fig, ax = plt.subplots(figsize=(8, 6))
    # アニメーションの実行
    ani = animation.FuncAnimation(fig, animate,init_func=init_animate, frames=2*len(path_list)+1 , interval=500, repeat=False,blit=False)
    current_temp_copy = copy.deepcopy(current_temp)
    soil_amount_real_copy = copy.deepcopy(soil_amount_real)
    soil_amount_color_copy = copy.deepcopy(soil_amount_color)
    # GIFや動画として保存したい場合
    ani.save('animation_all.gif', writer='Pillow')
    current_temp_copy = copy.deepcopy(current_temp)
    soil_amount_real_copy = copy.deepcopy(soil_amount_real)
    soil_amount_color_copy = copy.deepcopy(soil_amount_color)
    # アニメーションを表示
    plt.show()

cut_indices = [
    [(2, 0), 3700], [(3, 0), 9000], [(4, 0), 9000], [(5, 0), 8000], [(11, 0), 11200],
    [(1, 1), 22500], [(2, 1), 33800], [(3, 1), 36000], [(4, 1), 23000], [(5, 1), 22200], [(6, 1), 8100], [(10, 1), 14300], [(11, 1), 7900],
    [(1, 2), 22500], [(2, 2), 28100], [(3, 2), 23000], [(4, 2), 24300], [(5, 2), 14200],
    [(2, 3), 2300], [(3, 3), 1200], [(4, 3), 9000], [(8, 3), 2300]
]

# 盛土の座標と土量
fill_indices = [
    [(0, 0), 15000], [(1, 0), 3700], [(6, 0), 1000], [(7, 0), 11200], [(8, 0), 2300], [(9, 0), 22000], [(10, 0), 6900],
    [(0, 1), 62600], [(7, 1), 24800], [(8, 1), 9900], [(9, 1), 2200],
    [(0, 2), 3700], [(6, 2), 12400], [(7, 2), 34400], [(8, 2), 72500], [(9, 2), 28500], [(10, 2), 2500],
    [(1, 3), 1400], [(5, 3), 5900], [(6, 3), 9900], [(7, 3), 2700], [(9, 3), 100]
]
zero_indices = [
    [(11, 2), 0],
    [(0, 3), 0], [(10, 3), 0], [(11, 3), 0]
]

best_solution =  [[[(5, 1), (6, 1)], [0, 0, 0, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0]], [[(7, 1), (8, 2)], [0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0]]]

best_route =  [[[(11, 1), (10, 2)], 2500.0], [[(11, 1), (10, 2), (9, 2)], 5400.0], [[(11, 0), (10, 0)], 6900.0], [[(11, 0), (10, 0), (9, 0)], 4300.0], [[(10, 1), (9, 2)], 12100.0], [[(10, 1), (9, 1)], 2200.0], [[(8, 3), (9, 3)], 100.0], [[(8, 3), (9, 2)], 2200.0], [[(3, 1), (4, 1), (5, 1), (6, 1), (7, 1), (8, 2), (9, 2)], 8800.0], [[(5, 0), (6, 0), (7, 0), (8, 0), (9, 0)], 8000.0], [[(4, 0), (5, 0), (6, 0), (7, 0), (8, 0), (9, 0)], 4000.0], [[(3, 0), (4, 0), (5, 0), (6, 0), (7, 0), (8, 0), (9, 0)], 5700.0], [[(5, 2), (6, 2), (7, 2), (8, 2)], 14200.0], [[(4, 2), (5, 2), (6, 2), (7, 2), (8, 2)], 24300.0], [[(4, 1), (5, 1), (6, 1), (7, 1), (8, 2)], 17400.0], [[(3, 1), (4, 1), (5, 1), (6, 1), (7, 1), (8, 2)], 16600.0], [[(6, 1), (7, 1), (8, 1)], 8100.0], [[(3, 1), (4, 1), (5, 1), (6, 1), (7, 1), (8, 1)], 1800.0], [[(3, 0), (4, 0), (5, 0), (6, 0), (7, 0), (8, 0)], 2300.0], [[(2, 2), (3, 2), (4, 3), (5, 3), (6, 3), (7, 3)], 2700.0], [[(4, 1), (5, 1), (6, 1), (7, 2)], 5600.0], [[(3, 2), (4, 2), (5, 2), (6, 2), (7, 2)], 22100.0], [[(2, 2), (3, 2), (4, 2), (5, 2), (6, 2), (7, 2)], 6700.0], [[(5, 1), (6, 1), (7, 1)], 22200.0], [[(3, 1), (4, 1), (5, 1), (6, 1), (7, 1)], 2600.0], [[(4, 0), (5, 0), (6, 0), (7, 0)], 5000.0], [[(3, 1), (4, 1), (5, 1), (6, 1), (7, 0)], 6200.0], [[(4, 3), (5, 3), (6, 3)], 9000.0], [[(3, 2), (4, 2), (5, 2), (6, 3)], 900.0], [[(2, 2), (3, 2), (4, 2), (5, 2), (6, 2)], 12400.0], [[(3, 0), (4, 0), (5, 0), (6, 0)], 1000.0], [[(3, 3), (4, 3), (5, 3)], 1200.0], [[(2, 3), (3, 3), (4, 3), (5, 3)], 900.0], [[(2, 2), (3, 3), (4, 3), (5, 3)], 3800.0], [[(2, 3), (1, 3)], 1400.0], [[(2, 2), (1, 2), (0, 1)], 2500.0], [[(1, 2), (0, 2)], 3700.0], [[(2, 1), (1, 0)], 3700.0], [[(2, 1), (1, 1), (0, 1)], 30100.0], [[(1, 2), (0, 1)], 18800.0], [[(2, 0), (1, 0), (0, 0)], 3700.0], [[(1, 1), (0, 1)], 11200.0], [[(1, 1), (0, 0)], 11300.0]]

plot_route(best_route,best_solution,cut_indices,fill_indices,zero_indices)