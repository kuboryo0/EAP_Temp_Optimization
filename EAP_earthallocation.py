#直線距離をコスト行列とした土砂配分計画問題を解くプログラム
import numpy as np
import random
import pulp
import time
import math
import matplotlib.pyplot as plt
from function_ver2 import earth_allocation
from function_ver2 import plot_route

temp_eff = 0.7
v =  1/temp_eff
tan_alpha = math.sqrt(v**2-1)
sin_alpha = math.sqrt(v**2-1)/v
distance = 0


# 仮設道路のデザイン
temporary_roads = {
}

print("temporary_roads",temporary_roads)
#切土の座標と土量
cut_indices = [[(1, 0),1],[(1, 2),1],[(2, 0),1],[(2, 1),1],[(2, 2),1],[(2, 3),1],[(3, 0),1],[(3, 1),1],[(3, 3),2]]
#盛土の座標と土量
fill_indices = [[(0, 0),2],[(0, 1),1],[(0, 2),1],[(0, 3),1],[(1, 3),2],[(1, 1),1],[(3, 2),2]]

cut_indices_float = []
for i in range(len(cut_indices)):
    new_coords = (float(cut_indices[i][0][0]), float(cut_indices[i][0][1]))  # 新しいタプルを作成
    cut_indices_float.append([new_coords, cut_indices[i][1]])  # 新しいリストに追加

fill_indices_float = []
for i in range(len(fill_indices)):
    new_coords = (float(fill_indices[i][0][0]), float(fill_indices[i][0][1]))  # 新しいタプルを作成
    fill_indices_float.append([new_coords, fill_indices[i][1]])  # 新しいリストに追加

sum_cut = sum(cut_indices[i][1] for i in range(len(cut_indices)))
sum_fill = sum(fill_indices[i][1] for i in range(len(fill_indices)))
# 土量の合計が一致しているか確認
if (sum_cut != sum_fill):
    print("input error:切土と盛土の土量が違います")
    exit() 



start_time = time.time()

#土砂の分配を計画
routes = earth_allocation(cut_indices_float, fill_indices_float)

# # 初期解の評価
# path_list,cost = a_star(routes,temporary_roads,4)

# print("初期解",path_list)
# print("初期解のコスト",cost)
grid_size_x = 4
grid_size_y = 4
print("routes",routes)
plot_route(grid_size_x,grid_size_y,routes,temporary_roads, cut_indices, fill_indices)


