import pulp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import time
import copy
# セルの土量設定
# 土量 = np.array([-1000, -4000, -5000, 550, -500, 800, 450, 6700, 2000]).reshape((3, 3))
temp_eff = 0.7
v =  1/temp_eff
tan_alpha = math.sqrt(v**2-1)
sin_alpha = math.sqrt(v**2-1)/v
distance = 0
temp = [[(0, 0), (0, 1)],[(1, 1),(2, 2)],[(1, 2),(2, 1)]]



def calculate_cost(cut_indices, fill_indices,temp):

    def on_segment(p:tuple, q:tuple, r:tuple) -> bool:
            """点rが線分pq上にあるかどうかを確認する関数"""
            if min(p[0], q[0]) < r[0] < max(p[0], q[0]) and min(p[1], q[1]) < r[1] < max(p[1], q[1]):
                return True
            return False


    def is_intersect(p1:float, p2:float, q1:float, q2:float) -> bool:
        def orientation(p:tuple, q:tuple, r:tuple) -> int:
            """3点の並び順を計算。0: コリニア（同一直線上）, 1: 時計回り, 2: 反時計回り"""
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if val == 0:
                return 0  # コリニア
            elif val > 0:
                return 1  # 時計回り
            else:
                return 2  # 反時計回り

        

        # 4つの点 p1, p2, q1, q2 の並びを取得
        o1 = orientation(p1, p2, q1)
        o2 = orientation(p1, p2, q2)
        o3 = orientation(q1, q2, p1)
        o4 = orientation(q1, q2, p2)

        # 交差判定
        # 線分が交差する一般的な場合
        if o1 != o2 and o3 != o4:
            return True

        # 特殊なケース：線分がコリニアで、交差する場合
        if o1 == 0 and on_segment(p1, p2, q1):
            return True
        if o2 == 0 and on_segment(p1, p2, q2):
            return True
        if o3 == 0 and on_segment(q1, q2, p1):
            return True
        if o4 == 0 and on_segment(q1, q2, p2):
            return True

        return False

    def calculate_distance(i:float, j:float, k:float, l:float)->float:
        return np.sqrt((i - k)**2 + (j - l)**2)

    def calculate_intersection(p1:float, p2:float, q1:float, q2:float) -> tuple:
        """p1, p2: 1つ目の仮設道路, q1, q2: 2つ目の仮設道路の端点"""
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = q1
        x4, y4 = q2

        # 行列式を使って交点を計算
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denominator == 0:
            print("denominator == 0")
            return None  # 交差しない（平行または同一線上）

        intersect_x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator
        intersect_y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator

        return (intersect_x, intersect_y)


    #ある点(x,y)から仮設道路(temp_i)に角度alphaで下した点の座標を求める
    #また、その点から仮設道路までの距離を返す
    def shortest_point_to_road(x:float,y:float,temp_i:list):
        if not (isinstance(temp_i, list) and len(temp_i) == 2 and
            all(isinstance(point, tuple) and len(point) == 2 and 
                all(isinstance(coord, (int, float)) for coord in point) 
                for point in temp_i)):
            print("temp_i",temp_i)
            print("shortest_point_to_road error:temp_iは[(p1,q1),(p2,q2)]の型で入力してください")
            return None
        point = []
        if temp_i[0][0] == temp_i[1][0]: #仮設道路が垂直の場合
            a = temp_i[0][0]
            d = abs(x-a)
            if(x != a):
                point1_x = a
                point1_y = y + d/tan_alpha
                point2_x = a
                point2_y = y - d/tan_alpha
                point.append((point1_x,point1_y))
                point.append((point2_x,point2_y))
                distance = d/sin_alpha
            else:
                point.append((x,y))
                distance = 0
        else:
            m = (temp_i[1][1]-temp_i[0][1])/(temp_i[1][0]-temp_i[0][0]) 
            n = (temp_i[0][1]*temp_i[1][0]-temp_i[0][0]*temp_i[1][1])/(temp_i[1][0]-temp_i[0][0])
            beta = math.atan(m)
            d = abs(m*x-y+n)/math.sqrt(m**2+1) 
            #(x,y)から仮設道路に角度alphaで下した2つの点(point1とpoint2)の座標を求める
            if (y > m*x+n): #(x,y)が仮設道路(y = mx+n)より上にある場合
                point1_x = x + d*(math.cos(beta)/tan_alpha+math.sin(beta))
                point1_y = y - d*(math.cos(beta)-math.sin(beta)/tan_alpha)
                point2_x = x - d*(math.cos(beta)/tan_alpha-math.sin(beta))
                point2_y = y - d*(math.cos(beta)+math.sin(beta)/tan_alpha)
                point.append((point1_x,point1_y))
                point.append((point2_x,point2_y))
                distance = calculate_distance(x,y,point1_x,point1_y)
            elif (y < m*x+n): #(x,y)が仮設道路(y = mx+n)より下にある場合
                point1_x = x + d*(math.cos(beta)/tan_alpha-math.sin(beta))
                point1_y = y + d*(math.cos(beta)+math.sin(beta)/tan_alpha)
                point2_x = x - d*(math.cos(beta)/tan_alpha+math.sin(beta))
                point2_y = y + d*(math.cos(beta)-math.sin(beta)/tan_alpha)
                point.append((point1_x,point1_y))
                point.append((point2_x,point2_y))
                distance = calculate_distance(x,y,point1_x,point1_y)
            else: # (x,y)が仮設道路上にある場合
                point.append((x,y))
                distance = 0
        #pointに含まれている座標が仮設道路上にあるかどうかを確認
        for i in range(len(point)):
            if not(on_segment(temp_i[0],temp_i[1],point[i])):
                point[i] = None
        return point,distance

    # 重複を取り除く関数
    def remove_duplicates(temp_list):
        unique_points = set()  # ユニークな座標を保持するセット
        cleaned_temp_list = []

        for sublist in temp_list:
            cleaned_sublist = []
            for item in sublist:
                if isinstance(item, tuple):  # 座標がタプルの場合
                    if item not in unique_points:
                        unique_points.add(item)
                        cleaned_sublist.append(item)
                elif isinstance(item, list):  # 内部リストの場合
                    unique_inner_points = []
                    for inner_item in item:
                        if isinstance(inner_item, tuple) and inner_item not in unique_points:
                            unique_points.add(inner_item)
                            unique_inner_points.append(inner_item)
                    cleaned_sublist.append(unique_inner_points)  # ユニークな内側のリストを追加
                else:
                    cleaned_sublist.append(item)  # Noneや他の値はそのまま追加
            cleaned_temp_list.append(cleaned_sublist)

        return cleaned_temp_list


    distance = 0
    

    temp_number = len(temp)
    temp_list = [] #各仮設道路の端点、交点の座標を格納するリスト(交点がない場合はNoneを格納)
    #交点の存在の確認
    for i in range(temp_number):
        distance_list_tempi = [temp[i][0],temp[i][1]]
    #     for k in range(temp_number):
    #         distance_list_tempi.insert(-1,None)
    #     for j in range(temp_number):
    #         if(i!= j):
    #             if is_intersect(temp[i][0],temp[i][1],temp[j][0],temp[j][1]): 
    #                 distance_list_tempi[j+1] = calculate_intersection(temp[i][0],temp[i][1],temp[j][0],temp[j][1])
        temp_list.append(distance_list_tempi)

    #交点の存在の確認
    # for i in range(temp_number):
    #     distance_list_tempi = [temp[i][0],temp[i][1]]
    #     # for k in range(temp_number):
    #     #     distance_list_tempi.insert(-1,None)
    #     for j in range(temp_number):
    #         if(i!= j):
    #             if is_intersect(temp[i][0],temp[i][1],temp[j][0],temp[j][1]): 
    #                 distance_list_tempi.append(calculate_intersection(temp[i][0],temp[i][1],temp[j][0],temp[j][1]))
    #     temp_list.append(distance_list_tempi)

    # print(temp_list,"\n")

    #各仮設道路の端点、交点から別の仮設道路に角度alphaで下した点を下す
    # additional_point = [[] for _ in range(temp_number)] #各仮設道路の端点・交点から別の仮設道路に角度alphaで下した点を格納するリスト
    # for i in range(temp_number):
    #     for j in range(len(temp_list[i])):
    #         if temp_list[i][j] is not None and isinstance(temp_list[i][j], tuple):
    #             for k in range(temp_number):
    #                 if k != i: 
    #                     point,distance = shortest_point_to_road(temp_list[i][j][0],temp_list[i][j][1],temp[k])
    #                     # print(f"{i}番目の仮設道路の{j}番目の座標{temp_list[i][j]}から{k}番目の仮設道路に下した座標:",point)
    #                     for l in range(len(point)):
    #                         if point[l] != None and distance != 0:
    #                             # additional_point_tempi.append(point[l]) #下した点を追加
    #                             additional_point[k].append(point[l])
    # # print("additional_point",additional_point)
    # additional_point_cleaned = remove_duplicates(additional_point)
    # # print("additional_point_cleaned",additional_point_cleaned)
    # for k in range(len(temp_list)):
    #     if isinstance(additional_point_cleaned[k], (list,tuple)) and len(additional_point_cleaned[k]) != 0:
    #         for l in range(len(additional_point_cleaned[k])):
    #             temp_list[k].insert(-1,additional_point[k][l])


    # for i in range(len(temp_list)):
    #     print(f"temp_list[{i}])",temp_list[i],"\n")

    temp_list_cleaned = [list(filter(lambda x: x is not None, sublist)) for sublist in temp_list]
    # print("交点、端点、角度αで下した点のリスト",temp_list_cleaned)
    # print("temp_list",temp_list)
    # print("temp_list_cleaned",temp_list_cleaned)

    #startとgoalを加えて最短コストをWarshall-Floydを使って計算

    def calculate_distance(i:float, j:float, k:float, l:float)->float:
        return np.sqrt((i - k)**2 + (j - l)**2)


    # Warshall-Floyd法によって仮設道路を用いた最短距離を計算最短
    def Warshall_Floyd(temp:list):
        point_number = sum([len(temp[i]) for i in range(len(temp))])
        distance = np.zeros((point_number,point_number))  
        for i in range(len(temp)):
            for j in range(len(temp[i])):
                index_ij = sum([len(temp[k]) for k in range(i)]) + j
                for k in range(len(temp)):
                    for l in range(len(temp[k])):
                        index_kl = sum([len(temp[m]) for m in range(k)]) + l
                        if i == k:
                            # distance[index_ij][index_kl] = math.sqrt((temp[i][j][0] - temp[k][l][0])**2 + (temp[i][j][1] - temp[k][l][1])**2) * temp_eff
                            distance[index_ij][index_kl] = calculate_distance(temp[i][j][0],temp[i][j][1],temp[k][l][0],temp[k][l][1]) * temp_eff
                        else:
                            distance[index_ij][index_kl] = calculate_distance(temp[i][j][0],temp[i][j][1],temp[k][l][0],temp[k][l][1])
        
        pre_point = np.full((point_number, point_number), [-1])  # 経由点を記録する配列
        for i in range(point_number):
            for j in range(point_number):
                pre_point[i][j] = i
        # for i in range(len(distance)):
        #      print(f"distance[{i}]",distance[i])
        # print("\n")
        for k in range(point_number):
            for i in range(point_number):
                for j in range(point_number):
                    if distance[i][j] > distance[i][k] + distance[k][j]:
                        pre_point[i][j] = pre_point[k][j]
                    distance[i][j] = min(distance[i][j], distance[i][k] + distance[k][j])

        # for i in range(len(distance)):
        #      print(f"distance[{i}]",distance[i])
        return distance[0][-1],pre_point



    #全ての切土、盛土の組み合わせに対して最短経路とそのコストを計算
    def shortest_route_search(cut_indicies, fill_indices,temp_list_cleaned):
        cost = np.zeros((len(cut_indices),len(fill_indices)))
        pre_point_matrix = []  # 経由点を記録する配列
        for i in range(len(cut_indices)):
            for j in range(len(fill_indices)):
                temp_list_cleaned_copy_ij = copy.deepcopy(temp_list_cleaned)

                                #cut_indices[i][0]から各仮設道路に角度alphaで下した点をtemp_list_cleaned_copy_ijに追加
                # for k in range(len(temp_list_cleaned)):
                #     temp_k = [temp_list_cleaned_copy_ij[k][0],temp_list_cleaned_copy_ij[k][1]]
                #     point = shortest_point_to_road(cut_indices[i][0][0],cut_indices[i][0][1],temp_k)[0]
                #     if point != None:
                #         for l in range(len(point)):
                #             if point[l] != None:
                #                 temp_list_cleaned_copy_ij[k].insert(-1,point[l])

                temp_list_cleaned_copy_ij.insert(0,[cut_indices[i][0]])
                temp_list_cleaned_copy_ij.append([fill_indices[j][0]])
                # print("temp_list_cleaned",temp_list_cleaned)
                # print("temp_list_cleaned_copy_ij",temp_list_cleaned_copy_ij)
                cost[i][j],pre_point_matrix_value = Warshall_Floyd(temp_list_cleaned_copy_ij)
                # print("pre_point_matrix_value",pre_point_matrix_value)
                pre_point_matrix.append(path_search(pre_point_matrix_value,temp_list_cleaned_copy_ij))
        return cost,pre_point_matrix
    # print("temp_list_cleaned after",temp_list_cleaned)


    #経路のindexのリストであるpre_pointを座標リストに整理
    def path_search(pre_point,temp_list_cleaned_copy_ij):
        # print("pre_point",pre_point)
        # print(len(pre_point))
        # print("\n")

        def find_path(pre_point, start, end):
            path = [start]
            while start != end:
                path.insert(1,end)
                end = pre_point[start][end]
            return path
        def clear_online(route):
            delta = 0.000001
            route_copy = copy.deepcopy(route)
            
            i = 1  # 最初の点は残すのでインデックス1から始めます
            while i < len(route_copy) - 1:
                # 点 A, B, C を取得
                A = route_copy[i - 1]
                B = route_copy[i]
                C = route_copy[i + 1]

                # ベクトル AB の傾きと BC の傾きを比較
                if abs((B[1] - A[1]) * (C[0] - B[0]) - (B[0] - A[0]) * (C[1] - B[1])) < delta:
                    # 点 B が A と C の間にある直線上にあれば削除
                    route_copy.pop(i)
                else:
                    # 点 B が直線上にない場合は次の点へ進む
                    i += 1
            
            return route_copy
        path = find_path(pre_point, 0, len(pre_point)-1)
        # print("path",path)
        # print(f"0番目の点から{len(pre_point)-1}番目の点までの経路:", path)

        temp_list_cleaned2 = []
        route = []
        for i in range(len(temp_list_cleaned_copy_ij)):
            for j in range(len(temp_list_cleaned_copy_ij[i])):
                temp_list_cleaned2.append(temp_list_cleaned_copy_ij[i][j])
        # print("temp_list_cleaned2",temp_list_cleaned2)
        for i in range(len(path)):
            route.append(temp_list_cleaned2[int(path[i])])
        # print("route",route)
        route = clear_online(route)
        # print("route_after",route)
        return route
       
    cost,pre_point_matrix = shortest_route_search(cut_indices, fill_indices,temp_list_cleaned)
    # for i in range(len(cost)):
    #     # print(f"cost[{i}]",cost[i])
    # print("\n")
    # print("temp_list_cleaned",temp_list_cleaned)

# 0番目の点から14番目の点までの経路を探す
    return cost,pre_point_matrix

def temp_length(temp):
    distance = 0
    for road in temp:
        distance += np.sqrt((road[0][0] - road[1][0])**2 + (road[0][1] - road[1][1])**2)
    return distance

#仮設道路がどれくらい利用されているかを計算
def temp_usage(route_list, temp):
    def on_segment(p:tuple, q:tuple, r:tuple) -> bool:
            """点rが線分pq上にあるかどうかを確認する関数"""
            if min(p[0], q[0]) <= r[0] <= max(p[0], q[0]) and min(p[1], q[1]) <= r[1] <= max(p[1], q[1]):
                return True
            return False

    def on_line(p:tuple, q:tuple, r:tuple):
                delta = 0.000001
                bool = False
                    # ベクトル pr の傾きと qr の傾きを比較,rがpqの間にあるかどうか⇒rがpq上にあるかを確認
                if abs((r[1] - p[1]) * (q[0] - r[0]) - (r[0] - p[0]) * (q[1] - r[1])) < delta and on_segment(p, q, r):
                    bool = True
                return bool
                

    # 2点間の距離を計算する関数
    def distance(point1, point2):
        return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)
    

    # 仮設道路の形式をチェック
    if not (isinstance(temp, list) and len(temp) == 2 and
            all(isinstance(point, tuple) and len(point) == 2 and
                all(isinstance(coord, (int, float)) for coord in point)
                for point in temp)):
        print("仮設道路は1つである必要があります")
        return exit(1)
    
    # 仮設道路の総距離を計算
    temp_distance = distance(temp[0], temp[1])
    
    # 全経路で仮設道路が使われている割合を計算
    total_temp_usage = 0

    for route in route_list:
        route_temp_usage = 0
        
        # 経路上の各セグメントについて距離を計算
        for i in range(len(route) - 1):
            segment1 = route[i]
            segment2 = route[i + 1]
            # print("segment1,segment2",segment1,segment2)
            
            #経路上に仮設道路上がある場合
            if (on_line(segment1,segment2,temp[0]) and on_line(segment1,segment2,temp[1])):
                route_temp_usage += distance(temp[0], temp[1])
            #     print("distance",distance(temp[0], temp[1]))
            #     print("経路上に仮設道路上がある場合")
            # # segmetn1とsegment2が両方仮設道路上にある場合 または segment1とsegment2が仮設道路の端点と一致する場合
            # elif((on_line(temp[0], temp[1], segment1)  and on_line(temp[0], temp[1], segment2))
            #    or ((segment1 == temp[0] and segment2 == temp[1]) or (segment1 == temp[1] and segment2 == temp[0]))):
            #     print("segment1とsegment2が両方仮設道路上にある場合")
            #     route_temp_usage += distance(segment1, segment2)
            elif(on_line(temp[0], temp[1], segment1)  and on_line(temp[0], temp[1], segment2)):
                # print("distance",distance(segment1, segment2))
                # print("segment1とsegment2が両方仮設道路上にある場合")
                route_temp_usage += distance(segment1, segment2)
            # segment1が仮設道路上にあり、segment2が仮設道路上にない場合
            elif (on_line(temp[0], temp[1], segment1) and not on_line(temp[0], temp[1], segment2)):
                if distance(segment2, temp[0]) < distance(segment2, temp[1]): # segment2,temp[0],segment1,temp[1]の順番で並んでいる場合
                    route_temp_usage += distance(segment1, temp[0])
                #     print("distance",distance(segment1, temp[0]))
                #     print("segment2,temp[0],segment1,temp[1]の順番で並んでいる場合")
                else: # temp[0],segment1,temp[1],segment2の順番で並んでいる場合
                    route_temp_usage += distance(segment1, temp[1])
                #     print("distance",distance(segment1, temp[1]))
                #     print("temp[0],segment1,temp[1],segment2の順番で並んでいる場合")
                # print("segment1が仮設道路上にあり、segment2が仮設道路上にない場合")
            #segment2が仮設道路上にあり、segment1が仮設道路上にない場合
            elif (on_line(temp[0], temp[1], segment2) and not on_line(temp[0], temp[1], segment1)):
                if distance(segment1, temp[0]) < distance(segment1, temp[1]): # segment2,temp[0],segment1,temp[1]の順番で並んでいる場合
                    route_temp_usage += distance(segment2, temp[0])
                    # print("distance",distance(segment2, temp[0]))
                    # print("segment1,temp[0],segment2,temp[1]の順番で並んでいる場合")
                else: # temp[0],segment1,temp[1],segment2の順番で並んでいる場合
                    route_temp_usage += distance(segment2, temp[1])
                #     print("distance",distance(segment2, temp[1]))
                #     print("temp[0],segment2,temp[1],segment1の順番で並んでいる場合")
                # print("segment2が仮設道路上にあり、segment1が仮設道路上にない場合")

            else:
                route_temp_usage += 0
                # print("その他の場合")
        # 仮設道路が使われた割合を加算
        total_temp_usage += route_temp_usage


    
    # 仮設道路が使われた割合を仮設道路の距離に対して返す
    if temp_distance == 0:
        return 0
    weight = total_temp_usage / temp_distance
    return total_temp_usage / temp_distance


def plot(grid_size_x,grid_size_y, route_arrows,temp,cut_indices,fill_indices):  #結果の可視化

   
    # # グリッドのサイズ
    # grid_size_x = 4  # x方向のサイズ
    # grid_size_y = 4  # y方向のサイズ

    # 格子点の座標を生成
    x = [i for i in range(grid_size_x)]
    y = [i for i in range(grid_size_y)]
    X, Y = np.meshgrid(x, y)

    # プロット用の格子点を設定
    x_coords = X.flatten()
    y_coords = Y.flatten()

    # 土量マトリックスを作成（仮に色付けのためのデータを用意）
    soil_amount = np.zeros((grid_size_y, grid_size_x))
    max_soil = max(max(cut_indices[i][1] for i in range(len(cut_indices))),max(fill_indices[i][1] for i in range(len(fill_indices))))
    # print("max_soil",max_soil)
    for [(i, j),k] in cut_indices:
        soil_amount[int(j), int(i)] = 1
    for [(i, j),k] in fill_indices:
        soil_amount[int(j),int(i)] = 0

    soil_amount_real = np.zeros((grid_size_y, grid_size_x))
    for [(i, j),k] in cut_indices:
        soil_amount_real[int(j), int(i)] = 2*k
    for [(i, j),k] in fill_indices:
        soil_amount_real[int(j),int(i)] = -2*k
    # print("soil_amount",soil_amount)

    def init_animate (): #初期化関数(これがないとanimate関数のi=0が2回繰り返される)
        pass

    def animate(i):
        # 前のフレームの矢印をクリア
        ax.clear()

        # print("i:",i)


        #土量を更新
        if i % 2 == 0:
            route_arrows_i = route_arrows[i//2]
            start_point = route_arrows_i[0]
            end_point = route_arrows_i[-1]
            # print("start_point", start_point)
            # print("end_point", end_point)
            # soil_amount[int(start_point[1]), int(start_point[0])] -= 1/max_soil/2  # 開始点の土量を減少
            # soil_amount[int(end_point[1]), int(end_point[0])] += 1/max_soil/2 # 終了点の土量を増加
            # print("soil_amount",soil_amount)

            soil_amount_real[int(start_point[1]), int(start_point[0])] -= 1  # 開始点の土量を減少
            soil_amount_real[int(end_point[1]), int(end_point[0])] += 1 # 終了点の土量を増加

            if soil_amount_real[int(start_point[1]), int(start_point[0])] == 0:
                soil_amount[int(start_point[1]), int(start_point[0])] = 0.5
            if soil_amount_real[int(end_point[1]), int(end_point[0])] == 0:
                soil_amount[int(end_point[1]), int(end_point[0])] = 0.5


    # グリッドの描画（背景）
        ax.pcolormesh(soil_amount, edgecolors='gray', linewidth=2, cmap='viridis', shading='flat', alpha=0.2)
        ax.scatter(x_coords+0.5, y_coords+0.5, color='blue', marker='o')  # 格子点のプロット

    # 格子点のラベルを表示（x, y方向に0.5ずらす）
        for x_val, y_val in zip(x_coords, y_coords):
            ax.text(x_val + 0.5, y_val + 0.4, f'{int(soil_amount_real[y_val][x_val])}', fontsize=12, ha='center', va='top')

        #仮設道路を描画
        for k in range(len(temp)):
            start_point, end_point = temp[k]
            adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
            adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
            ax.plot([adjusted_start[0], adjusted_end[0]], [adjusted_start[1], adjusted_end[1]], 
                        color='black', linewidth=8, alpha=0.5)
        


        #矢印を描画
        if i <=2 or i == 2*len(route_arrows)-1: #最初と最後の矢印の描画
            for j in range(i + 1):
                # print("j", j)
                #切土から盛土への矢印
                if(j % 2 == 0):
                    route_arrows_j = route_arrows[j//2]
                    # print("j%2==0")
                    # print("route_arrows_j", route_arrows_j)
                    if len(route_arrows_j) == 2:
                        start_point, end_point = route_arrows_j
                        adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                        adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                        ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                        arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))

                    elif len(route_arrows_j) > 2:
                        for k in range(len(route_arrows_j) - 1):
                            start_point = route_arrows_j[k]
                            end_point = route_arrows_j[k+1]
                            adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                            adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                            ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                                arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))
                    # 矢印の番号を矢印の中心に表示
                    mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                    ax.text(mid_point[0], mid_point[1]+0.1, f'{j//2+1}', fontsize=12, ha='center', color='black')

                #盛土から切土までの矢印
                elif (j%2 == 1 and j//2 <= len(route_arrows)-2):
                    # print("j%2==1")
                    start_point = route_arrows[j//2][-1]  # 現在の矢印の終了点
                    end_point = route_arrows[j//2 + 1][0]  # 次の矢印の開始点
                    # print("start_point", start_point)
                    # print("end_point", end_point)
                    adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                    adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                    ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                                arrowprops=dict(facecolor='blue', edgecolor='black', linewidth=2, alpha=0.4, shrink=0.05))
                    # 矢印の番号を矢印の中心に表示
                    mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                    ax.text(mid_point[0], mid_point[1]+0.1, f'({j//2+1})', fontsize=12, ha='center', color='grey')




        else: #i>2での矢印の描画(描画される矢印が2つまでにする)
            for j in range(i - 1,i + 1):
                # print("j", j)
                #切土から盛土への矢印
                if(j % 2 == 0):
                    route_arrows_j = route_arrows[j//2]
                    # print("j%2==0")
                    # print("route_arrows_j", route_arrows_j)
                    if len(route_arrows_j) == 2:
                        start_point, end_point = route_arrows_j
                        adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                        adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                        ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                                arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))
                    elif len(route_arrows_j) > 2:
                        for k in range(len(route_arrows_j) - 1):
                            start_point = route_arrows_j[k]
                            end_point = route_arrows_j[k+1]
                            adjusted_start = (start_point[0] + 0.5, start_point[1] + 0.5)
                            adjusted_end = (end_point[0] + 0.5, end_point[1] + 0.5)
                            ax.annotate('', xy=adjusted_end, xytext=adjusted_start,
                                arrowprops=dict(facecolor='red', edgecolor='black', linewidth=2, alpha=0.7, shrink=0.05))
                    # 矢印の番号を矢印の中心に表示
                    mid_point = ((adjusted_start[0] + adjusted_end[0]) / 2, (adjusted_start[1] + adjusted_end[1]) / 2)
                    ax.text(mid_point[0], mid_point[1]+0.1, f'{j//2+1}', fontsize=12, ha='center', color='black')
                    
                #盛土から切土までの矢印
                elif (j%2 == 1 and j//2 <= len(route_arrows)-2):
                    # print("j%2==1")
                    start_point = route_arrows[j//2][-1]  # 現在の矢印の終了点
                    end_point = route_arrows[j//2 + 1][0]  # 次の矢印の開始点
                    # print("start_point", start_point)
                    # print("end_point", end_point)
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
        ax.set_title(f'{grid_size_x}x{grid_size_y} Grid Animation with Route Arrows')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')

    # アニメーションの準備
    fig, ax = plt.subplots(figsize=(8, 6))

    # アニメーションの実行
    ani = animation.FuncAnimation(fig, animate,init_func=init_animate, frames=2*len(route_arrows)  , interval=1000, repeat=False)

    # GIFや動画として保存したい場合
    ani.save('animation.gif', writer='Pillow')

    # アニメーションを表示
    plt.show()
    return ani
    
