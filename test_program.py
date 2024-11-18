import math
import copy



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
            print("segment1,segment2",segment1,segment2)
            
            #経路上に仮設道路上がある場合
            if (on_line(segment1,segment2,temp[0]) and on_line(segment1,segment2,temp[1])):
                route_temp_usage += distance(temp[0], temp[1])
                print("distance",distance(temp[0], temp[1]))
                print("経路上に仮設道路上がある場合")
            # segmetn1とsegment2が両方仮設道路上にある場合 または segment1とsegment2が仮設道路の端点と一致する場合
            # elif((on_line(temp[0], temp[1], segment1)  and on_line(temp[0], temp[1], segment2))
            #    or ((segment1 == temp[0] and segment2 == temp[1]) or (segment1 == temp[1] and segment2 == temp[0]))):
            #     print("segment1とsegment2が両方仮設道路上にある場合")
            #     route_temp_usage += distance(segment1, segment2)
            elif(on_line(temp[0], temp[1], segment1)  and on_line(temp[0], temp[1], segment2)):
                print("distance",distance(segment1, segment2))
                print("segment1とsegment2が両方仮設道路上にある場合")
                route_temp_usage += distance(segment1, segment2)
            # segment1が仮設道路上にあり、segment2が仮設道路上にない場合
            elif (on_line(temp[0], temp[1], segment1) and not on_line(temp[0], temp[1], segment2)):
                if distance(segment2, temp[0]) < distance(segment2, temp[1]): # segment2,temp[0],segment1,temp[1]の順番で並んでいる場合
                    route_temp_usage += distance(segment1, temp[0])
                    print("distance",distance(segment1, temp[0]))
                    print("segment2,temp[0],segment1,temp[1]の順番で並んでいる場合")
                else: # temp[0],segment1,temp[1],segment2の順番で並んでいる場合
                    route_temp_usage += distance(segment1, temp[1])
                    print("distance",distance(segment1, temp[1]))
                    print("temp[0],segment1,temp[1],segment2の順番で並んでいる場合")
                print("segment1が仮設道路上にあり、segment2が仮設道路上にない場合")
            #segment2が仮設道路上にあり、segment1が仮設道路上にない場合
            elif (on_line(temp[0], temp[1], segment2) and not on_line(temp[0], temp[1], segment1)):
                if distance(segment1, temp[0]) < distance(segment1, temp[1]): # segment2,temp[0],segment1,temp[1]の順番で並んでいる場合
                    route_temp_usage += distance(segment2, temp[0])
                    print("distance",distance(segment2, temp[0]))
                    print("segment1,temp[0],segment2,temp[1]の順番で並んでいる場合")
                else: # temp[0],segment1,temp[1],segment2の順番で並んでいる場合
                    route_temp_usage += distance(segment2, temp[1])
                    print("distance",distance(segment2, temp[1]))
                    print("temp[0],segment2,temp[1],segment1の順番で並んでいる場合")
                print("segment2が仮設道路上にあり、segment1が仮設道路上にない場合")

            else:
                route_temp_usage += 0
                print("その他の場合")
        # 仮設道路が使われた割合を加算
        total_temp_usage += route_temp_usage


    
    # 仮設道路が使われた割合を仮設道路の距離に対して返す
    return total_temp_usage / temp_distance

# print(on_line((1, 1), (2, 2), (1, 1)))  # False

# 例の使用法
# route_list = [[(1, 1), (1.5, 1.5), (1.6, 1.6)], [(1.61, 1.6), (3, 3), (1.5, 1.5)]]
route_list = [[(1, 1), (1.5, 1.5), (1.6, 1.6)]]

temp = [(1, 1), (2, 2)]

print(temp_usage(route_list, temp))  # 結果は仮設道路の利用割合
