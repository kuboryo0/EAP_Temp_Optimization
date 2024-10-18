
import numpy as np
import time
import math
cut_indices = [[(1, 0),2],[(1, 2),1],[(2, 0),1], [(2, 1),1], [(2, 2),1],[(3, 0),1],[(3, 1),1]]
fill_indices = [[(0, 0),1], [(0, 1),1], [(0, 2),1], [(1, 1),2],[(3, 2),2]]
temp_road = [[(0, 0), (2, 2)]]
temp_eff = 0.5
v =  1/temp_eff
tan_alpha = math.sqrt(v**2-1)
sin_alpha = math.sqrt(v**2-1)/v
distance = 0
def calculate_distance(i, j, k, l):
    return np.sqrt((i - k)**2 + (j - l)**2)
#ある座標からある道路への最短の点座標を求める
def shortest_point_to_road(x,y,temp_i):
    if len(temp_i) != 2:
        print("shortest_point_to_road error:temp_iは[(p1,q1),(p2,q2)]の型で入力してください")
    point = []
    m = (temp_i[1][1]-temp_i[0][1])/(temp_i[1][0]-temp_i[0][0]) 
    n = (temp_i[0][1]*temp_i[1][0]-temp_i[0][0]*temp_i[1][1])/(temp_i[1][0]-temp_i[0][0])
    beta = math.atan(m)
    d = abs(m*x-y+n)/math.sqrt(m**2+1) 
    if (y > m*x+n):
        point1_x = x + d*(math.cos(beta)/tan_alpha+math.sin(beta))
        point1_y = y - d*(math.cos(beta)-math.sin(beta)/tan_alpha)
        point2_x = x - d*(math.cos(beta)/tan_alpha-math.sin(beta))
        point2_y = y - d*(math.cos(beta)+math.sin(beta)/tan_alpha)
    elif (y < m*x+n):
        point1_x = x + d*(math.cos(beta)/tan_alpha-math.sin(beta))
        point1_y = y + d*(math.cos(beta)+math.sin(beta)/tan_alpha)
        point2_x = x - d*(math.cos(beta)/tan_alpha+math.sin(beta))
        point2_y = y + d*(math.cos(beta)-math.sin(beta)/tan_alpha)
    else:
        point.append((x,y))
        distance = 0
        return  point,distance
    #pointに座標を加える
    #point1が仮設道路上にある場合
    if ((point1_x >= temp_i[0][0] and point1_x <= temp_i[1][0]) or (point1_x >= temp_i[1][0] and point1_x <= temp_i[0][0]) or 
        (point1_y >= temp_i[0][1] and point1_y <= temp_i[1][1]) or (point1_y >= temp_i[1][1] and point1_y <= temp_i[0][1])):
        point.append((point1_x,point1_y))
        distance = calculate_distance(x,y,point1_x,point1_y)
    #point2が仮設道路上にある場合
    if ((point2_x >= temp_i[0][0] and point2_x <= temp_i[1][0]) or (point2_x >= temp_i[1][0] and point2_x <= temp_i[0][0]) or 
        (point2_y >= temp_i[0][1] and point2_y <= temp_i[1][1]) or (point2_y >= temp_i[1][1] and point2_y <= temp_i[0][1])):
        point.append((point2_x,point2_y))
        distance = calculate_distance(x,y,point2_x,point2_y)
    #point1もpoint2も仮設道路上にない場合
    if (len(point) == 0):
        # #仮設道路がpoint1とpoint2の間にある場合pointには加えない
        # if (((temp_i[0][0] >= point1_x and temp_i[0][0] <= point2_x) or (temp_i[0][0] <= point1_x and temp_i[0][0] >= point2_x)) and
        #     ((temp_i[0][1] >= point1_y and temp_i[0][1] <= point2_y) or (temp_i[0][1] <= point1_y and temp_i[0][1] >= point2_y)) and 
        #     ((temp_i[1][0] >= point1_x and temp_i[1][0] <= point2_x) or (temp_i[1][0] <= point1_x and temp_i[1][0] >= point2_x)) and
        #     ((temp_i[1][1] >= point1_y and temp_i[1][1] <= point2_y) or (temp_i[1][1] <= point1_y and temp_i[1][1] >= point2_y))):
        #仮設道路がpoint1側にある時
        if (calculate_distance(point1_x,point1_y,temp_i[0][0],temp_i[0][1]) < calculate_distance(point2_x,point2_y,temp_i[0][0],temp_i[0][1])):
            #(x1,y1)がpoint1に近い場合
            if (calculate_distance(point1_x,point1_y,temp_i[0][0],temp_i[0][1]) < calculate_distance(point1_x,point1_y,temp_i[1][0],temp_i[1][1])):
                point.append((temp_i[0][0],temp_i[0][1]))
                distance = calculate_distance(x,y,temp_i[0][0],temp_i[0][1])
            #(x2,y2)がpoint1に近い場合
            elif (calculate_distance(point1_x,point1_y,temp_i[0][0],temp_i[0][1]) > calculate_distance(point1_x,point1_y,temp_i[1][0],temp_i[1][1])):
                point.append((temp_i[1][0],temp_i[1][1]))
                distance = calculate_distance(x,y,temp_i[1][0],temp_i[1][1])
        #仮設道路がpoint2側にある時  
        elif (calculate_distance(point1_x,point1_y,temp_i[0][0],temp_i[0][1]) > calculate_distance(point2_x,point2_y,temp_i[0][0],temp_i[0][1])):
            if  (calculate_distance(point2_x,point2_y,temp_i[0][0],temp_i[0][1]) < calculate_distance(point2_x,point2_y,temp_i[1][0],temp_i[1][1])):
                point.append((temp_i[0][0],temp_i[0][1]))
                distance = calculate_distance(x,y,temp_i[0][0],temp_i[0][1])
            #(x2,y2)がpoint2に近い場合
            elif (calculate_distance(point2_x,point2_y,temp_i[0][0],temp_i[0][1]) > calculate_distance(point2_x,point2_y,temp_i[1][0],temp_i[1][1])):
                point.append((temp_i[1][0],temp_i[1][1]))
                distance = calculate_distance(x,y,temp_i[1][0],temp_i[1][1])
    return point,distance


# 2つの仮設道路が交差するかどうかを判定する関数
def is_intersect(p1, p2, q1, q2):
    def orientation(p, q, r):
        """3点の並び順を計算。0: コリニア（同一直線上）, 1: 時計回り, 2: 反時計回り"""
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0  # コリニア
        elif val > 0:
            return 1  # 時計回り
        else:
            return 2  # 反時計回り

    def on_segment(p, q, r):
        """点rが線分pq上にあるかどうかを確認する関数"""
        if min(p[0], q[0]) <= r[0] <= max(p[0], q[0]) and min(p[1], q[1]) <= r[1] <= max(p[1], q[1]):
            return True
        return False

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

# 交点の計算
def calculate_intersection(p1, p2, q1, q2):
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


#ある座標から2つの仮設道路に対してどちらの仮設道路が近いかを判定する
def closest_road(x,y,temp):
    if not (isinstance(temp, list) and len(temp) == 2 and
            isinstance(temp[0], list) and len(temp[0]) == 2 and
            isinstance(temp[1], list) and len(temp[1]) == 2 and
            isinstance(temp[0][0], (list, tuple)) and len(temp[0][0]) == 2 and
            isinstance(temp[0][1], (list, tuple)) and len(temp[0][1]) == 2 and
            isinstance(temp[1][0], (list, tuple)) and len(temp[1][0]) == 2 and
            isinstance(temp[1][1], (list, tuple)) and len(temp[1][1]) == 2):
        print("closest_road error:仮設道路の形式があっていません")
        exit()
    point = []
    point1,distance1 = shortest_point_to_road(x,y,temp[0])
    point2,distance2 = shortest_point_to_road(x,y,temp[1])
    # print("point1",point1)
    # print("distance1",distance1)
    # print("point2",point2)
    # print("distance2",distance2)
    if(distance1 < distance2):
        point.append(point1)
        return point,0
    elif(distance1 > distance2):
        point.append(point2)
        return point,1
    else:
        point.append(point1)
        point.append(point2)
        return point,2


# 2つの仮設道路に関して交点を利用するルートの距離コスト計算
#x1,y1,x2,y2:2つの点の座標,point1,point2:それぞれの点における最短の仮設道路上の点,temp:2つの仮設道路の情報
def calculate_distance_2temp_int(x1, y1, x2, y2,temp):
    print("step1")
    intersection_utilized = False
    if not (isinstance(temp, list) and len(temp) == 2 and
            isinstance(temp[0], list) and len(temp[0]) == 2 and
            isinstance(temp[1], list) and len(temp[1]) == 2 and
            isinstance(temp[0][0], (list, tuple)) and len(temp[0][0]) == 2 and
            isinstance(temp[0][1], (list, tuple)) and len(temp[0][1]) == 2 and
            isinstance(temp[1][0], (list, tuple)) and len(temp[1][0]) == 2 and
            isinstance(temp[1][1], (list, tuple)) and len(temp[1][1]) == 2):
        print("calculate_distance_temp_int error: 仮設道路の数が2つではありません")
        exit()
    print("step2")
    if (closest_road(x1,y1,temp)[1]  ==2 or closest_road(x2,y2,temp)[1] ==2):
        print("2つの仮設道路のどちらかが同じ距離の場合、交点を利用できません")
        return None,intersection_utilized
    elif (closest_road(x1,y1,temp)[1] == closest_road(x2,y2,temp)[1]):
        print("2つの仮設道路が同じ側にあるため、交点を利用できません")
        return None,intersection_utilized
    (temp1_start, temp1_end), (temp2_start, temp2_end) = temp
    print("step3")
    # 仮設道路が交差するかどうか確認
    if is_intersect(temp1_start, temp1_end, temp2_start, temp2_end):
        intersection = calculate_intersection(temp1_start, temp1_end, temp2_start, temp2_end)
        print(f"交点: {intersection}")
        if intersection:
            print(f"交点: {intersection}")
            min_dis1 = math.inf
            min_dis2 = math.inf
            # (x1,y1),(x2,y2)から仮設道路に下した点から交点までの距離
            if (closest_road(x1,y1,temp)[1] == 0):
                print("closest_road(x1,y1,temp)[1] == 0")
                print("shortest_point_to_road (x1,y1)から仮設道路1への最短の点:\n",shortest_point_to_road(x1,y1,temp[0])[0]) #(x1,y1)から仮設道路2への最短点の座標
                print("shortest_point_to_road (x2,y2)から仮設道路2への最短の点:\n",shortest_point_to_road(x1,y1,temp[1])[0]) #(x1,y1)から仮設道路1への最短点の座標
                
                for i in range(len(shortest_point_to_road(x1,y1,temp[0])[0])):
                    if(calculate_distance(shortest_point_to_road(x1,y1,temp[0])[0][i][0],shortest_point_to_road(x1,y1,temp[0])[0][i][1] , intersection[0], intersection[1]) < min_dis1):
                        min_dis1 = calculate_distance(shortest_point_to_road(x1,y1,temp[0])[0][i][0],shortest_point_to_road(x1,y1,temp[0])[0][i][1] , intersection[0], intersection[1])
                for i in range(len(shortest_point_to_road(x2,y2,temp[1])[0])):
                    if(calculate_distance(shortest_point_to_road(x2,y2,temp[1])[0][i][0],shortest_point_to_road(x2,y2,temp[1])[0][i][1] , intersection[0], intersection[1]) < min_dis2):
                        min_dis2 = calculate_distance(shortest_point_to_road(x2,y2,temp[1])[0][i][0],shortest_point_to_road(x2,y2,temp[1])[0][i][1] , intersection[0], intersection[1])
            elif (closest_road(x1,y1,temp)[1] == 1):
                print("closest_road(x1,y1,temp)[1] == 1")
                print("shortest_point_to_road (x1,y1)から仮設道路2への最短の点: ",shortest_point_to_road(x1,y1,temp[1])[0]) #(x1,y1)から仮設道路2への最短点の座標
                print("shortest_point_to_road (x2,y2)から仮設道路1への最短の点:",shortest_point_to_road(x1,y1,temp[0])[0]) #(x1,y1)から仮設道路1への最短点の座標
                # distance_1 = calculate_distance(shortest_point_to_road(x1,y1,temp[1])[0][0],shortest_point_to_road(x1,y1,temp[1])[0][1] , intersection[0], intersection[1])
                # distance_2 = calculate_distance(shortest_point_to_road(x2,y2,temp[0])[0][0],shortest_point_to_road(x2,y2,temp[0])[0][1] , intersection[0], intersection[1])
                for i in range(len(shortest_point_to_road(x1,y1,temp[1])[0])):
                    if(calculate_distance(shortest_point_to_road(x1,y1,temp[1])[0][i][0],shortest_point_to_road(x1,y1,temp[1])[0][i][1] , intersection[0], intersection[1]) < min_dis1):
                        min_dis1 = calculate_distance(shortest_point_to_road(x1,y1,temp[1])[0][i][0],shortest_point_to_road(x1,y1,temp[1])[0][i][1] , intersection[0], intersection[1])
                for i in range(len(shortest_point_to_road(x2,y2,temp[0])[0])):
                    if(calculate_distance(shortest_point_to_road(x2,y2,temp[0])[0][i][0],shortest_point_to_road(x2,y2,temp[0])[0][i][1] , intersection[0], intersection[1]) < min_dis2):
                        min_dis2 = calculate_distance(shortest_point_to_road(x2,y2,temp[0])[0][i][0],shortest_point_to_road(x2,y2,temp[0])[0][i][1] , intersection[0], intersection[1])
            print("min_dis1",min_dis1)
            print("min_dis2",min_dis2)
            total_distance = (min_dis1 + min_dis2)*temp_eff
            intersection_utilized = True
            return total_distance,intersection_utilized
        else:
            print("仮設道路は平行で交差しません。")
            return None,intersection_utilized
    else:
        print("仮設道路は交差しません。")
        return None,intersection_utilized

#交点を利用するルートの距離コスト計算
def calculate_distance_temp_int(x1,y1,x2,y2,temp):
    return

#1つの仮設道路を利用した最短ルートの距離コスト計算
#2つの点、それらにおける最短の仮設道路上の点、最短距離、仮設道路の情報を入力
#最短距離と仮設道路を利用したかを返す
def calculate_distance_temp(x1,y1,x2,y2,point1_list,point2_list,distace_point1_temp,distace_point2_temp,temp):
    temp_utillized = False
    bool = False
    min_dis =math.inf
    if(temp[0][0]==temp[1][0]):
        for i in range(len(point1_list)):
                for j in range(len(point2_list)):
                    if (point1_list[i][1] > point2_list[j][1]or
                        point1_list[i][1] < point2_list[j][1]):
                        bool = True
        if(bool):
            for i in range(len(point1_list)):
                for j in range(len(point2_list)):
                    dis = calculate_distance(point1_list[i][0],point1_list[i][1],point2_list[j][0],point2_list[j][1])
                    if(dis < min_dis):
                        min_dis = dis
            temp_utillized = True
        else:
            return None,temp_utillized
    else:           
        m = (temp[1][1]-temp[0][1])/(temp[1][0]-temp[0][0])
        if (m > 0):
            print("m>0")
            for i in range(len(point1_list)):
                for j in range(len(point2_list)):
                    if ((point1_list[i][0] > point2_list[j][0] and point1_list[i][1] > point2_list[j][1])or
                        (point1_list[i][0] < point2_list[j][0] and point1_list[i][1] < point2_list[j][1])):
                        bool = True
            if(bool):
                for i in range(len(point1_list)):
                    for j in range(len(point2_list)):
                        dis = calculate_distance(point1_list[i][0],point1_list[i][1],point2_list[j][0],point2_list[j][1])
                        if(dis < min_dis):
                            min_dis = dis
                temp_utillized = True
            else:
                return None,temp_utillized
        elif (m < 0):
            print("m<0")
            for i in range(len(point1_list)):
                for j in range(len(point2_list)):
                    if ((point1_list[i][0] > point2_list[j][0] and point1_list[i][1] < point2_list[j][1])or
                        (point1_list[i][0] < point2_list[j][0] and point1_list[i][1] > point2_list[j][1])):
                        bool = True
            if(bool):
                for i in range(len(point1_list)):
                    for j in range(len(point2_list)):
                        dis = calculate_distance(point1_list[i][0],point1_list[i][1],point2_list[j][0],point2_list[j][1])
                        if(dis < min_dis):
                            min_dis = dis
                temp_utillized = True
            else:
                return None,temp_utillized
        else:
            print("m=0")
            for i in range(len(point1_list)):
                for j in range(len(point2_list)):
                    if (point1_list[i][0] > point2_list[j][0]or
                        point1_list[i][0] < point2_list[j][0]):
                        bool = True
            if(bool):
                for i in range(len(point1_list)):
                    for j in range(len(point2_list)):
                        dis = calculate_distance(point1_list[i][0],point1_list[i][1],point2_list[j][0],point2_list[j][1])
                        if(dis < min_dis):
                            min_dis = dis
                temp_utillized = True
    if (temp_utillized):
        print("min_dis",min_dis)
        distance = distace_point1_temp + distace_point2_temp + temp_eff*min_dis
    return distance,temp_utillized


    
#2つの仮設道路から1つ選択して利用するルートの距離コスト計算
#最短距離と選択した仮設道路を返す
def calculate_distance_2temp(x1,y1,x2,y2,temp):
    if len(temp) != 2:
        print("仮設道路の数が2つではありません")
        exit ()
    m1 = (temp[0][1][1]-temp[0][0][1])/(temp[0][1][0]-temp[0][0][0])
    m2 = (temp[1][1][1]-temp[1][0][1])/(temp[1][1][0]-temp[1][0][0])
    n1 = (temp[0][0][1]*temp[0][1][0]-temp[0][0][0]*temp[0][1][1])/(temp[0][1][0]-temp[0][0][0])
    n2 = (temp[1][0][1]*temp[1][1][0]-temp[1][0][0]*temp[1][1][1])/(temp[1][1][0]-temp[1][0][0])
    if m1 == m2:
        print("2つの仮設道路が平行です")
        exit()
    point1_temp1_point_list, distace_point1_temp1= shortest_point_to_road(x1,y1,temp[0]) #(x1,y1)から仮設道路1への最短点のリスト
    point1_temp1_point_list, distace_point2_temp1= shortest_point_to_road(x2,y2,temp[0]) #(x2,y2)から仮設道路1への最短点のリスト
    point2_temp1_point_list, distace_point1_temp2= shortest_point_to_road(x1,y1,temp[1]) #(x1,y1)から仮設道路2への最短点のリスト
    point2_temp1_point_list, distace_point2_temp2= shortest_point_to_road(x2,y2,temp[1]) #(x2,y2)から仮設道路2への最短点のリスト

    #2つの点が近い仮設道路が同じ場合
    if (distace_point1_temp1 < distace_point1_temp2 and distace_point2_temp1 < distace_point2_temp2): #両点が仮設道路1に近い場合
        distance, temp_utillized = calculate_distance_temp(x1,y1,x2,y2,point1_temp1_point_list,point2_temp1_point_list,distace_point1_temp1,distace_point2_temp1,temp[0])
    elif (distace_point1_temp1 > distace_point1_temp2 and distace_point2_temp1 > distace_point2_temp2): #両点が仮設道路2に近い場合
        distance, temp_utillized = calculate_distance_temp(x1,y1,x2,y2,point1_temp1_point_list,point2_temp1_point_list,distace_point1_temp1,distace_point2_temp1,temp[1])

    #2つの点が近い仮設道路が異なる場合
    return

temp1 = [[(0, 0), (2, 2)], [(2, 0), (0, 3)]]
# temp2 = [[(0, 0), (2, 2)], [(2, 0), (0, 2)]]
# print(is_intersect(temp1[0][0], temp1[0][1], temp1[1][0], temp1[1][1]))
a = calculate_distance_2temp_int(1,2,1,0,temp1)
print(a) 

