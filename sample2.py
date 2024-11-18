import random

grid_size_x = 4
grid_size_y = 4
# 仮設道路リスト
temporary_roads = [
    {
        (2, 0): {2},
        (1, 1): {2, 5},
        (0, 2): {5},
    },
    {
        (1, 2): {2},
        (0, 3): {5},
    },
]

# DIRECTIONSリスト
DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

# 操作関数
def modify_temporary_roads(temp):
    if random.random() < 0.15 and len(temp) > 0:  # 15%の確率で要素を削除
        print("道路を削除")
        selected_road = random.choice(temp)
        print("selected_road",selected_road)
        temp.remove(selected_road)
    elif random.random() < 0.30:  # 15%（0.15 + 0.15）の確率で要素を追加
        print("道路を追加")
        new_temporary_road = {}
        coord = (random.randint(0, 3), random.randint(0, 3))  # ランダムな座標
        while True:
            new_direction_index = random.randint(0, len(DIRECTIONS) - 1)
            # print("new_direction_index",new_direction_index)
            print("DIRECTIONS[new_direction_index]",DIRECTIONS[new_direction_index])
            print("new_coord",coord[0]+  DIRECTIONS[new_direction_index][0], coord[1]+  DIRECTIONS[new_direction_index][1])
            if  0 <= coord[0]+  DIRECTIONS[new_direction_index][0] <= grid_size_x-1 and 0 <= coord[1]+ DIRECTIONS[new_direction_index][1] <= grid_size_y-1:
                new_neighbor_coord = (coord[0]+  DIRECTIONS[new_direction_index][0], coord[1]+  DIRECTIONS[new_direction_index][1])
                break
        new_temporary_road[coord] = {new_direction_index}
        neighbor_index = DIRECTIONS.index((-DIRECTIONS[new_direction_index][0], -DIRECTIONS[new_direction_index][1]))
        new_temporary_road[new_neighbor_coord] = {neighbor_index}
        print("new_temporary_road",new_temporary_road)
        temp.append(new_temporary_road)
    else:  # 残りの確率で既存の要素をランダムに変更
        if len(temp) > 0:
            selected_road = random.choice(temp)
            selected_coord = list(selected_road.keys())[0] if random.choice([True, False]) else list(selected_road.keys())[-1]
            print("selected_road",selected_road)
            print("selected_coord",selected_coord)
            # 方向の変更を実行
            if random.choice([True, False]):
                # 既存の方向を削除
                print("端点を削除")
                if len(selected_road[selected_coord]) > 0:
                    selected_road[selected_coord].remove(random.choice(list(selected_road[selected_coord])))
                # 値が空の場合はキー（座標）を削除
                if not selected_road[selected_coord]:  # 空のセットならTrue
                    del selected_road[selected_coord]       
            else:
                # 新しい方向を追加
                print("端点を追加")
                new_direction = random.randint(0, len(DIRECTIONS) - 1)
                print("new_direction",new_direction)
                if new_direction not in selected_road[selected_coord]:
                    selected_road[selected_coord].add(new_direction)
                    # 新しい座標を計算
                    dx, dy = DIRECTIONS[new_direction]
                    new_coord = (selected_coord[0] + dx, selected_coord[1] + dy)
                    # 新しい座標の方向を追加
                    reverse_direction = DIRECTIONS.index((-dx, -dy))
                    if new_coord not in selected_road:
                        selected_road[new_coord] = set()
                    selected_road[new_coord].add(reverse_direction)
    print("\n")

# 使用例
print("Before:", temporary_roads)
for i in range(5):
    modify_temporary_roads(temporary_roads) 
    print(f"{i+1}times後のAfter:", temporary_roads)