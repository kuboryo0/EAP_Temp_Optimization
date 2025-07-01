import numpy as np

# 定数としてグリッドサイズを設定
GRID_SIZE_X = 12  # x方向のサイズ
GRID_SIZE_Y = 4   # y方向のサイズ

# soil_amount配列の初期化
def initialize_soil_amount():
    return np.ones((GRID_SIZE_X, GRID_SIZE_Y))

# インデックスの値を適用する関数
def apply_indices(soil_amount, indices, multiplier):
    for (x, y), value in indices:
        soil_amount[x, y] = multiplier * value

# メイン関数
def main():
    # データ定義
    cut_indices = [
        ((2, 0), 3700), ((3, 0), 9000), ((4, 0), 9000), ((5, 0), 8000), ((11, 0), 11200),
        ((1, 1), 22500), ((2, 1), 33800), ((3, 1), 36000), ((4, 1), 23000), ((5, 1), 22200),
        ((6, 1), 8100), ((10, 1), 14300), ((11, 1), 7900),
        ((1, 2), 22500), ((2, 2), 28100), ((3, 2), 23000), ((4, 2), 24300), ((5, 2), 14200),
        ((2, 3), 2300), ((3, 3), 1200), ((4, 3), 9000), ((8, 3), 2300)
    ]

    fill_indices = [
        ((0, 0), 15000), ((1, 0), 3700), ((6, 0), 1000), ((7, 0), 11200), ((8, 0), 2300),
        ((9, 0), 22000), ((10, 0), 6900),
        ((0, 1), 62600), ((7, 1), 24800), ((8, 1), 9900), ((9, 1), 2200),
        ((0, 2), 3700), ((6, 2), 12400), ((7, 2), 34400), ((8, 2), 72500), ((9, 2), 28500),
        ((10, 2), 2500),
        ((1, 3), 1400), ((5, 3), 5900), ((6, 3), 9900), ((7, 3), 2700), ((9, 3), 100)
    ]

    zero_indices = [
        ((11, 2), 0),
        ((0, 3), 0), ((10, 3), 0), ((11, 3), 0)
    ]

    # 配列の初期化
    soil_amount = initialize_soil_amount()

    # 各インデックスに対応する値を適用
    apply_indices(soil_amount, cut_indices, 1.0)  # cut_indicesはそのままプラス
    apply_indices(soil_amount, fill_indices, -1.0)  # fill_indicesはマイナスに設定
    apply_indices(soil_amount, zero_indices, 0.0)  # zero_indicesは0に設定

    # 配列を表示
    for y in range(GRID_SIZE_Y):
        print("{")
        for x in range(GRID_SIZE_X):
            print(f"{soil_amount[x, y]:.1f}", end=", ")
        print("},")

if __name__ == "__main__":
    main()
