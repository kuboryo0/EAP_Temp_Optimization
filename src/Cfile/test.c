#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <limits.h>
#include <stdlib.h>
#include <float.h>
#include <limits.h>
#define GRID_SIZE_X 4
#define GRID_SIZE_Y 4
#define STEPS 10
#define TEMP_EFF 0.5 // 仮設道路の効果効率
#define grid_size 1
#define DIRECTIONS_COUNT 8
/*
temporary_roads.coordpair[i].coords[0].x: i番目の座標ペアの始点(0番目の座標)のx座標
temporary_roads.statuslist[i].status[0]: i番目の座標ペアの始点(0番目の座標)の状態 (0: 未建設, 1: 建設済み)

solution.coordpair[i].coords[0].x: i番目の座標ペアの始点(0番目の座標)のx座標
solution.timings[i][step]: i番目の座標ペアのstep番目の建設の有無　(0: 未建設, 1: 建設)
*/

typedef struct {
    int x, y; // x座標, y座標
} Coord;

// 道路の始点と終点をペアとして表す
typedef struct {
    Coord coords[2];
} CoordPair;

typedef struct {
    int status[2];           // 各端点の状態 (例: [0: 未使用, 1: 使用中])
} Status;

// 複数の仮設道路を管理する
typedef struct {
    int count;                         // 座標ペアの数
    CoordPair *coordpair;              // 座標ペアのリスト
    Status *statuslist;                // 仮設道路の状態のリスト (例:{{0,1},{1,1}....}, 0: 両方未建設, 1: 道路1のみ建設済み)
} TemporaryRoads;

// 解決策の情報を表す
typedef struct {
    int count;                // 座標ペアの数
    int step;                 // ステップ数
    CoordPair *coordpair;     // 座標ペアのリスト
    int **timings;              // 各道路の建設タイミング(例{{1,0,1},{1,0,0}....}, 0: 建設なし, 1: 建設). row: count, col: STEPS
} Solution;

typedef struct {
    Coord start;
    Coord goal;
    double volume;
} Allocation;

// グローバル変数として方向の定義
const Coord DIRECTIONS[DIRECTIONS_COUNT] = {
    {-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}
};


typedef struct {
    CoordPair coordpair;
    int *timing;
} CoordPairWithTiming;

int compare_with_timing(const void *a, const void *b) {
    const CoordPairWithTiming *entry1 = (const CoordPairWithTiming *)a;
    const CoordPairWithTiming *entry2 = (const CoordPairWithTiming *)b;
    return compare_search(&(entry1->coordpair), &(entry2->coordpair));
}

void sort_solution(Solution *solution) {
    // CoordPairとtimingを一緒に扱う構造体の配列を作成
    CoordPairWithTiming *entries = (CoordPairWithTiming *)malloc(solution->count * sizeof(CoordPairWithTiming));
    for (int i = 0; i < solution->count; i++) {
        entries[i].coordpair = solution->coordpair[i];
        entries[i].timing = solution->timings[i];
    }

    // ソート
    qsort(entries, solution->count, sizeof(CoordPairWithTiming), compare_with_timing);

    // ソート結果を元の構造に反映
    for (int i = 0; i < solution->count; i++) {
        solution->coordpair[i] = entries[i].coordpair;
        solution->timings[i] = entries[i].timing;
    }

    free(entries);
}

typedef struct {
    CoordPair coordpair;
    Status status;
} CoordPairWithStatus;

int compare_with_status(const void *a, const void *b) {
    const CoordPairWithStatus *entry1 = (const CoordPairWithStatus *)a;
    const CoordPairWithStatus *entry2 = (const CoordPairWithStatus *)b;
    return compare_search(&(entry1->coordpair), &(entry2->coordpair));
}

void sort_temporary_roads(TemporaryRoads *temporary_roads) {
    // CoordPairとstatusを一緒に扱う構造体の配列を作成
    CoordPairWithStatus *entries = (CoordPairWithStatus *)malloc(temporary_roads->count * sizeof(CoordPairWithStatus));
    for (int i = 0; i < temporary_roads->count; i++) {
        entries[i].coordpair = temporary_roads->coordpair[i];
        entries[i].status = temporary_roads->statuslist[i];
    }

    // ソート
    qsort(entries, temporary_roads->count, sizeof(CoordPairWithStatus), compare_with_status);

    // ソート結果を元の構造に反映
    for (int i = 0; i < temporary_roads->count; i++) {
        temporary_roads->coordpair[i] = entries[i].coordpair;
        temporary_roads->statuslist[i] = entries[i].status;
    }

    free(entries);
}

CoordPair normalize_pair(Coord a, Coord b) {
    CoordPair pair;
    if (a.x < b.x || (a.x == b.x && a.y < b.y)) {
        pair.coords[0] = a;
        pair.coords[1] = b;
    } else {
        pair.coords[0] = b;
        pair.coords[1] = a;
    }
    return pair;
}

double calculate_distance_3D(Coord a, Coord b, double soil_amount[][GRID_SIZE_Y]) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = (soil_amount[a.x][a.y]-soil_amount[b.x][b.y])/(grid_size*grid_size*grid_size);
    return sqrt(dx * dx + dy * dy + dz * dz);
}

void temp_in_step(TemporaryRoads *current_temps, int step, Solution solution, double *built_length_list, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], double *built_length) {
    int road_count = current_temps->count;
    printf("road_count: %d\n", road_count);
    for (int i = 0; i < road_count; i++) {
        CoordPair road = solution.coordpair[i];
        int coord1_x = road.coords[0].x;
        int coord1_y = road.coords[0].y;   
        int coord2_x = road.coords[1].x;
        int coord2_y = road.coords[1].y;
        printf("coord1_x: %d\n", coord1_x);
        printf("coord1_y: %d\n", coord1_y);
        printf("coord2_x: %d\n", coord2_x);
        printf("coord2_y: %d\n", coord2_y);
        printf("solution->timings[i].timing[step]: %d\n", solution.timings[i][step]);
        if (solution.timings[i][step] == 1) {
            printf("building road\n");
            double length = calculate_distance_3D(road.coords[0], road.coords[1], soil_amount) / 2;
            printf("length: %f\n", length);
            for (int j = 0; j < 2; j++) {
                printf("current_temps.statuslist[%d].status[0]: %d\n", i,  current_temps->statuslist[i].status[0]);
                printf("current_temps.statuslist[%d].status[1]: %d\n", i, current_temps->statuslist[i].status[1]);
                if (current_temps->statuslist[i].status[0] == 0) {
                    *built_length += length;
                    built_length_list[i] += length;
                    current_temps->statuslist[i].status[0] = 1;
                    printf("built_length: %f\n", *built_length);
                    printf("current_temps.statuslist[%d].status[0] after: %d\n", i, current_temps->statuslist[i].status[0]);

                }
                if (current_temps->statuslist[i].status[1] == 0) {
                    *built_length += length;
                    built_length_list[i] += length;
                    current_temps->statuslist[i].status[1] = 1;
                    printf("built_length: %f\n", *built_length);
                    printf("current_temps.statuslist[%d].status[1] after: %d\n", i, current_temps->statuslist[i].status[1]);
                }
            }
        }
        printf("\n");
    }
    
}

double heuristic(Coord a, Coord b, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    return TEMP_EFF * calculate_distance_3D(a, b, soil_amount);
}

// ユーティリティ関数: 2つのCoordPairを比較する
int compare_search(const void *a, const void *b) {
    CoordPair *pair1 = (CoordPair *)a;
    CoordPair *pair2 = (CoordPair *)b;

    // 座標ペアを比較 (x座標 → y座標の順)
    if (pair1->coords[0].x != pair2->coords[0].x)
        return pair1->coords[0].x - pair2->coords[0].x;
    if (pair1->coords[0].y != pair2->coords[0].y)
        return pair1->coords[0].y - pair2->coords[0].y;
    if (pair1->coords[1].x != pair2->coords[1].x)
        return pair1->coords[1].x - pair2->coords[1].x;
    return pair1->coords[1].y - pair2->coords[1].y;
}

double get_cost(Coord current, Coord neighbor, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], 
                TemporaryRoads temporary_roads) {
    double distance = calculate_distance_3D(current, neighbor, soil_amount);
    double current_distance = distance / 2;
    double neighbor_distance = distance / 2;

    // 検索対象ペアを作成&ソート
    CoordPair search_pair;
    if (current.x < neighbor.x || (current.x == neighbor.x && current.y < neighbor.y)) {
        search_pair.coords[0] = current;
        search_pair.coords[1] = neighbor;
    } else {
        search_pair.coords[0] = neighbor;
        search_pair.coords[1] = current;
    }

    qsort(temporary_roads.coordpair, temporary_roads.count, sizeof(CoordPair), compare_search);
    // for (int i = 0; i < temporary_roads.count; i++) {
    //     // printf("temporary_roads.coordpair[%d]: (%d,%d),(%d,%d)\n", i, temporary_roads.coordpair[i].coords[0].x, temporary_roads.coordpair[i].coords[0].y, temporary_roads.coordpair[i].coords[1].x, temporary_roads.coordpair[i].coords[1].y);
    // }
    // bsearch を使ってペアを検索
   CoordPair *found = bsearch(&search_pair, temporary_roads.coordpair, 
                                            temporary_roads.count, sizeof(CoordPair), 
                                            compare_search);

    if (found != NULL) {
        // ペアが見つかった場合、インデックスを取得
        int index = found - temporary_roads.coordpair;
        // printf("index: %d\n", index);

        // 仮設道路の状態を確認
        if (temporary_roads.statuslist[index].status[0] == 1) current_distance *= TEMP_EFF;
        if (temporary_roads.statuslist[index].status[1] == 1) neighbor_distance *= TEMP_EFF;
    }

    return current_distance + neighbor_distance;
}

// bool has_used_road(Coord *path, int path_length, TemporaryRoads temp) {
//     for (int i = 0; i < path_length - 1; i++) {
//         Coord current = path[i];
//         Coord next = path[i + 1];
//         if ((current.x == temp.coords[0].x && current.y == temp.coords[0].y &&
//              next.x == temp.coords[1].x && next.y == temp.coords[1].y) ||
//             (current.x == temp.coords[1].x && current.y == temp.coords[1].y &&
//              next.x == temp.coords[0].x && next.y == temp.coords[0].y)) {
//             return true;
//         }
//     }
//     return false;
// }

// 仮設道路のリストから、スタート地点またはゴール地点にある仮設道路を削除
void remove_temporary_roads(Allocation allocation, TemporaryRoads *temps) {
    int road_count = temps->count;
    int start_x = allocation.start.x, start_y = allocation.start.y;
    int goal_x = allocation.goal.x, goal_y = allocation.goal.y;
    for (int i = 0; i < road_count; i++) {
        Coord *coords = temps->coordpair[i].coords;
        int *status = temps->statuslist[i].status;

        // スタート地点またはゴール地点に接続する場合
        if ((coords[0].x == start_x && coords[0].y == start_y) ||
            (coords[0].x == goal_x && coords[0].y == goal_y)) {
            status[0] = 0; // 接続解除
        }

        if ((coords[1].x == start_x && coords[1].y == start_y) ||
            (coords[1].x == goal_x && coords[1].y == goal_y)) {
            status[1] = 0; // 接続解除
        }
    }
}

void change_soil(Allocation allocation, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    soil_amount[allocation.start.x][allocation.start.y] -= allocation.volume;
    soil_amount[allocation.goal.x][allocation.goal.y] += allocation.volume;
}

Coord *resize_path(Coord *path, int path_length) {
    Coord *resized_path = (Coord *)realloc(path, path_length * sizeof(Coord));
    if (resized_path == NULL) {
        fprintf(stderr, "Failed to resize path memory.\n");
        free(path);
        return NULL;
    }
    return resized_path;
}


// A*アルゴリズム本体
void astar(Allocation allocation, TemporaryRoads temps, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
           int used_temp_list[], double *total_cost, Coord path[]) {
    int temp_count = temps.count;
    Coord start = allocation.start;
    Coord goal = allocation.goal;

    // オープンリスト
    Coord open_set[GRID_SIZE_X * GRID_SIZE_Y];
    double open_set_costs[GRID_SIZE_X * GRID_SIZE_Y];
    int open_set_size = 0;

    // コスト追跡用
    double cost_so_far[GRID_SIZE_X][GRID_SIZE_Y];
    Coord came_from[GRID_SIZE_X][GRID_SIZE_Y];

    // 初期化
    for (int i = 0; i < GRID_SIZE_X; i++) {
        for (int j = 0; j < GRID_SIZE_Y; j++) {
            cost_so_far[i][j] = DBL_MAX;
        }
    }
    cost_so_far[start.x][start.y] = 0;

    open_set[open_set_size] = start;
    open_set_costs[open_set_size] = 0;
    open_set_size++;

    // メインループ
    while (open_set_size > 0) {
        // 最小コストのノードを探索
        int best_index = 0;
        for (int i = 1; i < open_set_size; i++) {
            if (open_set_costs[i] < open_set_costs[best_index]) {
                best_index = i;
            }
        }

        Coord current = open_set[best_index];
        open_set_size--;
        for (int i = best_index; i < open_set_size; i++) {
            open_set[i] = open_set[i + 1];
            open_set_costs[i] = open_set_costs[i + 1];
        }

        if (current.x == goal.x && current.y == goal.y) {
            break;
        }

        for (int d = 0; d < DIRECTIONS_COUNT; d++) {
            Coord neighbor = {current.x + DIRECTIONS[d].x, current.y + DIRECTIONS[d].y};

            // 範囲外をスキップ
            if (neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= GRID_SIZE_X || neighbor.y >= GRID_SIZE_Y) {
                continue;
            }

            double new_cost = cost_so_far[current.x][current.y] +
                              get_cost(current, neighbor, soil_amount, temps);
            if (new_cost < cost_so_far[neighbor.x][neighbor.y]) {
                cost_so_far[neighbor.x][neighbor.y] = new_cost;
                open_set[open_set_size] = neighbor;
                open_set_costs[open_set_size] = new_cost + heuristic(goal, neighbor, soil_amount);
                open_set_size++;

                came_from[neighbor.x][neighbor.y] = current;
            }
        }
    }

    // 経路再構築
    Coord current = goal;
    int path_length = 0;
    while (current.x != start.x || current.y != start.y) {
        path[(path_length)++] = current;
        current = came_from[current.x][current.y];
    }
    path[(path_length)++] = start;

    // 経路をリサイズ
    path = resize_path(path, path_length);

    // コスト計算
    *total_cost += cost_so_far[goal.x][goal.y];
}


// double process_allocations(
//     Allocation allocations[],
//     Solution solution,
//     int temp_count,
//     double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
//     int used_temp_list[][],
//     int built_length_list[],
//     Coord path[]
// ) {
//     //仮設道路の初期化
//     TemporaryRoads temps;
//     temps.count = solution.count;
//     temps.coordpair = (CoordPair *)malloc(sizeof(CoordPair) * temps.count);
//     temps.statuslist = (Status *)malloc(sizeof(Status) * temps.count);
//     for (int i = 0; i < temps.count; i++) {
//         temps.coordpair[i] = solution.coordpair[i];
//         temps.statuslist[i] = (Status){{0, 0}};
//     }

//     int step = solution.step;
//     double total_cost = 0;
//     for (int i = 0; i < step; i++) {
//         double cost;
//         temp_in_step(&temps, i, solution, built_length_list, soil_amount, &cost);
//         astar(allocations[i], temps, temp_count, soil_amount, &cost,path, &built_length_list[i]);
//         total_cost += cost;
//         // 仮設道路の削除
//         remove_temporary_roads(allocations[i],&temps);

//         // 土量の変更
//         change_soil(allocations[i],soil_amount);
//     }

//     return total_cost;
// }

int main() {
    // 必要な変数を初期化して、A*アルゴリズムをテスト
    printf("test\n");
    Allocation allocations[3] = {
        {{0, 0}, {3, 3}, 5.0},
        {{2, 2}, {0, 1}, 3.0},
        {{1, 1}, {3, 0}, 2.0}
    };
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y] ={
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {2, 2, 2, 2},
        {2, 2, 2, 2}
        };
    CoordPair coordpair1 = (CoordPair){{{0, 0}, {1, 1}}};
    CoordPair coordpair2 = (CoordPair){{{2, 2}, {3, 3}}};
    CoordPair coordpair3 = (CoordPair){{{1, 0}, {0, 1}}};
    coordpair1 = normalize_pair(coordpair1.coords[0], coordpair1.coords[1]);
    coordpair2 = normalize_pair(coordpair2.coords[0], coordpair2.coords[1]);
    coordpair3 = normalize_pair(coordpair3.coords[0], coordpair3.coords[1]);

    // solutionの初期化
    Solution solution;
    solution.count = 3;
    solution.step = 3;
    solution.coordpair = (CoordPair *)malloc(sizeof(CoordPair) * solution.count);
    solution.timings = (int **)malloc(sizeof(int*) * solution.count);
    for (int i = 0; i < solution.count; i++) {
        solution.timings[i] = (int *)malloc(sizeof(int) * solution.step);
    }
    solution.coordpair[0] = coordpair1;
    solution.coordpair[1] = coordpair2;
    solution.coordpair[2] = coordpair3;
    solution.timings[0] = (int[]){0, 0, 1};
    solution.timings[1] = (int[]){0, 1, 1};
    solution.timings[2] = (int[]){1, 1, 0};

    // ソート
    sort_solution(&solution);

    printf("\nAfter sorting:\n");
    for (int i = 0; i < solution.count; i++) {
        printf("solution.coordpair[%d]: (%d,%d),(%d,%d)\n", i, solution.coordpair[i].coords[0].x, solution.coordpair[i].coords[0].y, solution.coordpair[i].coords[1].x, solution.coordpair[i].coords[1].y);
        for (int j = 0; j < solution.step; j++) {
            printf("  timing[%d]: %d\n", j, solution.timings[i][j]);
        }
    }

    // current_tempsの初期化
    TemporaryRoads current_temps;
    current_temps.count = 3;
    current_temps.coordpair = (CoordPair *)malloc(sizeof(CoordPair) * current_temps.count);
    current_temps.statuslist = (Status *)malloc(sizeof(Status) * current_temps.count);

    current_temps.coordpair[0] = coordpair1;
    current_temps.coordpair[1] = coordpair2;
    current_temps.coordpair[2] = coordpair3;
    // qsort(current_temps.coordpair, current_temps.count, sizeof(CoordPair), compare_search);
    current_temps.statuslist[0] = (Status){{0, 1}};
    current_temps.statuslist[1] = (Status){{0, 1}};
    current_temps.statuslist[2] = (Status){{0, 1}};

    sort_temporary_roads(&current_temps);

    printf("\nAfter sorting:\n");
    for(int i = 0; i < current_temps.count; i++) {
        printf("current_temps.coordpair[%d]: (%d,%d),(%d,%d)\n", i, current_temps.coordpair[i].coords[0].x, current_temps.coordpair[i].coords[0].y, current_temps.coordpair[i].coords[1].x, current_temps.coordpair[i].coords[1].y);
        printf("current_temps.status[%d]: (%d,%d)\n", i, current_temps.statuslist[i].status[0], current_temps.statuslist[i].status[1]);
    }


    double built_length_list[2] = {0.0, 0.0};
    double built_length = 0.0;
    remove_temporary_roads(allocations[0], &current_temps);
    //current_tempsを出力
    printf("after remove_temporary_roads\n");
    for (int i = 0; i < current_temps.count; i++) {
        printf("current_temps.coordpair[%d]: (%d,%d),(%d,%d)\n", i, current_temps.coordpair[i].coords[0].x, current_temps.coordpair[i].coords[0].y, current_temps.coordpair[i].coords[1].x, current_temps.coordpair[i].coords[1].y);
        printf("current_temps.status[%d]: (%d,%d)\n", i, current_temps.statuslist[i].status[0], current_temps.statuslist[i].status[1]);
    }
    for (int i = 0; i < solution.step; i++) {
        change_soil(allocations[i], soil_amount);
    }
    printf("after change_soil\n");
    //soil_amountを出力
    for (int i = 0; i < GRID_SIZE_X; i++) {
        for (int j = 0; j < GRID_SIZE_Y; j++) {
            printf("soil_amount[%d][%d]: %f\n", i, j, soil_amount[i][j]);
        }
    }

    int used_temp_list[solution.step][solution.count] ;
    for (int i = 0; i < solution.count; i++) {
        for (int j = 0; j < solution.step; j++) {
            used_temp_list[i][j] = 0;
        }
    }

    Coord *path = (Coord *)malloc(GRID_SIZE_X * GRID_SIZE_Y * sizeof(Coord));
      if (path == NULL) {
        fprintf(stderr, "Memory allocation failed for path.\n");
        return 1;
    }

    double cost = 0;
    astar(allocations[0], current_temps, soil_amount,used_temp_list[0], &cost, path );

    printf("cost: %f\n", cost);
    for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; i++) {
        printf("path[%d]: (%d,%d)\n", i, path[i].x, path[i].y);
    }

    // double ans1 = calculate_distance_3D(start, goal, soil_amount);

    // printf("ans1: %f\n", ans1);
    // for (int i = 0; i < solution.step; i++) {
    //     printf("step: %d\n", i);
    //     temp_in_step(&current_temps, i, solution, built_length_list, soil_amount, &built_length);
    //     for(int j = 0; j < current_temps.count; j++) {
    //         printf("current_temps.status[%d]: %d,%d\n", j, current_temps.statuslist[j].status[0], current_temps.statuslist[j].status[1]);
    //     }

    // }
    // for (int i = 0; i < current_temps.count; i++) {
    //     printf("built_length_list[%d]: %f\n", i, built_length_list[i]);
    //     printf("current_temps.status[%d]: %d,%d\n", i, current_temps.statuslist[i].status[0], current_temps.statuslist[i].status[1]);
    // }
    // Coord current = {0, 0};
    // Coord neighbor = {1, 1};

    // double cost = get_cost(current, neighbor, soil_amount, current_temps);
    // printf("Cost: %f\n", cost);

    // Coord current2 = {0, 1};
    // Coord neighbor2 = {1, 0};
    // double cost2 = get_cost(current2, neighbor2, soil_amount, current_temps);
    // printf("Cost2: %f\n", cost2);

    // Coord current3 = {0, 0};
    // Coord neighbor3 = {0, 1};
    // double cost3 = get_cost(current3, neighbor3, soil_amount, current_temps);
    // printf("Cost3: %f\n", cost3);
    // メモリ解放
    free(current_temps.coordpair);
    free(current_temps.statuslist);
    free(solution.coordpair);
    for (int i = 0; i < solution.count; i++) {
        free(solution.timings[i]);
    }
    free(path);

    return 0;
}


