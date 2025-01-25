#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <limits.h>

#define GRID_SIZE_X 10
#define GRID_SIZE_Y 10
#define DIRECTIONS 4
#define STEPS 10
#define TEMP_EFF 0.5 // 仮設道路の効果効率
#define grid_size 150

typedef struct {
    int x, y;
} Coord;

typedef struct {
    Coord coords[2];
} CoordPair;

typedef struct {
    CoordPair coordpair;
    int status[2];
} TemporaryRoad_part;

typedef struct {
    int size;
   TemporaryRoad_part *road_list;
} TemporaryRoad;

typedef struct {
    int size;
    CoordPair *coordpair;
    int *timing;
} Solution;

typedef struct {
    Coord start;
    Coord goal;
    double volume;
} Allocation;

double calculate_distance_3D(Coord a, Coord b, double soil_amount) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = soil_amount/(grid_size*grid_size*grid_size);
    return sqrt(dx * dx + dy * dy + dz * dz);
}

void temp_in_step(TemporaryRoad *current_temps, int step, Solution *solution, double *built_length_list, double soil_amount_local, int road_count, double *built_length) {
    *built_length = 0.0;
    for (int i = 0; i < road_count; i++) {
        TemporaryRoad road = solution->coordpair[i];
        if (road.status[step] == 1) {
            for (int j = 0; j < 2; j++) {
                if (current_temps[i].status[j] == 0) {
                    double length = calculate_distance_3D(road.coords[0], road.coords[1], soil_amount_local) / 2;
                    *built_length += length;
                    built_length_list[i] += length;
                    current_temps[i].status[j] = 1;
                }
            }
        }
    }
}

double heuristic(Coord a, Coord b, double soil_amount) {
    return TEMP_EFF * calculate_distance_3D(a, b, soil_amount);
}

double get_cost(Coord current, Coord neighbor, double soil_amount, TemporaryRoad *temporary_roads, int road_count) {
    double distance = calculate_distance_3D(current, neighbor, soil_amount);
    double current_distance = distance / 2;
    double neighbor_distance = distance / 2;

    for (int i = 0; i < road_count; i++) {
        TemporaryRoad temp = temporary_roads[i];
        if ((current.x == temp.coords[0].x && current.y == temp.coords[0].y &&
             neighbor.x == temp.coords[1].x && neighbor.y == temp.coords[1].y) ||
            (current.x == temp.coords[1].x && current.y == temp.coords[1].y &&
             neighbor.x == temp.coords[0].x && neighbor.y == temp.coords[0].y)) {
            if (temp.status[0] == 1) current_distance *= TEMP_EFF;
            if (temp.status[1] == 1) neighbor_distance *= TEMP_EFF;
            break;
        }
    }

    return current_distance + neighbor_distance;
}

bool has_used_road(Coord *path, int path_length, TemporaryRoad temp) {
    for (int i = 0; i < path_length - 1; i++) {
        Coord current = path[i];
        Coord next = path[i + 1];
        if ((current.x == temp.coords[0].x && current.y == temp.coords[0].y &&
             next.x == temp.coords[1].x && next.y == temp.coords[1].y) ||
            (current.x == temp.coords[1].x && current.y == temp.coords[1].y &&
             next.x == temp.coords[0].x && next.y == temp.coords[0].y)) {
            return true;
        }
    }
    return false;
}

void remove_temporary_roads(Coord start, Coord goal, TemporaryRoad *temps, int road_count) {
    for (int i = 0; i < road_count; i++) {
        for (int j = 0; j < 2; j++) {
            if ((temps[i].coords[j].x == start.x && temps[i].coords[j].y == start.y) ||
                (temps[i].coords[j].x == goal.x && temps[i].coords[j].y == goal.y)) {
                temps[i].status[j] = 0;
            }
        }
    }
}

void change_soil(Coord start, Coord goal, double volume, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    soil_amount[start.x][start.y] -= volume;
    soil_amount[goal.x][goal.y] += volume;
}

double astar(Allocation allocation, TemporaryRoad *temp, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], int road_count, Coord *path, int *path_length) {
    Coord start = allocation.start;
    Coord goal = allocation.goal;
    double volume = allocation.volume;

    bool visited[GRID_SIZE_X][GRID_SIZE_Y] = {false};
    double cost_so_far[GRID_SIZE_X][GRID_SIZE_Y];
    Coord came_from[GRID_SIZE_X][GRID_SIZE_Y];

    for (int i = 0; i < GRID_SIZE_X; i++) {
        for (int j = 0; j < GRID_SIZE_Y; j++) {
            cost_so_far[i][j] = INT_MAX;
        }
    }
    cost_so_far[start.x][start.y] = 0;

    // Priority queue (simplified)
    Coord open_set[GRID_SIZE_X * GRID_SIZE_Y];
    int open_set_size = 0;
    open_set[open_set_size++] = start;

    while (open_set_size > 0) {
        Coord current = open_set[--open_set_size];
        if (current.x == goal.x && current.y == goal.y) break;

        visited[current.x][current.y] = true;

        for (int d = 0; d < DIRECTIONS; d++) {
            Coord neighbor = {current.x + d % 2, current.y + d / 2};
            if (neighbor.x < 0 || neighbor.x >= GRID_SIZE_X || neighbor.y < 0 || neighbor.y >= GRID_SIZE_Y || visited[neighbor.x][neighbor.y]) continue;

            double new_cost = cost_so_far[current.x][current.y] + get_cost(current, neighbor, soil_amount[current.x][current.y], temp, road_count);
            if (new_cost < cost_so_far[neighbor.x][neighbor.y]) {
                cost_so_far[neighbor.x][neighbor.y] = new_cost;
                came_from[neighbor.x][neighbor.y] = current;
                open_set[open_set_size++] = neighbor;
            }
        }
    }

    // Reconstruct path
    *path_length = 0;
    Coord current = goal;
    while (current.x != start.x || current.y != start.y) {
        path[(*path_length)++] = current;
        current = came_from[current.x][current.y];
    }
    path[(*path_length)++] = start;

    return cost_so_far[goal.x][goal.y];
}

int main() {
    // 必要な変数を初期化して、A*アルゴリズムをテスト
    return 0;
}
