#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>
#include <memory>

#define GRID_SIZE_X 4
#define GRID_SIZE_Y 4
#define STEPS 10
#define TEMP_EFF 0.5 // Temporary road efficiency
#define GRID_SIZE 1
#define DIRECTIONS_COUNT 8
#define INF std::numeric_limits<double>::infinity()

struct Coord {
    int x, y;
};

struct CoordPair {
    Coord coords[2];
};

struct Status {
    int status[2];
};

struct TemporaryRoads {
    int count;
    std::vector<CoordPair> coordpair;
    std::vector<Status> statuslist;
};

struct Solution {
    int count;
    int step;
    std::vector<CoordPair> coordpair;
    std::vector<std::vector<int>> timings;
};


struct Allocation {
    Coord start;
    Coord goal;
    double volume;
};

struct CoordPairWithTiming {
    CoordPair coordpair;
    std::vector<int> timing;
};

struct CoordPairWithStatus {
    CoordPair coordpair;
    Status status;
};

// CoordPair normalize_pair(const Coord& a, const Coord& b) {
//     CoordPair pair;
//     if (a.x < b.x || (a.x == b.x && a.y < b.y)) {
//         pair.coords[0] = a;
//         pair.coords[1] = b;
//     } else {
//         pair.coords[0] = b;
//         pair.coords[1] = a;
//     }
//     return pair;
// }

double calculate_distance_3D(const Coord& a, const Coord& b, const std::vector<std::vector<double>>& soil_amount) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = (soil_amount[a.x][a.y] - soil_amount[b.x][b.y]) / (GRID_SIZE * GRID_SIZE * GRID_SIZE);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// void sort_solution(Solution& solution) {
//     std::vector<CoordPairWithTiming> entries;
//     for (size_t i = 0; i < solution.coordpair.size(); ++i) {
//         entries.push_back({solution.coordpair[i], solution.timings[i]});
//     }

//     std::sort(entries.begin(), entries.end(), [](const CoordPairWithTiming& a, const CoordPairWithTiming& b) {
//         return std::tie(a.coordpair.coords[0].x, a.coordpair.coords[0].y, a.coordpair.coords[1].x, a.coordpair.coords[1].y) <
//                std::tie(b.coordpair.coords[0].x, b.coordpair.coords[0].y, b.coordpair.coords[1].x, b.coordpair.coords[1].y);
//     });

//     for (size_t i = 0; i < entries.size(); ++i) {
//         solution.coordpair[i] = entries[i].coordpair;
//         solution.timings[i] = entries[i].timing;
//     }
// }

// void sort_temporary_roads(TemporaryRoads& temporary_roads) {
//     std::vector<CoordPairWithStatus> entries;
//     for (size_t i = 0; i < temporary_roads.coordpair.size(); ++i) {
//         entries.push_back({temporary_roads.coordpair[i], temporary_roads.statuslist[i]});
//     }

//     std::sort(entries.begin(), entries.end(), [](const CoordPairWithStatus& a, const CoordPairWithStatus& b) {
//         return std::tie(a.coordpair.coords[0].x, a.coordpair.coords[0].y, a.coordpair.coords[1].x, a.coordpair.coords[1].y) <
//                std::tie(b.coordpair.coords[0].x, b.coordpair.coords[0].y, b.coordpair.coords[1].x, b.coordpair.coords[1].y);
//     });

//     for (size_t i = 0; i < entries.size(); ++i) {
//         temporary_roads.coordpair[i] = entries[i].coordpair;
//         temporary_roads.statuslist[i] = entries[i].status;
//     }
// }

// double get_cost(const Coord& current, const Coord& neighbor, const std::vector<std::vector<double>>& soil_amount, const TemporaryRoads& temporary_roads) {
//     double distance = calculate_distance_3D(current, neighbor, soil_amount);
//     double current_distance = distance / 2;
//     double neighbor_distance = distance / 2;

//     CoordPair search_pair = normalize_pair(current, neighbor);
//     auto it = std::find(temporary_roads.coordpair.begin(), temporary_roads.coordpair.end(), search_pair);

//     if (it != temporary_roads.coordpair.end()) {
//         size_t index = std::distance(temporary_roads.coordpair.begin(), it);
//         if (temporary_roads.statuslist[index].status[0] == 1) current_distance *= TEMP_EFF;
//         if (temporary_roads.statuslist[index].status[1] == 1) neighbor_distance *= TEMP_EFF;
//     }

//     return current_distance + neighbor_distance;
// }

void remove_temporary_roads(const Allocation& allocation, TemporaryRoads& temps) {
    int start_x = allocation.start.x, start_y = allocation.start.y;
    int goal_x = allocation.goal.x, goal_y = allocation.goal.y;

    for (size_t i = 0; i < temps.coordpair.size(); ++i) {
        auto& coords = temps.coordpair[i].coords;
        auto& status = temps.statuslist[i].status;

        if ((coords[0].x == start_x && coords[0].y == start_y) ||
            (coords[0].x == goal_x && coords[0].y == goal_y)) {
            status[0] = 0;
        }

        if ((coords[1].x == start_x && coords[1].y == start_y) ||
            (coords[1].x == goal_x && coords[1].y == goal_y)) {
            status[1] = 0;
        }
    }
}


struct Node {
    int priority;
    Coord coord;

    // 比較演算子で優先度を定義（小さい値が高優先度）
    bool operator>(const Node& other) const {
        return priority > other.priority;
    }
};

struct Path {
    std::vector<Coord> coord;

    void resize(size_t new_size) {
        coord.resize(new_size);
    }
    size_t size() const {
        return coord.size();
    }
};

double heuristic(const Coord& a, const Coord& b, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y); // 例: マンハッタン距離
}


void change_soil(const Allocation& allocation, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    soil_amount[allocation.start.x][allocation.start.y] -= allocation.volume;
    soil_amount[allocation.goal.x][allocation.goal.y] += allocation.volume;
}

// void astar(const Allocation& allocation, const TemporaryRoads& temps, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
//            std::vector<int>& used_temp_list, double& total_cost, Path& path) {
//     Coord start = allocation.start;
//     Coord goal = allocation.goal;

//     std::priority_queue<Node, std::vector<Node>, std::greater<>> open_set;
//     open_set.push({0, start});

//     double cost_so_far[GRID_SIZE_X][GRID_SIZE_Y];
//     std::fill(&cost_so_far[0][0], &cost_so_far[0][0] + GRID_SIZE_X * GRID_SIZE_Y, INF);
//     cost_so_far[start.x][start.y] = 0;

//     Coord came_from[GRID_SIZE_X][GRID_SIZE_Y];
//     int directions[8][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

//     while (!open_set.empty()) {
//         Node current_node = open_set.top();
//         open_set.pop();
//         Coord current = current_node.coord;

//         if (current.x == goal.x && current.y == goal.y) break;

//         for (const auto& dir : directions) {
//             Coord neighbor = {current.x + dir[0], current.y + dir[1]};

//             if (neighbor.x < 0 || neighbor.x >= GRID_SIZE_X || neighbor.y < 0 || neighbor.y >= GRID_SIZE_Y) continue;

//             double new_cost = cost_so_far[current.x][current.y] + get_cost(current, neighbor, soil_amount, temps);

//             if (new_cost < cost_so_far[neighbor.x][neighbor.y]) {
//                 cost_so_far[neighbor.x][neighbor.y] = new_cost;
//                 int priority = new_cost + heuristic(goal, neighbor, soil_amount);
//                 open_set.push({priority, neighbor});
//                 came_from[neighbor.x][neighbor.y] = current;
//             }
//         }
//     }

//     // 経路再構築
//     std::vector<Coord> reverse_path;
//     Coord current = goal;
//     while (!(current.x == start.x && current.y == start.y)) {
//         reverse_path.push_back(current);
//         Coord next = came_from[current.x][current.y];

//         // 仮設道路チェック
//         CoordPair search_pair = {std::min(current, next), std::max(current, next)};
//         auto it = std::lower_bound(temps.coordpair.begin(), temps.coordpair.end(), search_pair, [](const CoordPair& a, const CoordPair& b) {
//             return std::tie(a.coords[0].x, a.coords[0].y, a.coords[1].x, a.coords[1].y) <
//                    std::tie(b.coords[0].x, b.coords[0].y, b.coords[1].x, b.coords[1].y);
//         });

//         if (it != temps.coordpair.end() && *it == search_pair) {
//             size_t index = std::distance(temps.coordpair.begin(), it);
//             used_temp_list[index] = (temps.statuslist[index].status[0] == 1 && temps.statuslist[index].status[1] == 1) ? 2 : 1;
//         }

//         current = next;
//     }
//     reverse_path.push_back(start);

//     // 結果をPathに反映
//     path.coord.assign(reverse_path.rbegin(), reverse_path.rend());
//     total_cost += cost_so_far[goal.x][goal.y];

//     // 経路出力
//     std::cout << "Path:\n";
//     for (const auto& coord : path.coord) {
//         std::cout << "(" << coord.x << ", " << coord.y << ")\n";
//     }
// }


void temp_in_step(
    TemporaryRoads* current_temps, 
    int step, 
    const Solution& solution, 
    std::vector<double>& built_length_list, 
    double soil_amount[4][4], 
    double& built_length
) {
    int road_count = current_temps->count;

    for (int i = 0; i < road_count; i++) {
        CoordPair road = solution.coordpair[i];
        int coord1_x = road.coords[0].x;
        int coord1_y = road.coords[0].y;   
        int coord2_x = road.coords[1].x;
        int coord2_y = road.coords[1].y;

        if (solution.timings[i][step] == 1) {
            double length = calculate_distance_3D(road.coords[0], road.coords[1], soil_amount) / 2;

            for (int j = 0; j < 2; j++) {
                if (current_temps->statuslist[i].status[0] == 0) {
                    built_length += length;
                    built_length_list[i] += length;
                    current_temps->statuslist[i].status[0] = 1;
                }
                if (current_temps->statuslist[i].status[1] == 0) {
                    built_length += length;
                    built_length_list[i] += length;
                    current_temps->statuslist[i].status[1] = 1;
                }
            }
        }
    }
}

// int process_allocations(std::vector<Allocation>& allocations,
//                         Solution& solution,
//                         double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
//                         std::vector<double>& built_length_list,
//                         std::vector<std::vector<int>>& used_temp_list,
//                         std::vector<Path>& path,
//                         double* total_cost) {
//     // Initialize TemporaryRoads
//     TemporaryRoads temps;
//     temps.coordpair = solution.coordpair;
//     temps.statuslist.resize(solution.count, {{0, 0}});

//     int step = solution.step;
//     for (int i = 0; i < step; ++i) {
//         std::cout << "loop: " << i << std::endl;
//         double cost = 0;
//         temp_in_step(temps, i, solution, built_length_list, soil_amount, &cost);

//         // Print temps after temp_in_step
//         for (size_t j = 0; j < temps.coordpair.size(); ++j) {
//             std::cout << "temps.coordpair[" << j << "]: (" << temps.coordpair[j].coords[0].x << ", "
//                       << temps.coordpair[j].coords[0].y << "), (" << temps.coordpair[j].coords[1].x << ", "
//                       << temps.coordpair[j].coords[1].y << ")\n";
//             std::cout << "temps.statuslist[" << j << "]: (" << temps.statuslist[j].status[0] << ", "
//                       << temps.statuslist[j].status[1] << ")\n";
//         }

//         astar(allocations[i], temps, soil_amount, used_temp_list[i], &cost, path[i]);

//         // Print path after astar
//         for (size_t j = 0; j < path[i].coord.size(); ++j) {
//             std::cout << "path[" << j << "]: (" << path[i].coord[j].x << ", " << path[i].coord[j].y << ")\n";
//         }

//         // Print cost
//         std::cout << "cost: " << cost << std::endl;

//         *total_cost += cost;

//         // Remove temporary roads
//         remove_temporary_roads(allocations[i], &temps);

//         // Print temps after removing temporary roads
//         for (size_t j = 0; j < temps.coordpair.size(); ++j) {
//             std::cout << "temps.coordpair[" << j << "]: (" << temps.coordpair[j].coords[0].x << ", "
//                       << temps.coordpair[j].coords[0].y << "), (" << temps.coordpair[j].coords[1].x << ", "
//                       << temps.coordpair[j].coords[1].y << ")\n";
//             std::cout << "temps.statuslist[" << j << "]: (" << temps.statuslist[j].status[0] << ", "
//                       << temps.statuslist[j].status[1] << ")\n";
//         }

//         // Change soil
//         change_soil(allocations[i], soil_amount);

//         // Print soil_amount
//         for (int j = 0; j < GRID_SIZE_X; ++j) {
//             for (int k = 0; k < GRID_SIZE_Y; ++k) {
//                 std::cout << "soil_amount[" << j << "][" << k << "]: " << soil_amount[j][k] << "\n";
//             }
//         }
//     }

//     return 0;
// }

int main() {
    std::cout << "test" << std::endl;

    // Initialize allocations
    std::vector<Allocation> allocations = {
        {{0, 0}, {3, 3}, 5.0},
        {{2, 2}, {0, 1}, 3.0},
        {{1, 1}, {3, 0}, 2.0}};

    // Initialize soil_amount
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y] = {};

    // Initialize solution
    Solution solution;
    solution.count = 3;
    solution.step = 3;
    solution.coordpair = {{{{0, 0}, {1, 1}}}, {{{2, 2}, {3, 3}}}, {{{1, 0}, {0, 1}}}};
    solution.timings = {{1, 0, 1}, {0, 1, 1}, {0, 1, 0}};

    sort_solution(solution);

    // Print sorted solution
    for (int i = 0; i < solution.count; ++i) {
        std::cout << "solution.coordpair[" << i << "]: (" << solution.coordpair[i].coords[0].x << ", "
                  << solution.coordpair[i].coords[0].y << "), (" << solution.coordpair[i].coords[1].x << ", "
                  << solution.coordpair[i].coords[1].y << ")\n";
        for (int j = 0; j < solution.step; ++j) {
            std::cout << "  timing[" << j << "]: " << solution.timings[i][j] << "\n";
        }
    }

    // Initialize path and used_temp_list
    std::vector<Path> path(solution.step, Path{std::vector<Coord>()});
    std::vector<std::vector<int>> used_temp_list(solution.step, std::vector<int>(solution.count, 0));

    double total_cost = 0;
    std::vector<double> built_length_list(solution.count, 0.0);

    process_allocations(allocations, solution, soil_amount, built_length_list, used_temp_list, path, &total_cost);

    std::cout << "total_cost: " << total_cost << std::endl;

    return 0;
}
