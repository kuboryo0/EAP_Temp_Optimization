#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_map>
#include <memory>
#include <cstdlib> // for rand()
#include <random> // for std::discrete_distribution
#include <ctime> 
#include <chrono>

#define GRID_SIZE_X 12
#define GRID_SIZE_Y 4
#define STEPS 10
#define TEMP_EFF 0.5 // Temporary road efficiency
#define GRID_SIZE 150
#define V_truck 30.0
#define TRUCK_NUM 8
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
    bool operator==(const Status& other) const {
    return status[0] == other.status[0] && status[1] == other.status[1];
    }
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
    std::vector<std::vector<int>> timings;  //timings[road][step]
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

struct Node {
    int priority;
    Coord coord;
    // 比較演算子で優先度を定義（小さい値が高優先度）
    bool operator>(const Node& other) const {
        return priority > other.priority;
    }
};

struct State {
    std::vector<Status> roadStatus;
    int step;

    bool operator==(const State& other) const {
        return roadStatus == other.roadStatus && step == other.step;
    }
};

struct Path {
    size_t new_size;
    std::vector<Coord> coord;

    void resize() {
        coord.resize(new_size);
    }
    size_t size() const {
        return coord.size();
    }
};

struct Result {
    int size;  // Number of Steps
    std::vector<Path> path;
    std::vector<std::vector<int>> used_road_flow;  //used_temp_list[step][road]
    std::vector<double> built_length_list;
};


// Define hash function for Status
namespace std {
    template <>
    struct hash<Status> {
        size_t operator()(const Status& s) const {
            return hash<int>()(s.status[0]) ^ (hash<int>()(s.status[1]) << 1);
        }
    };

    template <>
    struct hash<std::vector<Status>> {
        size_t operator()(const std::vector<Status>& v) const {
            size_t seed = 0;
            for (const auto& s : v) {
                seed ^= hash<Status>()(s) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    template <>
    struct hash<std::tuple<std::vector<Status>, int>> {
        size_t operator()(const std::tuple<std::vector<Status>, int>& t) const {
            const auto& [statuses, step] = t;
            return hash<std::vector<Status>>()(statuses) ^ (hash<int>()(step) << 1);
        }
    };
}


double calculate_distance_3D(const Coord& a, const Coord& b, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = (soil_amount[a.x][a.y] - soil_amount[b.x][b.y]) / (GRID_SIZE * GRID_SIZE * GRID_SIZE);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}



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




double heuristic(const Coord& a, const Coord& b, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    return TEMP_EFF * calculate_distance_3D(a, b, soil_amount);
}


void change_soil(const Allocation& allocation, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    soil_amount[allocation.start.x][allocation.start.y] -= allocation.volume;
    soil_amount[allocation.goal.x][allocation.goal.y] += allocation.volume;
}


void temp_in_step(
    TemporaryRoads& current_temps, 
    const int step, 
    const Solution& solution, 
    std::vector<double>& built_length_list, 
    const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], 
    double& built_length
) {
    int road_count = current_temps.count;
    for (int i = 0; i < road_count; i++) {
        // std::cout << "i: " << i << std::endl;
        CoordPair road = solution.coordpair[i];
        int coord1_x = road.coords[0].x;
        int coord1_y = road.coords[0].y;   
        int coord2_x = road.coords[1].x;
        int coord2_y = road.coords[1].y;
        // std::cout << "solution.timings[" << i << "][" << step << "]: " << solution.timings[i][step] << std::endl;
        if (solution.timings[i][step] == 1) {
            // std::cout << "solution.timings[" << i << "][" << step << "] == 1" << std::endl;
            double length = calculate_distance_3D(road.coords[0], road.coords[1], soil_amount) / 2;
            if (current_temps.statuslist[i].status[0] == 0) {
                // std::cout << "building  temporary road: (" << coord1_x << ", " << coord1_y << "), (" << coord2_x << ", " << coord2_y << ")" << std::endl;
                // std::cout <<"length = " << length << std::endl;
                built_length += length;
                built_length_list[i] += length;
                current_temps.statuslist[i].status[0] = 1;
            }
            if (current_temps.statuslist[i].status[1] == 0) {
                // std::cout << "building temporary road: (" << coord1_x << ", " << coord1_y << "), (" << coord2_x << ", " << coord2_y << ")" << std::endl;
                // std::cout <<"length = " << length << std::endl;
                built_length += length;
                built_length_list[i] += length;
                current_temps.statuslist[i].status[1] = 1;
            }
        }
    }
    // //print current_temps
    // for (int j = 0; j < current_temps.coordpair.size(); ++j) {
    //     std::cout << "current_temps.coordpair[" << j << "]: (" << current_temps.coordpair[j].coords[0].x << ", "
    //               << current_temps.coordpair[j].coords[0].y << "), (" << current_temps.coordpair[j].coords[1].x << ", "
    //               << current_temps.coordpair[j].coords[1].y << ")\n";
    //     std::cout << "current_temps.statuslist[" << j << "]: (" << current_temps.statuslist[j].status[0] << ", "
    //               << current_temps.statuslist[j].status[1] << ")\n";
    // }
    // std::cout << std::endl;
}

CoordPair normalize_pair(const Coord& a, const Coord& b) {
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

double get_cost(const Coord& current, const Coord& neighbor, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], const TemporaryRoads& temporary_roads) {
    double distance = calculate_distance_3D(current, neighbor, soil_amount);
    double current_distance = distance / 2;
    double neighbor_distance = distance / 2;

    CoordPair search_pair = normalize_pair(current, neighbor);
    // auto it = std::find(temporary_roads.coordpair.begin(), temporary_roads.coordpair.end(), search_pair);
    auto it = std::find_if(
        temporary_roads.coordpair.begin(),
        temporary_roads.coordpair.end(),
        [&search_pair](const CoordPair& pair) {
            return pair.coords[0].x == search_pair.coords[0].x &&
                   pair.coords[0].y == search_pair.coords[0].y &&
                   pair.coords[1].x == search_pair.coords[1].x &&
                   pair.coords[1].y == search_pair.coords[1].y;
        }
    );

    if (it != temporary_roads.coordpair.end()) {
        size_t index = std::distance(temporary_roads.coordpair.begin(), it);
        // std::cout << "index: " << index << std::endl;
        if (temporary_roads.statuslist[index].status[0] == 1) current_distance *= TEMP_EFF;
        if (temporary_roads.statuslist[index].status[1] == 1) neighbor_distance *= TEMP_EFF;
    }

    return current_distance + neighbor_distance;
}

void astar(const Allocation& allocation, const TemporaryRoads& temps, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
           std::vector<int>& used_temp_list, double& total_cost, Path& path) {
    Coord start = allocation.start;
    Coord goal = allocation.goal;
    double volume = allocation.volume;
    

    std::priority_queue<Node, std::vector<Node>, std::greater<>> open_set;
    open_set.push({0, start});

    double cost_so_far[GRID_SIZE_X][GRID_SIZE_Y];
    std::fill(&cost_so_far[0][0], &cost_so_far[0][0] + GRID_SIZE_X * GRID_SIZE_Y, INF);
    cost_so_far[start.x][start.y] = 0;

    Coord came_from[GRID_SIZE_X][GRID_SIZE_Y];
    int directions[8][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

    while (!open_set.empty()) {
        Node current_node = open_set.top();
        open_set.pop();
        Coord current = current_node.coord;
        if (current.x == goal.x && current.y == goal.y) break;
        for (const auto& dir : directions) {
            Coord neighbor = {current.x + dir[0], current.y + dir[1]};
            if (neighbor.x < 0 || neighbor.x >= GRID_SIZE_X || neighbor.y < 0 || neighbor.y >= GRID_SIZE_Y) continue;
            double new_cost = cost_so_far[current.x][current.y] + get_cost(current, neighbor, soil_amount, temps);

            if (new_cost < cost_so_far[neighbor.x][neighbor.y]) {
                cost_so_far[neighbor.x][neighbor.y] = new_cost;
                int priority = new_cost + heuristic(goal, neighbor, soil_amount);
                open_set.push({priority, neighbor});
                came_from[neighbor.x][neighbor.y] = current;
            }
        }
    }

    // 経路再構築
    std::vector<Coord> reverse_path;
    Coord current = goal;
    while (!(current.x == start.x && current.y == start.y)) {
        reverse_path.push_back(current);
        Coord next = came_from[current.x][current.y];

        // 仮設道路チェック
        CoordPair search_pair = normalize_pair(current, next);
        auto it = std::find_if(
            temps.coordpair.begin(),
            temps.coordpair.end(),
            [&search_pair](const CoordPair& pair) {
                return pair.coords[0].x == search_pair.coords[0].x &&
                       pair.coords[0].y == search_pair.coords[0].y &&
                       pair.coords[1].x == search_pair.coords[1].x &&
                       pair.coords[1].y == search_pair.coords[1].y;
            });

        if (it != temps.coordpair.end()) {
            size_t index = std::distance(temps.coordpair.begin(), it);
            // used_temp_list[index] = (temps.statuslist[index].status[0] == 1 && temps.statuslist[index].status[1] == 1) ? 2 : 1;
            if (temps.statuslist[index].status[0] == 1 && temps.statuslist[index].status[1] == 1) {
                used_temp_list[index] = 2;
            } else if 
                (temps.statuslist[index].status[0] == 1|| temps.statuslist[index].status[1] == 1) {
                used_temp_list[index] = 1;
            }
            else {
                used_temp_list[index] = 0;
            }
        }

        current = next;
    }
    reverse_path.push_back(start);

    // 結果をPathに反映
    path.coord.assign(reverse_path.rbegin(), reverse_path.rend());
    total_cost += cost_so_far[goal.x][goal.y] * volume/ V_truck / TRUCK_NUM; //往復回数込みのコスト
    // // 経路出力
    // std::cout << "Path:\n";
    // for (const auto& coord : path.coord) {
    //     std::cout << "(" << coord.x << ", " << coord.y << ")\n";
    // }
}



std::tuple<Result,double,double> process_allocations(std::vector<Allocation>& allocations,
                        const Solution& solution,
                        const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
                        ) {
    double local_soil_amount[GRID_SIZE_X][GRID_SIZE_Y] = {0};
    for (int i = 0; i < GRID_SIZE_X; i++) {
        for (int j = 0; j < GRID_SIZE_Y; j++) {
            local_soil_amount[i][j] = local_soil_amount[i][j];
        }
    }
    // Initialize TemporaryRoads
    TemporaryRoads temps;
    temps.count = solution.count;
    temps.coordpair = solution.coordpair;
    temps.statuslist.resize(solution.count, {{0, 0}});
    double operate_cost = 0;
    double built_length = 0;
    Result result;
    result.size = solution.step;
    result.path.resize(solution.step);
    result.used_road_flow.resize(solution.step);
    for (int i = 0; i < solution.step; i++) {
        result.used_road_flow[i].resize(solution.count);
        for (int j = 0; j < solution.count; j++) {
            result.used_road_flow[i][j] = 0;
        }
    }

    result.built_length_list.resize(solution.count, 0.0);

    int step = solution.step;
    //print soution
    // for (int i = 0; i < solution.count; ++i) {
    //     std::cout << "solution.coordpair[" << i << "]: (" << solution.coordpair[i].coords[0].x << ", "
    //               << solution.coordpair[i].coords[0].y << "), (" << solution.coordpair[i].coords[1].x << ", "
    //               << solution.coordpair[i].coords[1].y << ")\n";
    //     for (int j = 0; j < solution.step; ++j) {
    //         std::cout << "  timing[" << j << "]: " << solution.timings[i][j] << "\n";
    //     }
    // }
    std::cout << "start process_allocations" << std::endl;
    for (int i = 0; i < step; ++i) {
        std::cout << "loop: " << i << std::endl;
        // std::cout << "local_soil_amount" << std::endl;
        // for (int j = 0; j < GRID_SIZE_X; ++j) {
        //     for (int k = 0; k < GRID_SIZE_Y; ++k) {
        //         std::cout << local_soil_amount[j][k] << " ";
        //     }
        //     std::cout << std::endl;
        // }
        double cost = 0;
        double built_length_i = 0;
        temp_in_step(temps, i, solution, result.built_length_list, local_soil_amount, built_length_i);
        built_length += built_length_i;
        // std::cout << "built_length_i: " << built_length_i << std::endl;
        // // temp_in_step(current_temps, 0, solution, built_length_list, local_soil_amount, operate_cost);
        // // Print temps after temp_in_step
        // std::cout << "temps after temp_in_step" << std::endl;
        // for (size_t j = 0; j < temps.coordpair.size(); ++j) {
        //     std::cout << "temps.coordpair[" << j << "]: (" << temps.coordpair[j].coords[0].x << ", "
        //               << temps.coordpair[j].coords[0].y << "), (" << temps.coordpair[j].coords[1].x << ", "
        //               << temps.coordpair[j].coords[1].y << ")\n";
        //     std::cout << "temps.statuslist[" << j << "]: (" << temps.statuslist[j].status[0] << ", "
        //               << temps.statuslist[j].status[1] << ")\n";
        // }
        // std::cout << "built_length: " << built_length << std::endl;
        astar(allocations[i], temps, local_soil_amount, result.used_road_flow[i], cost, result.path[i]);
        // Print path after astar
        // for (size_t j = 0; j < path[i].coord.size(); ++j) {
        //     std::cout << "path[" << j << "]: (" << path[i].coord[j].x << ", " << path[i].coord[j].y << ")\n";
        // }
        // // Print cost
        // std::cout << "cost: " << cost << std::endl;
        operate_cost += cost;
        // Remove temporary roads
        remove_temporary_roads(allocations[i], temps);

        // Print temps after removing temporary roads
        // for (size_t j = 0; j < temps.coordpair.size(); ++j) {
        //     std::cout << "temps.coordpair[" << j << "]: (" << temps.coordpair[j].coords[0].x << ", "
        //               << temps.coordpair[j].coords[0].y << "), (" << temps.coordpair[j].coords[1].x << ", "
        //               << temps.coordpair[j].coords[1].y << ")\n";
        //     std::cout << "temps.statuslist[" << j << "]: (" << temps.statuslist[j].status[0] << ", "
        //               << temps.statuslist[j].status[1] << ")\n";
        // }

        // Change soil
        change_soil(allocations[i], local_soil_amount);

        // Print soil_amount
        // for (int j = 0; j < GRID_SIZE_X; ++j) {
        //     for (int k = 0; k < GRID_SIZE_Y; ++k) {
        //         std::cout << "soil_amount[" << j << "][" << k << "]: " << soil_amount[j][k]<<" ";
        //     }
            std::cout << std::endl;
        // }
    }
    std::cout << "LS built length" << built_length << std::endl;
    std::cout << "LS operate cost" << operate_cost << std::endl;
    return {result, operate_cost, built_length};
}

double cost_calculation( double operate_cost,double built_length) {
    // 定数（適宜変更してください）
    // const double grid_length = GRID_SIZE;            // グリッドの長さ (m)
    const double depth_unit = 1.0;            // 深さの単位 (m)
    const double speed_rough_road = 36.0;     // 速度 (km/h)
    const double cost_hour = 2000.0;            // 1時間あたりのコスト ($)
    const double volume_unit_truck = 30.0;    // 1台あたりの容量 (m3)
    const double work_eff = 0.75;              // 作業効率
    const double construction_temp = 17.5;   // 仮設道路の建設コスト係数 ($/m)

    // 運搬距離コスト
    // double soil_volume = grid_length * grid_length * depth_unit; // 1ブロックの土量 (m3)
    double time_average = operate_cost * GRID_SIZE / (speed_rough_road * 1000.0); // 所要時間 (h)
    double cost_construction = cost_hour * time_average / work_eff;
    // 仮設道路の建設コスト
    double cost_road = built_length * construction_temp * GRID_SIZE;
    // // std::cout << "cost_operation: " << operate_cost << std::endl;
    // std::cout << "built_length: " << built_length << std::endl;
    // // std::cout << "cost_construction: $" << cost_construction << std::endl;
    // std::cout << "cost_road: $" << cost_road << std::endl;
    // std::cout << "Total cost: $" << cost_construction + cost_road << std::endl;
    // std::cout << std::endl;
    return cost_construction + cost_road;
}

// evaluate_design 関数
std::tuple<Result,double> evaluate_design(
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
    const Solution& solution
) {
    auto [path_list, cost_operate, built_length] = process_allocations(allocations, solution, soil_amount);
    double total_cost = cost_calculation(cost_operate,built_length);
    // コストの出力
    std::cout << "Total cost: $" << total_cost << std::endl;
    return {path_list, total_cost};
}

// std::vector<std::vector<int>> delete_list(const std::vector<Allocation>& allocation, const Solution& solution) {
//     std::vector<std::vector<int>> deleted_list;
//     for(size_t i=0;i<allocation.size();i++){
//         deleted_list.push_back(std::vector<int>(solution.count,0));
//     }
//     for (int i = 0; i < solution.count; i++) {
//         CoordPair road = solution.coordpair[i];
//         for (int j = 0; j < allocation.size(); j++) {
//             Coord start = allocation[j].start;
//             Coord goal = allocation[j].goal;
//             if ((road.coords[0].x == start.x && road.coords[0].y == start.y) ||
//                 (road.coords[0].x == goal.x && road.coords[0].y == goal.y) ||
//                 (road.coords[1].x == start.x && road.coords[1].y == start.y) ||
//                 (road.coords[1].x == goal.x && road.coords[1].y == goal.y)) {
//                 deleted_list[j][i] = 1;
//             }
//         }
//     }
//     std::cout << "deleted_list" << std::endl;
//     for (int i = 0; i < deleted_list.size(); i++) {
//         for (int j = 0; j < deleted_list[i].size(); j++) {
//             std::cout << "deleted_list[" << i << "][" << j << "]: " << deleted_list[i][j] << std::endl;
//         }
//     }
//     return deleted_list;
// }

std::tuple<Solution,double,std::vector<Path>> local_search(
    const Solution& current_solution,
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
    ) 
{   
    Solution neighbor_solution = current_solution;

    const std::vector<std::pair<int, int>>& DIRECTIONS = {{-1,-1},{-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
    
    // Step 2: Set all timings to 1 and evaluate the solution
    for (auto& timing : neighbor_solution.timings) {
        std::fill(timing.begin(), timing.end(), 1);
    }
    std::cout << "neighbor_solution before adjust timings" << std::endl;
    auto [local_result,local_operate_cost,local_builtlength] = process_allocations(allocations, neighbor_solution, soil_amount);
    // Print local_result.used_road_flow
    std::cout << std::endl;
   
    // Step 3: Adjust timings based on used_temp_list
    for (size_t i = 0; i < neighbor_solution.step; ++i) {
        for (size_t j = 0; j < neighbor_solution.count; ++j) {
            if (local_result.used_road_flow[i][j] == 0) {
                neighbor_solution.timings[j][i] = 0;
            }
            else {
                neighbor_solution.timings[j][i] = 1;
            }
        }
    }
    std::cout << "neighbor_solution after adjust timings" << std::endl;
    auto [new_result,total_cost] = evaluate_design(allocations,soil_amount,neighbor_solution);
    return {neighbor_solution,total_cost,new_result.path};
}


std::tuple<std::vector<std::vector<int>>, double,std::vector<Path>> DynamicPrograming(std::vector<Allocation> allocations, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], const TemporaryRoads& roads) {
    std::cout << "starting optimizeRoadConstruction" << std::endl;
    using Key = std::vector<Status>; // temporary_road
    std::unordered_map<Key, std::vector<std::tuple<std::vector<std::vector<int>>, double, std::vector<Path>>>> dp; 
    int road_num = roads.count;
    size_t step_num = allocations.size();
    double local_soil_amount[GRID_SIZE_X][GRID_SIZE_Y] = {0};
    for (int i = 0; i < GRID_SIZE_X; i++) {
        for (int j = 0; j < GRID_SIZE_Y; j++) {
            local_soil_amount[i][j] = local_soil_amount[i][j];
        }
    }
    // Initial state
    std::vector<Status> initialStatus(roads.count, {{0, 0}});
    std::vector<std::vector<int>> initialTimings(roads.count, std::vector<int>(allocations.size(), 0)); // Initial timings
    std::vector<Path> initialPath(step_num);
    dp[initialStatus].push_back({initialTimings, 0, initialPath});
    //print initail dp
    for (int t = 0; t < step_num; ++t) {
        std::cout << "t: " << t << std::endl;
        std::unordered_map<Key, std::vector<std::tuple<std::vector<std::vector<int>>, double, std::vector<Path>>>> nextDp;
        const Allocation& allocation = allocations[t];
        //print soil_amount
        // std::cout << "soil_amount" << std::endl;
        // for (int i = 0; i < GRID_SIZE_X; ++i) {
        //     for (int j = 0; j < GRID_SIZE_Y; ++j) {  
        //         std::cout << local_soil_amount[i][j] << " ";
        //     }
        //     std::cout << std::endl;
        // }

        for (auto& [state, value] : dp) {
            // std::cout << "state: ";
            // for (const auto& status : state) {
            //     std::cout << "(" << status.status[0] << ", " << status.status[1] << ") ";
            // }
            // std::cout << std::endl;
            for(const auto& [timingsList, cost , pathlist] : value) {
                // std::cout << "cost: " << cost << std::endl;
                // // Calculate the base cost for the current state
                // std::cout << "timingsList: " << std::endl;
                // for (const auto& timing : timingsList) {
                //     for (int t : timing) {
                //         std::cout << t << " ";
                //     }
                //     std::cout << std::endl;
                // }
            double base_cost_t=0;
            TemporaryRoads current_temps;
            current_temps.count = roads.count;
            current_temps.coordpair = roads.coordpair;
            current_temps.statuslist = state;
            // std::cout << std::endl;
            //check current_temps
            // std::cout << "current_temps infomation for check: " << std::endl;
            // std::cout << "coordpair: " << std::endl;
            // for (const auto& coordpair : current_temps.coordpair) {
            //     std::cout << "(" << coordpair.coords[0].x << ", " << coordpair.coords[0].y << "), (" << coordpair.coords[1].x << ", " << coordpair.coords[1].y << ")" << std::endl;
            // }
            // std::cout << "statuslist: " << std::endl;
            // for (const auto& status : current_temps.statuslist) {
            //     std::cout << "(" << status.status[0] << ", " << status.status[1] << ") ";
            // }
            // std::cout << std::endl;
            std::vector<int> usedRoads_0;
            usedRoads_0.resize(roads.count, 0);
            Path path_0;
            path_0.new_size = 0;
            path_0.resize();
            // std::cout << "before astar" << std::endl;
            astar(allocation, current_temps, local_soil_amount, usedRoads_0, base_cost_t, path_0);
            // std::cout << "after astar" << std::endl;
            // //print path, base_cost_t,usedRoads
            // std::cout << "base_cost_t: " << base_cost_t << std::endl;
            // std::cout << "path_0: ";
            // for (const auto& coord : path_0.coord) {
            //     std::cout << "(" << coord.x << ", " << coord.y << ") ";
            // }
            // std::cout << "usedRoads_0: ";
            // for (int i : usedRoads_0) {
            //     std::cout << i << " ";
            // }
            // std::cout << std::endl;
            for (int j = 0; j < roads.count; ++j) {
                if ((roads.coordpair[j].coords[0].x == allocation.start.x && roads.coordpair[j].coords[0].y == allocation.start.y) ||
                    (roads.coordpair[j].coords[0].x == allocation.goal.x && roads.coordpair[j].coords[0].y == allocation.goal.y)) {
                    current_temps.statuslist[j].status[0] = 0;}
                if ((roads.coordpair[j].coords[1].x == allocation.start.x && roads.coordpair[j].coords[1].y == allocation.start.y) ||
                    (roads.coordpair[j].coords[1].x == allocation.goal.x && roads.coordpair[j].coords[1].y == allocation.goal.y)) {
                    current_temps.statuslist[j].status[1] = 0;}
                // Update the dp for the new status and cost
                }

            auto cost_0 = cost_calculation(base_cost_t,0);
            std::vector<Path> path_0_list = pathlist;
            path_0_list[t] = path_0;
            nextDp[current_temps.statuslist].push_back({timingsList, cost+cost_0, path_0_list});
            
            // //for debug
            // auto cost_0 = base_cost_t;
            // std::vector<Path> path_0_list = pathlist;
            // path_0_list[t] = path_0;
            // nextDp[current_temps.statuslist].push_back({timingsList, cost_0, path_0_list});

            // Explore all subsets of roads to build
            for (int i = 1; i < (1 << roads.count); ++i) {
                // std::cout << "i: " << i << std::endl;
                TemporaryRoads current_temps;
                current_temps.count = roads.count;
                current_temps.coordpair = roads.coordpair;
                double cost_i = 0;
                std::vector<Status> newStatus = state;
                bool changed = false;
                std::vector<std::vector<int>> newTimings = timingsList;
                std::vector<int> changedRoads;
                changedRoads.resize(roads.count, 0);
                double distance = 0;
                // Update the road statuses based on the subset
                // //print coord and status 
                // for ( int k = 0;k<roads.count;k++){
                //     std::cout << "coordpair: (" << current_temps.coordpair[k].coords[0].x << ", " << current_temps.coordpair[k].coords[0].y << "), (" << current_temps.coordpair[k].coords[1].x << ", " << current_temps.coordpair[k].coords[1].y << ")" << std::endl;
                //     std::cout << "status before: (" << newStatus[k].status[0] << ", " << newStatus[k].status[1] << ")" << std::endl;
                // }

                // for (const auto& coordpair : current_temps.coordpair) {
                //     std::cout << "coordpair: " << "(" << coordpair.coords[0].x << ", " << coordpair.coords[0].y << "), (" << coordpair.coords[1].x << ", " << coordpair.coords[1].y << ")" << std::endl;
                //     std::cout << "status before: ";
                //     std::cout << "(" << newStatus[coordpair.coords[0].x].status[0] << ", " << newStatus[coordpair.coords[0].y].status[1] << ") " << std::endl;
                // }
                for (int j = 0; j < roads.count; ++j) {
                    if (i & (1 << j) ){
                        newTimings[j][t] = 1;
                        double distance_j = calculate_distance_3D(roads.coordpair[j].coords[0], roads.coordpair[j].coords[1], local_soil_amount);
                        // std::cout << "distance_j from (" << roads.coordpair[j].coords[0].x << ", " << roads.coordpair[j].coords[0].y << ") to (" << roads.coordpair[j].coords[1].x << ", " << roads.coordpair[j].coords[1].y << ") : " << distance_j << std::endl;
                        // 状態に応じた距離の計算
                        if (newStatus[j].status[0] == 0 && newStatus[j].status[1] == 0) {
                            // 両方が 0 の場合はそのまま距離を加算
                            distance += distance_j;
                            changedRoads[j] = 1;
                            changed = true;
                        } else if (newStatus[j].status[0] == 0 || newStatus[j].status[1] == 0) {
                            // どちらか片方だけが 1 の場合は距離の半分を加算
                            distance += distance_j / 2;
                            changedRoads[j] = 1;
                            changed = true;
                        }
                        // 状態の更新
                        newStatus[j].status[0] = 1;
                        newStatus[j].status[1] = 1;
                        // 状態変更フラグの設定
                        }
                    }
                // std::cout << "distance " << distance << std::endl;
                //print newTimings
                // std::cout << "newTimings: " << std::endl;
                // for (const auto& timing : newTimings) {
                //     for (int t : timing) {
                //         std::cout << t << " ";
                //     }
                //     std::cout << std::endl;
                // }
                // std::cout << "newStatus after: " << std::endl;
                // for (int i = 0; i < newStatus.size(); i++) {
                //     std::cout << "(" << newStatus[i].status[0] << ", " << newStatus[i].status[1] << ") " << std::endl;
                // }
                // std::cout << std::endl;
                // std::cout << "changed: " << changed << std::endl;
                // std::cout << "changed roads: ";
                // for (int i : changedRoads) {
                //     std::cout << i << " ";
                // }
                // std::cout << std::endl;
                if (!changed) continue;
                current_temps.statuslist = newStatus;

                // bool print = false;
                // //statusが(0,1)か(1,0)の場合、statusを出力
                // for( int j = 1; j < roads.count; ++j) {
                //     if((current_temps.statuslist[j].status[0] == 0 && current_temps.statuslist[j].status[1] == 1) ||
                //         (current_temps.statuslist[j].status[0] == 1 && current_temps.statuslist[j].status[1] == 0)) {
                //         print = true;
                //         break;
                //     }
                // }
                // if(print) {
                //     std::cout << "current_temps coord + status before astar and removal: " << std::endl;
                //     for (int k = 0; k < current_temps.statuslist.size(); k++) {
                //         std::cout << "coordpair: (" << current_temps.coordpair[k].coords[0].x << ", " << current_temps.coordpair[k].coords[0].y << "), (" << current_temps.coordpair[k].coords[1].x << ", " << current_temps.coordpair[k].coords[1].y << ")" << std::endl;
                //         std::cout << "status: (" << current_temps.statuslist[k].status[0] << ", " << current_temps.statuslist[k].status[1] << ")" << std::endl;
                //     }
                //     std::cout << std::endl;
                // }                   
    
                // Check route feasibility with A*
                std::vector<int> usedRoads;
                usedRoads.resize(roads.count, 0);
                Path path;
                path.new_size = 0;
                path.resize();
                // std::cout << "cost_i before astar: " << cost_i << std::endl;
                // std::cout << "soil amount before astar";
                // std::cout << "alloation before astar: " << std::endl;
                // std::cout << "start: (" << allocation.start.x << ", " << allocation.start.y << ")" << std::endl;
                // std::cout << "goal: (" << allocation.goal.x << ", " << allocation.goal.y << ")" << std::endl;
                // std::cout << "volume: " << allocation.volume << std::endl;
                //print current_temps
                // std::cout << "current_temps infomation for check: " << std::endl;
                // for (int i = 0; i < GRID_SIZE_X; ++i) {
                //     for (int j = 0; j < GRID_SIZE_Y; ++j) {
                //         std::cout << local_soil_amount[i][j] << " ";
                //     }
                //     std::cout << std::endl;
                // }                
                astar(allocation, current_temps, local_soil_amount, usedRoads, cost_i, path);
                // std::cout << "path: ";
                // for (const auto& coord : path.coord) {
                //     std::cout << "(" << coord.x << ", " << coord.y << ") ";
                // }
                // std::cout <<"cost_i: " << cost_i << std::endl;
                // std::cout << std::endl;
                // // std::cout << "usedRoads: ";
                // // for (int i : usedRoads) {
                // //     std::cout << i << " ";
                // // }
                // if(print){
                // std::cout << "cost_i: " << cost_i << std::endl;
                // std::cout << "path: ";
                // for (const auto& coord : path.coord) {
                //     std::cout << "(" << coord.x << ", " << coord.y << ") ";
                // }
                // }
                // cost_i が base_cost_t 以上であればスキップ
                if (cost_i >= base_cost_t) continue;
                // Ensure all newly built roads are used
                bool allUsed = true;
                for (int j = 0; j < roads.count; ++j) {
                    if (changedRoads[j] == 1 && usedRoads[j] == 0) {
                        allUsed = false;
                        break;
                    }
                }
                // std::cout << "allUsed: " << allUsed << std::endl;
                if (!allUsed) continue;
            
            // change soil amount

            // allocation のスタートとゴール上に存在する仮設道路の status を 0 に変更
            for (int j = 0; j < roads.count; ++j) {
                if ((roads.coordpair[j].coords[0].x == allocation.start.x && roads.coordpair[j].coords[0].y == allocation.start.y) ||
                    (roads.coordpair[j].coords[0].x == allocation.goal.x && roads.coordpair[j].coords[0].y == allocation.goal.y)) {
                    newStatus[j].status[0] = 0;
                    // std::cout << "newStatus[" << j << "].status[0] == 0" << std::endl;
                    }
                if ((roads.coordpair[j].coords[1].x == allocation.start.x && roads.coordpair[j].coords[1].y == allocation.start.y) ||
                    (roads.coordpair[j].coords[1].x == allocation.goal.x && roads.coordpair[j].coords[1].y == allocation.goal.y)) {
                    newStatus[j].status[1] = 0;
                    // std::cout << "newStatus[" << j << "].status[1] == 0" << std::endl;
                    }
                // Update the dp for the new status and cost
                }
            // std::cout << "newStatus after removal: " << std::endl;
            // for (int k = 0; k < newStatus.size(); k++) {
            //     std::cout << "status: (" << newStatus[k].status[0] << ", " << newStatus[k].status[1] << ")" << std::endl;
            // }
            // std::cout << std::endl;

            auto cost_all_i = cost_calculation(cost_i,distance);
            std::vector<Path> path_i_list = pathlist;
            path_i_list[t] = path;
              nextDp[newStatus].push_back({newTimings, cost+cost_all_i, path_i_list});  
            
            //for debug
            // auto cost_all_i  = cost_i + distance;
            // std::vector<Path> path_i_list = pathlist;
            // path_i_list[t] = path;
            //   nextDp[newStatus].push_back({newTimings, cost+cost_all_i, path_i_list});  
            
        
        }
        }
        
        }
        // After updating dp, remove the higher cost solutions for the same key
        for (auto& [key, costList] : nextDp) {
            std::sort(costList.begin(), costList.end(), [](const auto& a, const auto& b) {
                return std::get<1>(a) < std::get<1>(b);  // Sort by cost (ascending)
            });

            // Keep only the lowest cost solution for each key
            costList.erase(std::unique(costList.begin(), costList.end(), [](const auto& a, const auto& b) {
                return std::get<1>(a) == std::get<1>(b);
            }), costList.end());
        }
        dp = std::move(nextDp);  // Move to the next state

        change_soil(allocation, local_soil_amount);    

        // print dp timing if timings for coordpair 2,3,4 are all 1
        // for (const auto& [key, value] : dp) {
        //     for (const auto& [timings, cost, path] : value) {
        //         if (timings[2][t] == 1 && timings[3][t] == 1 && timings[4][t] == 1) {
        //             std::cout << "dp in step " << t << std::endl;
        //             for (const auto& status : key) {
        //                 std::cout << "(" << status.status[0] << ", " << status.status[1] << ") ";
        //             }
        //             std::cout << std::endl;
        //             for (const auto& [timings, cost, path] : value) {
        //                 std::cout << "Cost: " << cost << std::endl;
        //                 std::cout << "Timings:\n";
        //                 for (const auto& timing : timings) {
        //                     for (int t : timing) {
        //                         std::cout << t << " ";
        //                     }
        //                     std::cout << std::endl;
        //                 }
        //             }
        //         }
        //     }
        // }


        // std::cout << "dp in step " << t << std::endl;
        // for (const auto& [key, value] : dp) {
        //     std::cout << "Key: ";
        //     for (const auto& status : key) {
        //         std::cout << "(" << status.status[0] << ", " << status.status[1] << ") ";
        //     }
        //     std::cout << std::endl;
        //     for (const auto& [timings, cost, path] : value) {
        //         std::cout << "Cost: " << cost << std::endl;
        //         std::cout << "Timings:\n";
        //         for (const auto& timing : timings) {
        //             for (int t : timing) {
        //                 std::cout << t << " ";
        //             }
        //             std::cout << std::endl;
        //         }
        //     }
        //     }
            //   std::cout << std::endl;
    }

    // Find the minimum cost among final states
    double minCost = INF;
    std::vector<std::vector<int>> min_timing;
    std::vector<Path> min_path;
    for (const auto& [state, costList] : dp) {
        for (const auto& [timings, cost, path] : costList) {
            if (cost < minCost){
                min_timing = timings;
                minCost = cost;
                min_path = path;
            }
        }
    }
    std::cout << "minPath: " << std::endl;
    for (const auto& path : min_path) {
        for (const auto& coord : path.coord) {
            std::cout << "(" << coord.x << ", " << coord.y << ") ";
        }
        std::cout << std::endl;
    }

    std::cout << "Minimum cost to complete all tasks: " << minCost << std::endl;
    std::cout << "minimum timing: " << std::endl;
    for (const auto& timing : min_timing) {

        for (int t : timing) {
            std::cout << t << " ";
        }
        std::cout << std::endl;
    }
    return {min_timing, minCost,min_path};
}


int main() {
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "test" << std::endl;
    int count = 7;
    Coord coord1 = {0, 1};
    Coord coord2 = {1, 0};
    //Initialize soil_amount
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y] = {
        {0, 1, 1, 1},
        {1, 1, 1, 1},
        {1, 1, 1, 1},
        {1, 1, 1, 1}};
    // Initialize allocations
    // std::vector<Allocation> allocations = {
    //     {{0, 0}, {3, 1}, 15000.0},
    //     {{0, 0}, {3, 1}, 15000.0}
    //     };

    // std::vector<Allocation> allocations = {
    //     {{0, 0}, {1, 1}, 15000.0},
    //     {{1, 0}, {2, 0}, 15000.0},
    //     {{2, 0}, {3, 1}, 15000.0},
    //     {{3, 1}, {3, 2}, 15000.0},
    //     {{3, 2}, {2, 3}, 15000.0},
    //     {{2, 3}, {1, 3}, 15000.0},
    //     {{1, 3}, {0, 2}, 15000.0},
    //     {{0, 2}, {0, 1}, 15000.0},
    //     {{0, 1}, {1, 0}, 15000.0},
    //     {{1, 0}, {2, 1}, 15000.0},
    //     {{2, 1}, {2, 2}, 15000.0},
    //     {{2, 2}, {1, 2}, 15000.0},
    //     {{1, 2}, {1, 1}, 15000.0},
    //     {{1, 1}, {2, 0}, 15000.0},
    //     {{2, 0}, {3, 1}, 15000.0},
    //     {{3, 1}, {3, 2}, 15000.0},
    //     {{3, 2}, {2, 3}, 15000.0},
    //     {{2, 3}, {1, 3}, 15000.0},
    //     {{1, 3}, {0, 2}, 15000.0},
    //     {{0, 2}, {0, 1}, 15000.0},
    //     {{0, 1}, {1, 0}, 15000.0},
    //     {{1, 0}, {2, 1}, 15000.0},
    //     {{2, 1}, {2, 2}, 15000.0},
    //     };
    std::vector<Allocation> allocations = {
        {{11, 1}, {10, 2}, 2500.0},
        {{11, 1}, {9, 2}, 5400.0}, 
        {{11, 0}, {10, 0}, 6900.0}, 
        {{11, 0}, {9, 0}, 4300.0}, 
        {{10, 1}, {9, 2}, 12100.0}, 
        {{10, 1}, {9, 1}, 2200.0}, 
        {{8, 3}, {9, 3}, 100.0},
        {{8, 3}, {9, 2}, 2200.0},
        {{4, 1}, {9, 2}, 8800.0}, 
        {{5, 0}, {9, 0}, 8000.0},
        {{4, 0}, {9, 0}, 4000.0},
        {{3, 0}, {9, 0}, 5700.0},
        {{5, 2}, {8, 2}, 14200.0},
        {{4, 2}, {8, 2}, 24300.0},
        {{3, 1}, {8, 2}, 34000.0},
    };        

    int step_num = allocations.size();
    std::cout << "step_num: " << step_num << std::endl;
    CoordPair coordpair[count] = {
        {{{6, 2}, {5, 2}}}, 
        {{{6, 2}, {7, 2}}}, 
        {{{6, 0}, {7, 0}}}, 
        {{{4, 2}, {5, 2}}}, 
        {{{5, 0}, {6, 0}}}, 
        {{{5, 0}, {4, 0}}}, 
        {{{3, 2}, {4, 2}}}
    };
    //normalize coordpair
    for (int i = 0; i < count; i++) {
        coordpair[i] = normalize_pair(coordpair[i].coords[0], coordpair[i].coords[1]);
    }
    // Initialize current_temps
    TemporaryRoads current_temps;
    current_temps.count = count;
    for (int i = 0; i < count; i++) {
        current_temps.coordpair.push_back(coordpair[i]);
    }
    current_temps.statuslist.resize(count, {{0, 0}});
    // Initialize solution
    Solution solution;
    solution.count = count;
    solution.step = allocations.size();
    for (int i = 0; i < count; i++) {
        solution.coordpair.push_back(coordpair[i]);
    }
    // solution.timings = std::vector<std::vector<int>>(count, std::vector<int>(step_num, 0));
    solution.timings = std::vector<std::vector<int>>(count, std::vector<int>(step_num, 0));
    for(int i = 0; i < count; i++){
        if(solution.timings[i].size() != solution.step){
            std::clog << "Error: The number of elements in coordpair does not match the number of elements in the solution" << std::endl;
            return 1;
        }
    }
// //evaluate_design
//     auto [result, sample_cost] = evaluate_design(allocations, soil_amount, solution);
//     std::cout << "sample_cost: " << sample_cost << std::endl;

    // double sample_ope = (2+sqrt(2))/2 * 500 + (sqrt(pow(15000,2)+pow(1,2))/2+1/2+sqrt(pow(15000,2)+2)/2)*500;
    // double sample_built = 2 + sqrt(2)+sqrt(pow(15000,2)+pow(1,2))/2+sqrt(pow(15000,2)+2)/2;
    // double sample_ope = 3022.1;
    // // double sample_built = 2 + sqrt(2)+sqrt(pow(300,2)+pow(1,2))/2+sqrt(pow(300,2)+2)/2;
    // double sample_built = 303.417;
    // double sample_best = cost_calculation(sample_ope,sample_built);
    // std::cout << "sample_best: " << sample_best << std::endl;
    Path path;
    path.new_size = 0;
    path.resize();
    double total_cost = 0;
    // Initialize path and used_temp_list
    // std::vector<Path> path(solution.step, Path{std::vector<Coord>()});
    std::vector<std::vector<int>> used_temp_list(solution.step, std::vector<int>(solution.count, 0));

    std::vector<double> built_length_list(solution.count, 0.0);

    // for(int i = 0; i <5; i++){ 
    //     total_cost = 0;
    //     astar(allocations[0], current_temps, soil_amount, used_temp_list[0],total_cost, path);
    //     std::cout << "path: ";
    //     for (const auto& coord : path.coord) {
    //         std::cout << "(" << coord.x << ", " << coord.y << ") ";
    //     }
    //     std::cout << "total_cost: " << total_cost << std::endl;
    //     //print soil_amount
    //     std::cout << "soil_amount" << std::endl;
    //     for (int i = 0; i < GRID_SIZE_X; ++i) {
    //         for (int j = 0; j < GRID_SIZE_Y; ++j) {
    //             std::cout << soil_amount[i][j] << " ";
    //         }
    //         std::cout << std::endl;
    //     }
    // }
    // Evaluate the design
    // auto [result, total_cost, built_length] = process_allocations(allocations, solution, soil_amount);
    // if (result.used_road_flow[0].size() >1000) {
    //     std::clog << "Error: The number of elements is too large" << std::endl;
    //     return 1;
    // }
    // Print result
    // for (int i = 0; i < result.size; i++) {
    //     for (int j = 0; j < result.used_road_flow[i].size(); j++) {
    //         std::cout << "result.used_road_flow[" << i << "][" << j << "]: " << result.used_road_flow[i][j] << std::endl;
    //     }
    // }

//局所探索
auto [solution_LS,cost_LS,pathlist_LS] =  local_search(solution, allocations, soil_amount);
    // // Print local_search result
    std::cout << "local_search cost: " << cost_LS << std::endl;
    for (int i = 0; i < solution_LS.count; i++) {
        std::cout << "solution_LS.coordpair[" << i << "]: (" << solution_LS.coordpair[i].coords[0].x << ", "
                  << solution_LS.coordpair[i].coords[0].y << "), (" << solution_LS.coordpair[i].coords[1].x << ", "
                  << solution_LS.coordpair[i].coords[1].y << ")\n";
        std::cout << "solution_LS.timings[" << i << "]: ";
        for (int j = 0; j < solution_LS.step; j++) {
            std::cout << solution_LS.timings[i][j] << " ";
        }
        std::cout << std::endl;
    }
    //print path
    for (int i = 0; i < pathlist_LS.size(); i++) {
        std::cout << "pathlist_LS[" << i << "]: ";
        for (int j = 0; j < pathlist_LS[i].coord.size(); j++) {
            std::cout << " (" << pathlist_LS[i].coord[j].x << ", " << pathlist_LS[i].coord[j].y << ")";
        }
        std::cout << std::endl;
    }


// //動的計画
auto [timing_DP,cost_DP,pathlist_DP] = DynamicPrograming(allocations, soil_amount, current_temps);
    std::cout << std::endl;
    // // Print DynamicPrograming result
    std::cout << "DynamicPrograming cost: " << cost_DP << std::endl;
    for (int i = 0; i < count; i++) {
        std::cout << "solution_DP.coordpair[" << i << "]: (" << solution.coordpair[i].coords[0].x << ", "
                  << solution.coordpair[i].coords[0].y << "), (" << solution.coordpair[i].coords[1].x << ", "
                  << solution.coordpair[i].coords[1].y << ")\n";
        std::cout << "  timing: ";
        for (int j = 0; j < timing_DP[i].size(); j++) {
            std::cout  << timing_DP[i][j] << " ";
        }
        std::cout << std::endl;
    }
    //print path
    for (int i = 0; i < pathlist_DP.size(); i++) {
        std::cout << "pathlist_DP[" << i << "]: ";
        for (int j = 0; j < pathlist_DP[i].coord.size(); j++) {
            std::cout << " (" << pathlist_DP[i].coord[j].x << ", " << pathlist_DP[i].coord[j].y << ")";
        }
        std::cout << std::endl;
    }
    // simulated_annealing(allocations, solution, soil_amount, 0.95, 1000);
    auto end = std::chrono::high_resolution_clock::now();

    // 経過時間を取得 (ミリ秒)
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "total time: " << duration.count() << " ms" << std::endl;
    return 0;
}

