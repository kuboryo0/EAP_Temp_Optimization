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

#define GRID_SIZE_X 4
#define GRID_SIZE_Y 4
#define STEPS 10
#define TEMP_EFF 0.67 // Temporary road efficiency
#define GRID_SIZE 150
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
                // std::cout <<"current_temps.statuslist["<<i<<"].status[0] == 0" << std::endl;
                built_length += length;
                built_length_list[i] += length;
                current_temps.statuslist[i].status[0] = 1;
            }
            if (current_temps.statuslist[i].status[1] == 0) {
                // std::cout <<"current_temps.statuslist["<<i<<"].status[1] == 0" << std::endl;
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
    total_cost += cost_so_far[goal.x][goal.y];

    // // 経路出力
    // std::cout << "Path:\n";
    // for (const auto& coord : path.coord) {
    //     std::cout << "(" << coord.x << ", " << coord.y << ")\n";
    // }
}



std::tuple<Result,double,double> process_allocations(std::vector<Allocation>& allocations,
                        const Solution& solution,
                        double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
                        ) {
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
    for (int i = 0; i < step; ++i) {
        // std::cout << "loop: " << i << std::endl;
        double cost = 0;
        double built_length_i = 0;
        temp_in_step(temps, i, solution, result.built_length_list, soil_amount, built_length_i);
        built_length += built_length_i;
        // std::cout << "built_length_i: " << built_length_i << std::endl;
        // // temp_in_step(current_temps, 0, solution, built_length_list, soil_amount, operate_cost);
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
        astar(allocations[i], temps, soil_amount, result.used_road_flow[i], cost, result.path[i]);
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
        change_soil(allocations[i], soil_amount);

        // Print soil_amount
        // for (int j = 0; j < GRID_SIZE_X; ++j) {
        //     for (int k = 0; k < GRID_SIZE_Y; ++k) {
        //         std::cout << "soil_amount[" << j << "][" << k << "]: " << soil_amount[j][k]<<" ";
        //     }
        //     std::cout << std::endl;
        // }
    }
    return {result, operate_cost, built_length};
}

double cost_calculation( double operate_cost,double built_length) {
    // 定数（適宜変更してください）
    const double grid_length = GRID_SIZE;            // グリッドの長さ (m)
    const double depth_unit = 1.0;            // 深さの単位 (m)
    const double speed_rough_road = 36.0;     // 速度 (km/h)
    const double cost_hour = 2000.0;            // 1時間あたりのコスト ($)
    const int truck_num = 1;                  // トラックの台数
    const double volume_unit_truck = 30.0;    // 1台あたりの容量 (m3)
    const double work_eff = 0.75;              // 作業効率
    const double construction_temp = 17.5;   // 仮設道路の建設コスト係数 ($/m)

    // 運搬距離コスト
    double soil_volume = grid_length * grid_length * depth_unit; // 1ブロックの土量 (m3)
    double time_average = operate_cost * grid_length / (speed_rough_road * 1000.0); // 所要時間 (h)
    double cost_construction = cost_hour * soil_volume / (truck_num * volume_unit_truck) * time_average / work_eff;
    // 仮設道路の建設コスト
    double cost_road = built_length * construction_temp * grid_length;
    // std::cout << "cost_construction: $" << cost_construction << std::endl;
    // std::cout << "cost_road: $" << cost_road << std::endl;
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

std::tuple<std::vector<std::vector<int>>, double,std::vector<Path>> optimizeRoadConstruction(std::vector<Allocation> allocations, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], const TemporaryRoads& roads) {
    std::cout << "starting optimizeRoadConstruction" << std::endl;
    using Key = std::vector<Status>; // temporary_road
    std::unordered_map<Key, std::vector<std::tuple<std::vector<std::vector<int>>, double, std::vector<Path>>>> dp; 
    int road_num = roads.count;
    size_t step_num = allocations.size();
    // Initial state
    std::vector<Status> initialStatus(roads.count, {{0, 0}});
    std::vector<std::vector<int>> initialTimings(roads.count, std::vector<int>(allocations.size(), 0)); // Initial timings
    std::vector<Path> initialPath(step_num);
    dp[initialStatus].push_back({initialTimings, 0, initialPath});
    //print initail dp
    // std::cout << "Initial dp:\n";   
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
    // }
    for (int t = 0; t < step_num; ++t) {
        // std::cout << "t: " << t << std::endl;
        std::unordered_map<Key, std::vector<std::tuple<std::vector<std::vector<int>>, double, std::vector<Path>>>> nextDp;
        const Allocation& allocation = allocations[t];

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
            //check current_temps
            // std::cout << "current_temps: " << std::endl;
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
            astar(allocation, current_temps, soil_amount, usedRoads_0, base_cost_t, path_0);
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
                for (int j = 0; j < roads.count; ++j) {
                    if (i & (1 << j) ){
                        newTimings[j][t] = 1;
                        //print road coord
                        // std::cout << "road coord: (" << roads.coordpair[j].coords[0].x << ", " << roads.coordpair[j].coords[0].y << "), (" << roads.coordpair[j].coords[1].x << ", " << roads.coordpair[j].coords[1].y << ")" << std::endl;

                        double distance_j = calculate_distance_3D(roads.coordpair[j].coords[0], roads.coordpair[j].coords[1], soil_amount);
                        // std::cout << "distance_j: " << distance_j << std::endl;
                        if(newStatus[j].status[0] == 0||newStatus[j].status[1] == 0) {
                        if(newStatus[j].status[0] == 0||newStatus[j].status[1] == 0) {
                            if(!(newStatus[j].status[0] == 0 && newStatus[j].status[1] == 0)) {
                                distance = distance / 2;}}
                        newStatus[j].status[0] = 1;
                        newStatus[j].status[1] = 1;
                        changedRoads[j] = 1;
                        changed = true;
                        distance+=distance_j; 
                        }}
                    }
                //     std::cout << "distance " << distance << std::endl;
                // //print newTimings
                // std::cout << "newTimings: " << std::endl;
                // for (const auto& timing : newTimings) {
                //     for (int t : timing) {
                //         std::cout << t << " ";
                //     }
                //     std::cout << std::endl;
                // }
                // std::cout << "changed: " << changed << std::endl;
                // std::cout << "changed roads: ";
                // for (int i : changedRoads) {
                //     std::cout << i << " ";
                // }
                // std::cout << std::endl;
                if (!changed) continue;
                current_temps.statuslist = newStatus;
                // std::cout << "current_temps status: " << std::endl;
                // for (const auto& status : current_temps.statuslist) {
                //     std::cout << "(" << status.status[0] << ", " << status.status[1] << ") ";
                // }
                // std::cout << std::endl;
                // Check route feasibility with A*
                std::vector<int> usedRoads;
                usedRoads.resize(roads.count, 0);
                Path path;
                path.new_size = 0;
                path.resize();
                astar(allocation, current_temps, soil_amount, usedRoads, cost_i, path);
                // std::cout << "path: ";
                // for (const auto& coord : path.coord) {
                //     std::cout << "(" << coord.x << ", " << coord.y << ") ";
                // }
                // std::cout << "usedRoads: ";
                // for (int i : usedRoads) {
                //     std::cout << i << " ";
                // }
                // std::cout << "cost_i: " << cost_i << std::endl;
                // std::cout << std::endl;

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

            // allocation のスタートとゴール上に存在する仮設道路の status を 0 に変更
            for (int j = 0; j < roads.count; ++j) {
                if ((roads.coordpair[j].coords[0].x == allocation.start.x && roads.coordpair[j].coords[0].y == allocation.start.y) ||
                    (roads.coordpair[j].coords[0].x == allocation.goal.x && roads.coordpair[j].coords[0].y == allocation.goal.y)) {
                    newStatus[j].status[0] = 0;}
                if ((roads.coordpair[j].coords[1].x == allocation.start.x && roads.coordpair[j].coords[1].y == allocation.start.y) ||
                    (roads.coordpair[j].coords[1].x == allocation.goal.x && roads.coordpair[j].coords[1].y == allocation.goal.y)) {
                    newStatus[j].status[1] = 0;}
                // Update the dp for the new status and cost
                }
            auto cost_all_i = cost_calculation(cost_i,distance);
            // std::cout << "cost_i: " << cost_i << std::endl;
            // std::cout << "distance: " << distance << std::endl;
            std::vector<Path> path_i_list = pathlist;
            path_i_list[t] = path;
              nextDp[newStatus].push_back({newTimings, cost+cost_all_i, path_i_list});  
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
        //print dp timing
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
        //       std::cout << std::endl;
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

std::vector<std::vector<int>> delete_list(const std::vector<Allocation>& allocation, const Solution& solution) {
    std::vector<std::vector<int>> deleted_list;
    for(size_t i=0;i<allocation.size();i++){
        deleted_list.push_back(std::vector<int>(solution.count,0));
    }
    for (int i = 0; i < solution.count; i++) {
        CoordPair road = solution.coordpair[i];
        for (int j = 0; j < allocation.size(); j++) {
            Coord start = allocation[j].start;
            Coord goal = allocation[j].goal;
            if ((road.coords[0].x == start.x && road.coords[0].y == start.y) ||
                (road.coords[0].x == goal.x && road.coords[0].y == goal.y) ||
                (road.coords[1].x == start.x && road.coords[1].y == start.y) ||
                (road.coords[1].x == goal.x && road.coords[1].y == goal.y)) {
                deleted_list[j][i] = 1;
            }
        }
    }
    std::cout << "deleted_list" << std::endl;
    for (int i = 0; i < deleted_list.size(); i++) {
        for (int j = 0; j < deleted_list[i].size(); j++) {
            std::cout << "deleted_list[" << i << "][" << j << "]: " << deleted_list[i][j] << std::endl;
        }
    }
    return deleted_list;
}

std::tuple<Solution,double> generate_neighbor(
    const Solution& current_solution,
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
    ) 
{   

    Solution neighbor_solution = current_solution;

    const std::vector<std::pair<int, int>>& DIRECTIONS = {{-1,-1},{-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
    // Step 1: Modify coordpair in the solution
    std::random_device rd;  // ハードウェア乱数生成器
    std::mt19937 gen(rd()); // メルセンヌ・ツイスタ乱数エンジン
    std::uniform_int_distribution<> dist(0, 2); // 0から2の整数乱数
    int modification_type = dist(gen);

    if (modification_type == 0 || neighbor_solution.coordpair.empty()) {
        std::cout << "Addition" << std::endl;
        // Addition: Add a new random adjacent coordinate pair
        int loop_num = 0;
        while (true) {
            loop_num++;
            // std::cout << "Addition while loop" << std::endl;
            std::random_device rd;
            std::mt19937 gen(rd()); // メルセンヌ・ツイスタ乱数エンジン
            std::uniform_int_distribution<> coord_dist(0, GRID_SIZE_X - 1); // 座標生成用の分布
            std::uniform_int_distribution<> dir_dist(0, DIRECTIONS.size() - 1); // 方向生成用の分布
            // std::cout << "coord_dist: " << coord_dist(gen) << std::endl;
            // std::cout << "dir_dist: " << dir_dist(gen) << std::endl;
            Coord coord1 = {coord_dist(gen), coord_dist(gen)};
            auto [dx, dy] = DIRECTIONS[dir_dist(gen)];
            Coord coord2 = {coord1.x + dx, coord1.y + dy};
            // Ensure the new pair is within grid bounds and does not duplicate existing pairs
            if (0 <= coord2.x && coord2.x < GRID_SIZE_X &&
                0 <= coord2.y && coord2.y < GRID_SIZE_Y 
                ){
                CoordPair search_pair = normalize_pair(coord1, coord2);
                auto it = std::find_if(
                    neighbor_solution.coordpair.begin(),
                    neighbor_solution.coordpair.end(),
                    [&](const CoordPair& pair) {
                    return pair.coords[0].x == search_pair.coords[0].x &&
                        pair.coords[0].y == search_pair.coords[0].y &&
                        pair.coords[1].x == search_pair.coords[1].x &&
                        pair.coords[1].y == search_pair.coords[1].y;
                    }
                );
                 if (it == neighbor_solution.coordpair.end()) 
                {   
                    std::cout << "added coord" << std::endl;
                    //print coord1 and coord2
                    std::cout << "(" << search_pair.coords[0].x << "," << search_pair.coords[0].y << "), (" << search_pair.coords[1].x << "," << search_pair.coords[1].y << ")" << std::endl;
                    neighbor_solution.coordpair.push_back(search_pair);
                    neighbor_solution.timings.push_back(std::vector<int>(allocations.size(), 0)); // Add corresponding timing
                    neighbor_solution.count++;
                    break;
                }
                }
        }
  
        // for (size_t i = 0;i<result.size;i++){
        //     result.used_road_flow[i].push_back(0);
        // }
        // result.built_length_list.push_back(0);

        std::cout << "addition finished" << std::endl;
    } else if (modification_type == 1) {
        // Deletion: Remove a random coordinate pair weighted by its usage and length
        std::cout << "Deletion" << std::endl;
            std::vector<double> weights(neighbor_solution.coordpair.size(), 0.0);
            double total_weight = 0.0;
        for (int i = 0; i < neighbor_solution.count; ++i) {
            // Calculate the total timings for the current road
            double total_timing = 0.0;
            for (const auto& step_timing : neighbor_solution.timings[i]) {
                total_timing += step_timing;
            }
            if (total_timing == 0) {
                // Large weight for roads that were never built
                weights[i] = 1.0; // Assign a higher base weight (adjust as needed)
            } else {
                // Smaller weight for roads that were built
                weights[i] = 3.0; // Inversely proportional to the timing
            }
            total_weight += weights[i];
        }

            // std :: cout << "total_weight: " << total_weight << std::endl;
            // Normalize weights
            if (total_weight ==0){total_weight = 1;}
            for (auto& weight : weights) {
                weight /= total_weight;
            }
            //print weights
            for (int i = 0; i < weights.size(); i++) {
                std::cout << "weights[" << i << "]: " << weights[i] << std::endl;
            }
            // Choose a random pair to delete based on weights
            std::random_device rd;
            std::mt19937 gen(rd()); // Random number generator
            std::discrete_distribution<> dist(weights.begin(), weights.end()); // discrete_distribution with weights

            size_t to_remove = dist(gen);

            std::cout << "to_remove: " << to_remove << std::endl;
            neighbor_solution.coordpair.erase(neighbor_solution.coordpair.begin() + to_remove);
            neighbor_solution.timings.erase(neighbor_solution.timings.begin() + to_remove);  
            neighbor_solution.count--;  


            std::cout << "deletion finished" << std::endl;

         
    } else if (modification_type == 2) {
    // Reassignment: Modify a random coordinate pair, weighted by usage and length
        // Calculate weights for each coordinate pair
        std::cout << "Reassignment" << std::endl;
        std::vector<double> weights(neighbor_solution.coordpair.size(), 0.0);
        // std::cout <<"neighbor_solution.coordpair.size(): " << neighbor_solution.coordpair.size() << std::endl;
        // std::cout << "neighbor_solution.count: " << neighbor_solution.count << std::endl;
        double total_weight = 0.0;
                   for (int i = 0; i < neighbor_solution.count; ++i) {
            // Calculate the total timings for the current road
            double total_timing = 0.0;
            for (const auto& step_timing : neighbor_solution.timings[i]) {
                total_timing += step_timing;
            }
            if (total_timing == 0) {
                // Large weight for roads that were never built
                weights[i] = 1.0; // Assign a higher base weight (adjust as needed)
            } else {
                // Smaller weight for roads that were built
                weights[i] = 3.0; // Inversely proportional to the timing
            }
            total_weight += weights[i];
        }
        // Normalize weights
        if (total_weight == 0) {total_weight = 1;}
        for (auto& weight : weights) {
            weight /= total_weight;
        }
        //print weights
        // for (int i = 0; i < weights.size(); i++) {
        //     std::cout << "weights[" << i << "]: " << weights[i] << std::endl;
        // }
        // Choose a random pair to modify based on weights
        std::random_device rd;
        std::mt19937 gen(rd()); // Random number generator
        std::uniform_int_distribution<> coord_dist(0, GRID_SIZE_X - 1); // Distribution for coordinates
        std::uniform_int_distribution<> dir_dist(0, DIRECTIONS.size() - 1); // Distribution for directions
        std::discrete_distribution<> dist(weights.begin(), weights.end()); // Weighted random distribution

        // Generate random index based on weighted distribution
        size_t to_modify = dist(gen);
        // Modify the selected coordinate pair
        Coord coord1 = {coord_dist(gen), coord_dist(gen)};
        auto [dx, dy] = DIRECTIONS[dir_dist(gen)];
        Coord coord2 = {coord1.x + dx, coord1.y + dy};

        // Ensure the new coord2 is within bounds
        while (0 > coord2.x || coord2.x >= GRID_SIZE_X || 0 > coord2.y || coord2.y >= GRID_SIZE_Y) {
            // std::cout << "Reassignment while loop" << std::endl;
        coord1 = {coord_dist(gen), coord_dist(gen)};
        coord2 = {coord1.x + dx, coord1.y + dy};
        }
        auto coordpair = normalize_pair(coord1, coord2);
        // Replace the selected coordinate pair with the new one
        neighbor_solution.coordpair[to_modify] = coordpair;
        // std::cout << "to_modify: " << to_modify << std::endl;
        // std::cout << "coordpair: (" << coordpair.coords[0].x << ", " << coordpair.coords[0].y << "), ("
        //           << coordpair.coords[1].x << ", " << coordpair.coords[1].y << ")\n";
        // std::cout << "reassignment finished" << std::endl;
        }
        //print neighbor_solution coordpair
        std::cout << "neighbor_solution after" << std::endl;
        for (int i = 0; i < neighbor_solution.count; i++) {
            std::cout << "neighbor_solution.coordpair[" << i << "]: (" << neighbor_solution.coordpair[i].coords[0].x << ", "
                      << neighbor_solution.coordpair[i].coords[0].y << "), (" << neighbor_solution.coordpair[i].coords[1].x << ", "
                      << neighbor_solution.coordpair[i].coords[1].y << ")\n";
            std::cout << " timing[" << i << "]: " << std::endl;
            for (int j = 0; j < neighbor_solution.step; j++) {
                std::cout <<  neighbor_solution.timings[i][j] << " ";
            }
            std::cout << std::endl;
        }

        TemporaryRoads temp;
        temp.count = neighbor_solution.count;
        temp.coordpair = neighbor_solution.coordpair;
        temp.statuslist = std::vector<Status>(neighbor_solution.count, {0, 0});

       auto [timing,min_cost,pathlist] = optimizeRoadConstruction(allocations,soil_amount,temp);
       neighbor_solution.timings = timing;

       //print path,timing,min_cost
        std::cout << "pathlist after optimize" << std::endl;
        for (int i = 0; i < pathlist.size(); i++) {
        std::cout << "pathlist[" << i << "]: ";
        for (int j = 0; j < pathlist[i].size(); j++) {
            std::cout << "(" << pathlist[i].coord[j].x << ", " << pathlist[i].coord[j].y << ") ";
        }
        std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << "min_cost after optimize: " << min_cost << std::endl;
        std::cout << "timing after optimize" << std::endl;
        for (int i = 0; i < timing.size(); i++) {
            std::cout << "neighbor_solution.coordpair[" << i << "]: (" << neighbor_solution.coordpair[i].coords[0].x << ", "
                      << neighbor_solution.coordpair[i].coords[0].y << "), (" << neighbor_solution.coordpair[i].coords[1].x << ", "
                      << neighbor_solution.coordpair[i].coords[1].y << ") ";
            for (int j = 0; j < timing[i].size(); j++) {
                std::cout << timing[i][j] << " ";
            }
            std::cout << std::endl;
        }

    return {neighbor_solution,min_cost};
}

void simulated_annealing(std::vector<Allocation>& allocations,  Solution& timing, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],double alpha, int max_iter) {
    Solution current_solution = timing;
    auto [Result,current_score] = evaluate_design(allocations,soil_amount, current_solution);
    std::cout << " evaluate_design finished" << std::endl;
    std::cout << "current_score: " << current_score << std::endl;
    std::cout << "result.size: " << Result.size << std::endl;

    Solution best_solution = current_solution;
    double best_score = current_score;
    double temperature = 1000;
    int best_score_loop = 0;

    std::vector<std::pair<double, Solution>> best_solution_flow;
    std::vector<std::pair<double, Solution>> current_solution_flow;
    std::vector<std::pair<double, Solution>> neighbor_solution_flow;
    Solution neighbor_solution;
    double neighbor_score;
    
    for (int iter = 0; iter < max_iter; ++iter) {
        std::cout << "\n";
        std::cout << "\n";
        std::cout << "simulated annealing loop: " << iter << std::endl;
        temperature *= alpha;

        //print current solution and result
        std::cout << "current_solution before" << std::endl;
        std::cout << "current_solution.count" << current_solution.count << std::endl;
        std::cout << "current_solution.step" << current_solution.step << std::endl;

        for (int i = 0; i < current_solution.count; ++i) {
            std::cout << "current_solution.coordpair[" << i << "]: (" << current_solution.coordpair[i].coords[0].x << ", "
                      << current_solution.coordpair[i].coords[0].y << "), (" << current_solution.coordpair[i].coords[1].x << ", "
                      << current_solution.coordpair[i].coords[1].y << ")\n";
            std::cout << " timing[" << i << "]: " << std::endl;
            for (int j = 0; j < current_solution.step; ++j) {
                std::cout <<  current_solution.timings[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "Result before" << std::endl;
        std::cout << "Result.size: " << Result.size << std::endl;
        for( int i = 0; i < Result.size; i++){
            std::cout << "Result.used_road_flow[" << i << "]: " << std::endl;
            for (int j = 0; j < Result.used_road_flow[i].size(); j++) {
                std::cout <<  Result.used_road_flow[i][j] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "current_score: " << current_score << std::endl;
        std::tie(neighbor_solution,neighbor_score) = generate_neighbor(current_solution, allocations, soil_amount);
        std::cout << std::endl;
        std::cout << "neighbor_solution after" << std::endl;
        std::cout << "neighbor_solution.count" << neighbor_solution.count << std::endl;
        std::cout << "neighbor_solution.step" << neighbor_solution.step << std::endl;
        for (int i = 0; i < neighbor_solution.count; ++i) {
            std::cout << "neighbor_solution.coordpair[" << i << "]: (" << neighbor_solution.coordpair[i].coords[0].x << ", "
                      << neighbor_solution.coordpair[i].coords[0].y << "), (" << neighbor_solution.coordpair[i].coords[1].x << ", "
                      << neighbor_solution.coordpair[i].coords[1].y << ")\n";
            std::cout << "  timing[" << i << "]: " << std::endl;
            for (int j = 0; j < neighbor_solution.step; ++j) {
                std::cout <<  neighbor_solution.timings[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "neighbor_score: " << neighbor_score << std::endl;
        //print result
        std::cout << "Result after" << std::endl;
        std::cout << "Result.size: " << Result.size << std::endl;
        for( int i = 0; i < Result.size; i++){
            std::cout << "Result.used_road_flow[" << i << "]: " << std::endl;
            for (int j = 0; j < Result.used_road_flow[i].size(); j++) {
                std::cout <<  Result.used_road_flow[i][j] << " ";
            }
            std::cout << std::endl;
        }
        //print path
        for (int i = 0; i < Result.size; i++) {
            std::cout << "Result.path[" << i << "]: " << std::endl;
            for (int j = 0; j < Result.path[i].coord.size(); j++) {
                std::cout << "(" << Result.path[i].coord[j].x << ", " << Result.path[i].coord[j].y << ") ";
            }
            std::cout << std::endl;
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        double random_value = dis(gen);
        // 受け入れ判定
        if ((neighbor_score < current_score) || 
            ((double)rand() / random_value < std::exp(-(std::abs(neighbor_score - current_score)) / temperature))) {
            std::cout << "Accept solution" << std::endl;
            current_solution = neighbor_solution;
            current_score = neighbor_score;
        }

        // ベスト解の更新
        if (current_score < best_score) {
            best_solution = current_solution;
            best_score = current_score;
            best_score_loop = iter;
        }

        // best_solution_flow.emplace_back(best_score, best_solution);
        // current_solution_flow.emplace_back(current_score, current_solution);
        // neighbor_solution_flow.emplace_back(neighbor_score, neighbor_solution);
    }
    std::cout << "best_score_loop: " << best_score_loop << std::endl;
    std::cout << "best_score: " << best_score << std::endl;
    std::cout << "best_solution" << std::endl;
    for (int i = 0; i < best_solution.count; ++i) {
        std::cout << "best_solution.coordpair[" << i << "]: (" << best_solution.coordpair[i].coords[0].x << ", "
                  << best_solution.coordpair[i].coords[0].y << "), (" << best_solution.coordpair[i].coords[1].x << ", "
                  << best_solution.coordpair[i].coords[1].y << ")\n";
        for (int j = 0; j < best_solution.step; ++j) {
            std::cout << "  timing[" << j << "]: " << best_solution.timings[i][j] << "\n";
        }
    }
    std::cout << "best_path" << std::endl;
    for (int i = 0; i < Result.size; i++) {
        std::cout << "Result.path[" << i << "]: " << std::endl;
        for (int j = 0; j < Result.path[i].coord.size(); j++) {
            std::cout << "(" << Result.path[i].coord[j].x << ", " << Result.path[i].coord[j].y << ") ";
        }
        std::cout << std::endl;
    }
}

int main() {
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "test" << std::endl;
    int count = 5;
    Coord coord1 = {0, 1};
    Coord coord2 = {1, 0};
    //Initialize soil_amount
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y] = {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}};
    // Initialize allocations
    std::vector<Allocation> allocations = {
        {{0, 0}, {1, 3}, 1.0},
        {{0, 1}, {1, 3}, 3.0}
        };
    int step_num = allocations.size();
    std::cout << "step_num: " << step_num << std::endl;
    CoordPair coordpair[count] = {{{{0, 0}, {1, 1}}}, {{{0, 1}, {1, 2}}}, {{{0, 2}, {1, 3}}},{{{0, 0}, {0, 1}}},{ {{0, 1}, {0, 2}}}};
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
    current_temps.statuslist.resize(count, {{0, 1}});
    // Initialize solution
    Solution solution;
    solution.count = count;
    solution.step = allocations.size();
    for (int i = 0; i < count; i++) {
        solution.coordpair.push_back(coordpair[i]);
    }
    solution.timings = std::vector<std::vector<int>>(count, std::vector<int>(step_num, 0));
    for(int i = 0; i < count; i++){
        if(solution.timings[i].size() != solution.step){
            std::clog << "Error: The number of elements in coordpair does not match the number of elements in the solution" << std::endl;
            return 1;
        }
    }

    // Initialize path and used_temp_list
    // std::vector<Path> path(solution.step, Path{std::vector<Coord>()});
    // std::vector<std::vector<int>> used_temp_list(solution.step, std::vector<int>(solution.count, 0));

    std::vector<double> built_length_list(solution.count, 0.0);

    // Evaluate the design
    auto [result, total_cost, built_length] = process_allocations(allocations, solution, soil_amount);
    if (result.used_road_flow[0].size() >1000) {
        std::clog << "Error: The number of elements is too large" << std::endl;
        return 1;
    }
    // Print result
    // for (int i = 0; i < result.size; i++) {
    //     for (int j = 0; j < result.used_road_flow[i].size(); j++) {
    //         std::cout << "result.used_road_flow[" << i << "][" << j << "]: " << result.used_road_flow[i][j] << std::endl;
    //     }
    // }
Solution new_solution;
double cost;
for (size_t i = 0; i<1; i++){
    std::cout << std::endl;
    std::cout <<"in loop" << i << std::endl;
    std::tie(new_solution,cost) =  generate_neighbor(solution, allocations, soil_amount);
    solution = new_solution;
}
    // simulated_annealing(allocations, solution, soil_amount, 0.95, 1000);
    auto end = std::chrono::high_resolution_clock::now();

    // 経過時間を取得 (ミリ秒)
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "total time: " << duration.count() << " ms" << std::endl;
    return 0;
}
