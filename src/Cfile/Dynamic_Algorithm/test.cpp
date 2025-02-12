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

#include <unordered_set>
#include <bitset>
#include <functional>


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
    int status[2]; // status[0]: road from start to midpoint, status[1]: road from end to midpoint
    bool operator==(const Status& other) const {
        return status[0] == other.status[0] && status[1] == other.status[1];
    }
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

struct TemporaryRoads {
    int count;
    std::vector<CoordPair> coordpair;
    std::vector<Status> statuslist;
};

struct Task {
    Coord start;
    Coord goal;
};

struct State {
    std::vector<Status> roadStatus;
    int step;

    bool operator==(const State& other) const {
        return roadStatus == other.roadStatus && step == other.step;
    }
};


struct Allocation {
    Coord start;
    Coord goal;
    double volume;
};

struct Node {
    int priority;
    Coord coord;

    // 比較演算子で優先度を定義（小さい値が高優先度）
    bool operator>(const Node& other) const {
        return priority > other.priority;
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

double calculate_distance_3D(const Coord& a, const Coord& b, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = (soil_amount[a.x][a.y] - soil_amount[b.x][b.y]) / (GRID_SIZE * GRID_SIZE * GRID_SIZE);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
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
    std::cout << "cost_construction: $" << cost_construction << std::endl;
    std::cout << "cost_road: $" << cost_road << std::endl;
    return cost_construction + cost_road;
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

// Placeholder for A* pathfinding
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
                // int priority = new_cost + heuristic(goal, neighbor, soil_amount);
                int priority = new_cost + calculate_distance_3D(goal, neighbor, soil_amount);
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
    // path.new_size = reverse_path.size();
    // path.resize();
    path.coord.assign(reverse_path.rbegin(), reverse_path.rend());
    total_cost += cost_so_far[goal.x][goal.y];

    // // 経路出力
    // std::cout << "Path:\n";
    // for (const auto& coord : path.coord) {
    //     std::cout << "(" << coord.x << ", " << coord.y << ")\n";
    // }
}
void optimizeRoadConstruction(std::vector<Allocation> allocations, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], const TemporaryRoads& roads) {
    std::cout << "starting optimizeRoadConstruction" << std::endl;
    using Key = std::vector<Status>; // temporary_road
    std::unordered_map<Key, std::vector<std::pair<std::vector<std::vector<int>>, double>>> dp; // key: temporary_road, value: list of {timings, cost}
    int road_num = roads.count;
    int step_num = allocations.size();
    // Initial state
    std::vector<Status> initialStatus(roads.count, {{0, 0}});
    std::vector<std::vector<int>> initialTimings(roads.count, std::vector<int>(allocations.size(), 0)); // Initial timings
    dp[initialStatus].push_back({initialTimings, 0});
    //print initail dp
    std::cout << "Initial dp:\n";   
    for (const auto& [key, value] : dp) {
        std::cout << "Key: ";       
        for (const auto& status : key) {
            std::cout << "(" << status.status[0] << ", " << status.status[1] << ") ";
        }
        std::cout << std::endl;
        for (const auto& [timings, cost] : value) {
            std::cout << "Cost: " << cost << std::endl;
            std::cout << "Timings:\n";
            for (const auto& timing : timings) {
                for (int t : timing) {
                    std::cout << t << " ";
                }
                std::cout << std::endl;
            }
        }
    }
    for (int t = 0; t < step_num; ++t) {
        std::cout << "t: " << t << std::endl;
        std::unordered_map<Key, std::vector<std::pair<std::vector<std::vector<int>>, double>>> nextDp;
        const Allocation& allocation = allocations[t];

        for (auto& [state, value] : dp) {
            std::cout << "state: ";
            for (const auto& status : state) {
                std::cout << "(" << status.status[0] << ", " << status.status[1] << ") ";
            }
            std::cout << std::endl;
            for(const auto& [timingsList, cost] : value) {
                std::cout << "cost: " << cost << std::endl;
                // Calculate the base cost for the current state
                std::cout << "timingsList: " << std::endl;
                for (const auto& timing : timingsList) {
                    for (int t : timing) {
                        std::cout << t << " ";
                    }
                    std::cout << std::endl;
                }
            double base_cost_t=0;
            TemporaryRoads current_temps;
            current_temps.count = roads.count;
            current_temps.coordpair = roads.coordpair;
            current_temps.statuslist = state;
            //check current_temps
            std::cout << "current_temps: " << std::endl;
            std::cout << "coordpair: " << std::endl;
            for (const auto& coordpair : current_temps.coordpair) {
                std::cout << "(" << coordpair.coords[0].x << ", " << coordpair.coords[0].y << "), (" << coordpair.coords[1].x << ", " << coordpair.coords[1].y << ")" << std::endl;
            }
            std::cout << "statuslist: " << std::endl;
            for (const auto& status : current_temps.statuslist) {
                std::cout << "(" << status.status[0] << ", " << status.status[1] << ") ";
            }
            std::cout << std::endl;
            std::vector<int> usedRoads_0;
            usedRoads_0.resize(roads.count, 0);
            Path path_0;
            path_0.new_size = 0;
            path_0.resize();
            std::cout << "before astar" << std::endl;
            astar(allocation, current_temps, soil_amount, usedRoads_0, base_cost_t, path_0);
            std::cout << "after astar" << std::endl;
            //print path, base_cost_t,usedRoads
            std::cout << "base_cost_t: " << base_cost_t << std::endl;
            std::cout << "path_0: ";
            for (const auto& coord : path_0.coord) {
                std::cout << "(" << coord.x << ", " << coord.y << ") ";
            }
            std::cout << "usedRoads_0: ";
            for (int i : usedRoads_0) {
                std::cout << i << " ";
            }
            std::cout << std::endl;
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
            nextDp[current_temps.statuslist].push_back({timingsList, cost+cost_0});

            // Explore all subsets of roads to build
            for (int i = 1; i < (1 << roads.count); ++i) {
                std::cout << "i: " << i << std::endl;
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
                        std::cout << "road coord: (" << roads.coordpair[j].coords[0].x << ", " << roads.coordpair[j].coords[0].y << "), (" << roads.coordpair[j].coords[1].x << ", " << roads.coordpair[j].coords[1].y << ")" << std::endl;

                        double distance_j = calculate_distance_3D(roads.coordpair[j].coords[0], roads.coordpair[j].coords[1], soil_amount);
                        std::cout << "distance_j: " << distance_j << std::endl;
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
                    std::cout << "distance " << distance << std::endl;
                //print newTimings
                std::cout << "newTimings: " << std::endl;
                for (const auto& timing : newTimings) {
                    for (int t : timing) {
                        std::cout << t << " ";
                    }
                    std::cout << std::endl;
                }
                std::cout << "changed: " << changed << std::endl;
                std::cout << "changed roads: ";
                for (int i : changedRoads) {
                    std::cout << i << " ";
                }
                std::cout << std::endl;
                if (!changed) continue;
                current_temps.statuslist = newStatus;
                std::cout << "current_temps status: " << std::endl;
                for (const auto& status : current_temps.statuslist) {
                    std::cout << "(" << status.status[0] << ", " << status.status[1] << ") ";
                }
                std::cout << std::endl;
                // Check route feasibility with A*
                std::vector<int> usedRoads;
                usedRoads.resize(roads.count, 0);
                Path path;
                path.new_size = 0;
                path.resize();
                astar(allocation, current_temps, soil_amount, usedRoads, cost_i, path);
                std::cout << "path: ";
                for (const auto& coord : path.coord) {
                    std::cout << "(" << coord.x << ", " << coord.y << ") ";
                }
                std::cout << "usedRoads: ";
                for (int i : usedRoads) {
                    std::cout << i << " ";
                }
                std::cout << "cost_i: " << cost_i << std::endl;
                std::cout << std::endl;

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
                std::cout << "allUsed: " << allUsed << std::endl;
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
            std::cout << "cost_i: " << cost_i << std::endl;
            std::cout << "distance: " << distance << std::endl;
              nextDp[newStatus].push_back({newTimings, cost+cost_all_i});  
            }
        }
        
        }
        // After updating dp, remove the higher cost solutions for the same key
        for (auto& [key, costList] : nextDp) {
            std::sort(costList.begin(), costList.end(), [](const auto& a, const auto& b) {
                return a.second < b.second;  // Sort by cost (ascending)
            });

            // Keep only the lowest cost solution for each key
            costList.erase(std::unique(costList.begin(), costList.end(), [](const auto& a, const auto& b) {
                return a.second == b.second;
            }), costList.end());
        }
        dp = std::move(nextDp);  // Move to the next state
        //print dp timing
        std::cout << "dp in step " << t << std::endl;
        for (const auto& [key, value] : dp) {
            std::cout << "Key: ";
            for (const auto& status : key) {
                std::cout << "(" << status.status[0] << ", " << status.status[1] << ") ";
            }
            std::cout << std::endl;
            for (const auto& [timings, cost] : value) {
                std::cout << "Cost: " << cost << std::endl;
                std::cout << "Timings:\n";
                for (const auto& timing : timings) {
                    for (int t : timing) {
                        std::cout << t << " ";
                    }
                    std::cout << std::endl;
                }
            }
            }
              std::cout << std::endl;
    }

    // Find the minimum cost among final states
    double minCost = INF;
    std::vector<std::vector<int>> min_timing = std::vector<std::vector<int>>(roads.count, std::vector<int>(step_num, 0));
    for (const auto& [state, costList] : dp) {
        for (const auto& [timings, cost] : costList) {
            if (cost < minCost){
                min_timing = timings;
                minCost = cost;
            }
        }
    }

    std::cout << "Minimum cost to complete all tasks: " << minCost << std::endl;
    std::cout << "minimum timing: " << std::endl;
    for (const auto& timing : min_timing) {
        for (int t : timing) {
            std::cout << t << " ";
        }
        std::cout << std::endl;
    }
}

int main() {
    std::cout << "starting main" << std::endl;
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y] = {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}};


    Coord coord1 = {0, 0};
    Coord coord2 = {1, 1};
    CoordPair road1= normalize_pair(coord1, coord2);
    Coord coord3 = {0, 1};
    Coord coord4 = {0, 2};
    CoordPair road2= normalize_pair(coord3, coord4);
    Coord coord5 = {2, 2};
    Coord coord6 = {3, 3};
    CoordPair road3= normalize_pair(coord5, coord6);
    TemporaryRoads roads{
        3,
        {road1, road2, road3},
        {{{0, 0}}, {{0, 0}}, {{0, 0}}}
    };

    std::vector<Allocation> allocations = {
        { {0, 0}, {1, 3} },
        { {0, 0}, {3, 3} }
        
    };

    optimizeRoadConstruction(allocations, soil_amount, roads);
    // std::vector<Status> testKey = {{0, 0}, {0, 0}, {0, 0}};
    // std::unordered_map<std::vector<Status>, int> testMap;
    // testMap[testKey] = 42;
    // for (const auto& [key, value] : testMap) {
    // std::cout << "Value: " << value << "\nKey: ";
    // for (const auto& status : key) {
    //     std::cout << "(" << status.status[0] << ", " << status.status[1] << ") ";
    // }
    // std::cout << std::endl;
    // }
    return 0;
}
