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

// Objective function parameters
#define GRID_SIZE_X 12
#define GRID_SIZE_Y 4
#define DIRECTIONS_COUNT 8
#define INF std::numeric_limits<double>::infinity()
// #define VELOCITY_ROUGH 24.0
#define VELOCITY_ROUGH 0.01
#define VELOCITY_PAVE 36.0
#define TEMP_EFF VELOCITY_ROUGH/VELOCITY_PAVE // Temporary road efficiency
#define GRID_SIZE 150
#define V_truck 40.0
#define TRUCK_NUM 1
#define COST_HOUR 2000.0
#define WORK_EFF 0.75
#define CONSTRUCTION_TEMP 17.5

#define entranceXPostion 0
#define entranceYPostion 0

// Simulated Annealing parameters
#define alpha 0.95
#define max_iter 1
#define initialTemperature 1000.0

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
    int roadNum;
    std::vector<CoordPair> coordpair;
    std::vector<Status> statuslist;   // status[0]: startと中間地点の建設状況, status[1]: goalと中間地点の建設状況.0:未建設,1:建設済み
};

struct Solution {
    int roadNum;
    int stepNum;
    std::vector<CoordPair> coordpair;
    std::vector<std::vector<int>> timings;  //timings[road][stepNum]

    void printParameters() {
        std::cout << "roadNum: " << roadNum << std::endl;
        std::cout << "stepNum: " << stepNum << std::endl;
        for (int i = 0; i < roadNum; i++) {
            std::cout << "coordpair[" << i << "]: (" << coordpair[i].coords[0].x << ", " << coordpair[i].coords[0].y << "), ("
                      << coordpair[i].coords[1].x << ", " << coordpair[i].coords[1].y << ")" << std::endl;
            for (int j = 0; j < stepNum; j++) {
                std::cout << "  timing[" << j << "]: " << timings[i][j] << std::endl;
            }
        }
    }
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

struct Path {
    std::vector<Coord> coord;

    void resize(size_t new_size) {
        coord.resize(new_size);
    }
    size_t size() const {
        return coord.size();
    }
};

struct Result {
    int size;  // Number of Steps
    std::vector<Path> path;
    std::vector<std::vector<int>> used_road_flow;  //used_temp_list[stepNum][road]
    std::vector<double> built_length_list;

    void printPath() {
        for (int i = 0; i < size; i++) {
            std::cout << "stepNum " << i << ": ";
            for (int j = 0; j < path[i].size(); j++) {
                std::cout << "(" << path[i].coord[j].x << ", " << path[i].coord[j].y << ") ";
            }
            std::cout << std::endl;
        }
    }
};

double calculate_distance_3D(const Coord& a, const Coord& b, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = (soil_amount[a.x][a.y] - soil_amount[b.x][b.y]) / (GRID_SIZE * GRID_SIZE * GRID_SIZE);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void checkSoilAmountTotal(const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    double total_soil_amount = 0;
    for (int i = 0; i < GRID_SIZE_X; i++) {
        for (int j = 0; j < GRID_SIZE_Y; j++) {
            total_soil_amount += soil_amount[i][j];
        }
    }
    if(total_soil_amount != 0){
        std::cout << "Error: sum of soil_amount is not zero" << std::endl;
        std::abort();
    }
}

TemporaryRoads initiallizedTemoraryRoadsStatus(Solution solution) {
    TemporaryRoads temps;
    temps.roadNum = solution.roadNum;
    temps.coordpair = solution.coordpair;
    temps.statuslist.resize(solution.roadNum, {{0, 0}});
    return temps;
}

Result initiallizedResult(Solution solution){
    Result result;
    result.size = solution.stepNum;
    result.path.resize(solution.stepNum);
    result.used_road_flow.resize(solution.stepNum);
    for (int i = 0; i < solution.stepNum; i++) {
        result.used_road_flow[i].resize(solution.roadNum);
        for (int j = 0; j < solution.roadNum; j++) {
            result.used_road_flow[i][j] = 0;
        }
    }
    result.built_length_list.resize(solution.roadNum, 0.0);
    return result;
}

Solution initializedSolution(const int& stepNum,const int& roadNum,const std::vector<CoordPair> coordpair) {
    // Initialize solution
    Solution solution;
    solution.roadNum = roadNum;
    solution.stepNum = stepNum;
    for (int i = 0; i < roadNum; i++) {
        solution.coordpair.push_back(coordpair[i]);
    }
    solution.timings = std::vector<std::vector<int>>(roadNum, std::vector<int>(stepNum, 0));
    return solution;
}

void setAllTimingtoOne(Solution& solution) {
    for (int i = 0; i < solution.roadNum; i++) {
        for (int j = 0; j < solution.stepNum; j++) {
            solution.timings[i][j] = 1;
        }
    }
}

TemporaryRoads initializedTemporaryRoads(const int& roadNum,const std::vector<CoordPair> coordpair) {
        // Initialize current_temps
        TemporaryRoads temp;
        temp.roadNum = roadNum;
        for (int i = 0; i < roadNum; i++) {
            temp.coordpair.push_back(coordpair[i]);
        }
        temp.statuslist.resize(roadNum, {{0, 0}});
        return temp;
}

// 整数乱数を生成する関数
int generateRandomInt(int min, int max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(min, max);
    return dist(gen);
}

// 実数乱数を生成する関数
double generateRandomDouble(double min, double max) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(min, max);
    return dist(gen);
}

// 重み付き乱数を生成する関数
size_t generateWeightedRandomIndex(const std::vector<double>& weights) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> dist(weights.begin(), weights.end());
    return dist(gen);
}

void removeTemporaryRoads(const Allocation& allocation, TemporaryRoads& temps) {
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


void changeSoil(const Allocation& allocation, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    soil_amount[allocation.start.x][allocation.start.y] -= allocation.volume;
    soil_amount[allocation.goal.x][allocation.goal.y] += allocation.volume;
}

std::vector<int> buildNewRoad(
    TemporaryRoads& current_temps, 
    const int stepNum, 
    const Solution& solution, 
    std::vector<double>& built_length_list, 
    const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], 
    double& built_length
) {
    std::vector<int> builtRoadList;
    int road_count = current_temps.roadNum;
    for (int i = 0; i < road_count; i++) {
        CoordPair road = solution.coordpair[i];
        if (solution.timings[i][stepNum] == 1) {
            double length = calculate_distance_3D(road.coords[0], road.coords[1], soil_amount) / 2;
            if (current_temps.statuslist[i].status[0] == 0&&current_temps.statuslist[i].status[1] == 0) {
                built_length += length*2;
                built_length_list[i] += length*2;
                current_temps.statuslist[i].status[0] = 1;
                current_temps.statuslist[i].status[1] = 1;
                builtRoadList.push_back(i);
            }else if (current_temps.statuslist[i].status[1] == 0||current_temps.statuslist[i].status[1] == 0) {
                built_length += length;
                built_length_list[i] += length;
                current_temps.statuslist[i].status[0] = 1;
                current_temps.statuslist[i].status[1] = 1; 
                builtRoadList.push_back(i);
            }
        }
    }
    return builtRoadList;
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


void normalize_pair(CoordPair& pair) {
    CoordPair newPair;
    if (pair.coords[0].x < pair.coords[1].x || (pair.coords[0].x == pair.coords[1].x && pair.coords[0].y < pair.coords[1].y)) {
        newPair.coords[0] = pair.coords[0];
        newPair.coords[1] = pair.coords[1];
    } else {
        newPair.coords[0] = pair.coords[1];
        newPair.coords[1] = pair.coords[0];
    }
    pair = newPair;
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
    total_cost = 0;

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
    total_cost = cost_so_far[goal.x][goal.y] * volume/ V_truck / TRUCK_NUM; //往復回数込みのコスト

}



std::tuple<Result,double,double> process_allocations(std::vector<Allocation>& allocations,
                        const Solution& solution,
                        double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
                        ) {
    // Initialize TemporaryRoads
    auto temps = initiallizedTemoraryRoadsStatus(solution);
    double operate_cost = 0;
    double built_length = 0;
    auto result = initiallizedResult(solution);
    int stepNum = solution.stepNum;

    for (int i = 0; i < stepNum; ++i) {
        double cost = 0;
        double built_length_i = 0;
        buildNewRoad(temps, i, solution, result.built_length_list, soil_amount, built_length_i);
        built_length += built_length_i;
        astar(allocations[i], temps, soil_amount, result.used_road_flow[i], cost, result.path[i]);
        operate_cost += cost;
        // Remove temporary roads
        removeTemporaryRoads(allocations[i], temps);
        // Change soil
        changeSoil(allocations[i], soil_amount);
    }
    return {result, operate_cost, built_length};
}

double cost_calculation( double operate_cost,double built_length) {
    // 運搬距離コスト
    double time_average = operate_cost * GRID_SIZE / (VELOCITY_ROUGH * 1000.0); // 所要時間 (h)
    double cost_construction = COST_HOUR * time_average / WORK_EFF;
    // 仮設道路の建設コスト
    double cost_road = built_length * CONSTRUCTION_TEMP * GRID_SIZE;
    return cost_construction + cost_road;
}

std::vector<std::vector<int>> initiallizedConnectLabelSet(){
    std::vector<std::vector<int>> connectLabelSet(GRID_SIZE_X * GRID_SIZE_Y);    
    for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; ++i) {
        connectLabelSet[i] = {i};
    }
    return connectLabelSet;
}

//仮設道路が立てられたときに，セルの接続状況を管理するための集合を更新．接続しているセルのラベルを同じにする．
void updateConnectLabelSet(const TemporaryRoads& temps, std::vector<std::vector<int>> connectLabelSet)
{
    for(int i=0;i<temps.roadNum;i++){
        if(temps.statuslist[i].status[0] == 1 && temps.statuslist[i].status[1] == 1){
            int label1 = temps.coordpair[i].coords[0].x * GRID_SIZE_Y + temps.coordpair[i].coords[0].y;
            int label2 = temps.coordpair[i].coords[1].x * GRID_SIZE_Y + temps.coordpair[i].coords[1].y;
            // label1が属する集合を見つける
            auto itLabel1 = std::find_if(connectLabelSet.begin(), connectLabelSet.end(), 
                                        [label1](const std::vector<int>& set) {
                                            return std::find(set.begin(), set.end(), label1) != set.end();
                                        });
            // label2が属する集合を見つける
            auto itLabel2 = std::find_if(connectLabelSet.begin(), connectLabelSet.end(), 
                                        [label2](const std::vector<int>& set) {
                                            return std::find(set.begin(), set.end(), label2) != set.end();
                                        });
        
            // 両方の集合が見つかった場合にマージを行う
            if (itLabel1 != connectLabelSet.end() && itLabel2 != connectLabelSet.end() && itLabel1 != itLabel2) {
                // label2の集合をlabel1の集合にマージ
                itLabel1->insert(itLabel1->end(), itLabel2->begin(), itLabel2->end());

                // 重複を削除
                std::sort(itLabel1->begin(), itLabel1->end());
                itLabel1->erase(std::unique(itLabel1->begin(), itLabel1->end()), itLabel1->end());

                // label2の集合を削除
                connectLabelSet.erase(itLabel2);
            }
        }
    }
}


bool isConnectedtoEntrance(const std::vector<std::vector<int>> connectLabelVector,const Allocation& allocation)
{
    if(connectLabelVector[allocation.start.y][allocation.start.x] == connectLabelVector[entranceYPostion][entranceXPostion]||
       connectLabelVector[allocation.goal.y][allocation.goal.x] == connectLabelVector[entranceYPostion][entranceXPostion])
    {
        return true;
    }
    return false;
}


Solution connectToEntrance(const std::vector<Allocation> allocations,double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],const Solution& solution)
{
    Solution new_solution = solution;
    TemporaryRoads temps = initiallizedTemoraryRoadsStatus(solution);
    std::vector<double> built_length_list(solution.roadNum, 0.0);
    std::vector<int> used_temp_list(solution.roadNum, 0);
    
    auto result = initiallizedResult(solution);
    double built_length = 0.0;
    for (int i = 0; i < solution.stepNum;i++){
        auto connectLabelSet = initiallizedConnectLabelSet();
        auto soil_amount_copy= soil_amount;
        auto builtRoadList =  buildNewRoad(temps, 0, solution, built_length_list, soil_amount, built_length);
        updateConnectLabelSet(temps, connectLabelSet);
        if(!isConnectedtoEntrance(connectLabelSet,allocations[i]))
        {
            double connectRoadLength = 0;
            astar(allocations[i], temps, soil_amount_copy, used_temp_list, connectRoadLength, result.path[i]);
            std::cout << "connetRoadLength: " << connectRoadLength << std::endl;
        }
    }
    return new_solution;
}

// evaluate_design 関数
std::tuple<Result,double> evaluate_design(
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
    const Solution& solution
) {
    auto [path_list, cost_operate, built_length] = process_allocations(allocations, solution, soil_amount);
    double total_cost = cost_calculation(cost_operate,built_length);
    return {path_list, total_cost};
}

// std::tuple<Solution,double> generate_neighbor(
//     const Solution& current_solution,
//     Result& result,
//     std::vector<Allocation> allocations,
//     double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
//     ) {   
//     Solution neighbor_solution = current_solution;
//     return {neighbor_solution,total_cost};
// }

std::tuple<Solution,double> generate_neighbor(
    const Solution& current_solution,
    Result& result,
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
    ) {   
    Solution neighbor_solution = current_solution;

    const std::vector<std::pair<int, int>>& DIRECTIONS = {{-1,-1},{-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
    // stepNum 1: Modify coordpair in the solution
    int modification_type = generateRandomInt(0, 2);

    if (modification_type == 0 || neighbor_solution.coordpair.empty()) {
        std::cout << "Addition" << std::endl;
        // Addition: Add a new random adjacent coordinate pair
        while (true) {
            Coord coord1 = {generateRandomInt(0, GRID_SIZE_X - 1), generateRandomInt(0, GRID_SIZE_Y - 1)};
            auto [dx, dy] = DIRECTIONS[generateRandomInt(0, DIRECTIONS_COUNT - 1)];
            Coord coord2 = {coord1.x + dx, coord1.y + dy};
            // Ensure the new pair is within grid bounds and does not duplicate existing pairs
            if (0 <= coord2.x && coord2.x < GRID_SIZE_X &&
                0 <= coord2.y && coord2.y < GRID_SIZE_Y ){
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
                    neighbor_solution.coordpair.push_back(search_pair);
                    neighbor_solution.timings.push_back(std::vector<int>(allocations.size(), 1)); // Add corresponding timing
                    neighbor_solution.roadNum++;
                    break;
                }
            }
        }
  
        for (size_t i = 0;i<result.size;i++){
            result.used_road_flow[i].push_back(0);
        }
        result.built_length_list.push_back(0);
    } else if (modification_type == 1) {
        // Deletion: Remove a random coordinate pair weighted by its usage and length
            std::vector<double> weights(neighbor_solution.coordpair.size(), 0.0);
            double total_weight = 0.0;
            for (int i = 0; i < neighbor_solution.roadNum; ++i) {
                double usage = result.used_road_flow[0][i];
                for (int j = 1; j < result.used_road_flow.size(); ++j) {
                    usage += result.used_road_flow[j][i];
                }
                if(result.built_length_list[i] == 0)
                {
                double random = generateRandomDouble(0.0, 1.0);
                if (random < 0.05) 
                {weights[i] = 0;
                } else{
                weights[i] = std::numeric_limits<double>::max();} 
                }
                else{weights[i] = result.built_length_list[i] / (usage + 1e-6); }// Weight is inversely proportional to usage
                total_weight += weights[i];
            }
            // Normalize weights
            if (total_weight ==0){total_weight = 1;}
            for (auto& weight : weights) {
                weight /= total_weight;
            }
            // Generate random index based on weighted distribution
            size_t to_remove = generateWeightedRandomIndex(weights);
            neighbor_solution.coordpair.erase(neighbor_solution.coordpair.begin() + to_remove);
            neighbor_solution.timings.erase(neighbor_solution.timings.begin() + to_remove);  
            neighbor_solution.roadNum--;  
            for (int i = 0;i<result.size;i++){
            result.used_road_flow[i].erase(result.used_road_flow[i].begin() + to_remove);
            }
            result.built_length_list.erase(result.built_length_list.begin() + to_remove);
    } else if (modification_type == 2) {
    // Reassignment: Modify a random coordinate pair, weighted by usage and length
        // Calculate weights for each coordinate pair
        std::vector<double> weights(neighbor_solution.coordpair.size(), 0.0);
        double total_weight = 0.0;
            for (int i = 0; i < neighbor_solution.roadNum; ++i) {
                double usage = result.used_road_flow[0][i];
                for (int j = 1; j < result.used_road_flow.size(); ++j) {
                    usage += result.used_road_flow[j][i];}
                if(result.built_length_list[i] == 0){
                double random = generateRandomDouble(0.0, 1.0);
                if (random < 0.05) 
                {weights[i] = 0;
                } else{weights[i] = std::numeric_limits<double>::max();} 
            }else{weights[i] = result.built_length_list[i] / (usage + 1e-6); }// Weight is inversely proportional to usage
                total_weight += weights[i];
            }
        // Normalize weights
        if (total_weight == 0) {total_weight = 1;}
        for (auto& weight : weights) {
            weight /= total_weight;
        }

        // Generate random index based on weighted distribution
        size_t to_modify = generateWeightedRandomIndex(weights);
        // Modify the selected coordinate pair
        Coord coord1 = {generateRandomInt(0, GRID_SIZE_X - 1), generateRandomInt(0, GRID_SIZE_Y - 1)};
        auto [dx, dy] = DIRECTIONS[generateRandomInt(0, DIRECTIONS_COUNT - 1)];
        Coord coord2 = {coord1.x + dx, coord1.y + dy};

        // Ensure the new coord2 is within bounds
        while (0 > coord2.x || coord2.x >= GRID_SIZE_X || 0 > coord2.y || coord2.y >= GRID_SIZE_Y) {
            // std::cout << "Reassignment while loop" << std::endl;
            coord1 = {generateRandomInt(0, GRID_SIZE_X - 1), generateRandomInt(0, GRID_SIZE_Y - 1)};
            auto [dx, dy] = DIRECTIONS[generateRandomInt(0, DIRECTIONS_COUNT - 1)];
            coord2 = {coord1.x + dx, coord1.y + dy};
        }
        auto coordpair = normalize_pair(coord1, coord2);
        // Replace the selected coordinate pair with the new one
        neighbor_solution.coordpair[to_modify] = coordpair;
        }


    // stepNum 2: Set all timings to 1 and evaluate the solution
    setAllTimingtoOne(neighbor_solution);
    auto [local_result,local_operate_cost,local_builtlength] = process_allocations(allocations, neighbor_solution, soil_amount);

    // stepNum 3: Adjust timings based on used_temp_list
    for (size_t i = 0; i < neighbor_solution.stepNum; ++i) {
        for (size_t j = 0; j < neighbor_solution.roadNum; ++j) {
            if (local_result.used_road_flow[i][j] == 0) {
                neighbor_solution.timings[j][i] = 0;
            }
            else {
                neighbor_solution.timings[j][i] = 1;
            }
        }
    }
    auto [new_result,total_cost] = evaluate_design(allocations,soil_amount,neighbor_solution);
    // result = new_result;

    // auto [new_result,total_cost] = connectToEntrance(allocations,soil_amount, neighbor_solution);

    return {neighbor_solution,total_cost};
}

std::tuple<Solution,double,std::vector<Path>> local_search(
    const Solution& current_solution,
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
    ) 
{   
    Solution neighbor_solution = current_solution;

    const std::vector<std::pair<int, int>>& DIRECTIONS = {{-1,-1},{-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
    
    // stepNum 2: Set all timings to 1 and evaluate the solution
    for (auto& timing : neighbor_solution.timings) {
        std::fill(timing.begin(), timing.end(), 1);
    }
    auto [local_result,local_operate_cost,local_builtlength] = process_allocations(allocations, neighbor_solution, soil_amount);
    // Print local_result.used_road_flow
    std::cout << std::endl;
   
    // stepNum 3: Adjust timings based on used_temp_list
    for (size_t i = 0; i < neighbor_solution.stepNum; ++i) {
        for (size_t j = 0; j < neighbor_solution.roadNum; ++j) {
            if (local_result.used_road_flow[i][j] == 0) {
                neighbor_solution.timings[j][i] = 0;
            }
            else {
                neighbor_solution.timings[j][i] = 1;
            }
        }
    }
    auto [new_result,total_cost] = evaluate_design(allocations,soil_amount,neighbor_solution);
    return {neighbor_solution,total_cost,new_result.path};
}

void simulated_annealing(std::vector<Allocation>& allocations, const Solution& initSolution, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    Solution current_solution = initSolution;
    //setAllTimingtoOne(current_solution);
    auto [current_result,current_score] = evaluate_design(allocations,soil_amount, current_solution);

    Solution best_solution = current_solution;
    double best_score = current_score;
    Result best_result = current_result;
    int best_score_loop = 0;

    // std::vector<std::pair<double, Solution>> best_score_flow;
    // std::vector<std::pair<double, Solution>> current_score_flow;
    // std::vector<std::pair<double, Solution>> neighbor_score_flow;
    // std::vector<std::pair<double, Solution>> neighbor_solution_flow;

    int temperature = initialTemperature;
    for (int iter = 0; iter < max_iter; ++iter) {
        //reset soil_amount copy
        double soil_amount_copy[GRID_SIZE_X][GRID_SIZE_Y];
        for (int i = 0; i < GRID_SIZE_X; i++) {
            for (int j = 0; j < GRID_SIZE_Y; j++) {
                soil_amount_copy[i][j] = soil_amount[i][j];
            }
        }
        auto neighbor_result = current_result;
        temperature *= alpha;

        //print current solution and result
        auto [neighbor_solution,neighbor_score] = generate_neighbor(current_solution,neighbor_result, allocations, soil_amount_copy);
        neighbor_solution.printParameters();
        
        double random_value = generateRandomDouble(0.0, 1.0);
        // 受け入れ判定
        if ((neighbor_score < current_score) || 
            (random_value < std::exp(-(std::abs(neighbor_score - current_score)) / temperature))) {
            current_solution = neighbor_solution;
            current_score = neighbor_score;
            current_result = neighbor_result;
        }

        // ベスト解の更新
        if (current_score < best_score) {
            best_solution = current_solution;
            best_score = current_score;
            best_score_loop = iter;
            best_result = current_result;
        }
    }

    //print best solution and result
    std::cout << "best_score: " << best_score << std::endl;
    best_solution.printParameters();
    best_result.printPath();
    std::cout << "best_score_loop: " << best_score_loop << std::endl;

}

int main() {
    auto start = std::chrono::high_resolution_clock::now();
    
    //Initialize soil_amount
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y] =
    {
        {-15000.0, -62600.0, -3700.0, 0.0},
        {-3700.0, 22500.0, 22500.0, -1400.0},
        {3700.0, 33800.0, 28100.0, 2300.0},
        {9000.0, 36000.0, 23000.0, 1200.0},
        {9000.0, 23000.0, 24300.0, 9000.0},
        {8000.0, 22200.0, 14200.0, -5900.0},
        {-1000.0, 8100.0, -12400.0, -9900.0},
        {-11200.0, -24800.0, -34400.0, -2700.0},
        {-2300.0, -9900.0, -72500.0, 2300.0},
        {-22000.0, -2200.0, -28500.0, -100.0},
        {-6900.0, 14300.0, -2500.0, 0.0},
        {11200.0, 7900.0, 0.0, 0.0}
    };
    // double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]  =
    // {
    //     {-2,-1,-1,-1},
    //     {1,-1,1,-2},
    //     {1,1,1,1},
    //     {1,1,-2,2}
    // };
    //check if sum of soil_amount is zero
    checkSoilAmountTotal(soil_amount);
    // Initialize allocations
    std::vector<Allocation> allocations = 
    {
        {{11, 1}, {10, 2}, 2500.0},
        {{11, 1}, {9, 2}, 5400.0},
        {{11, 0}, {10, 0}, 6900.0},
        {{11, 0}, {9, 0}, 4300.0},
        {{10, 1}, {9, 2}, 12100.0},
        {{10, 1}, {9, 1}, 2200.0},
        {{8, 3}, {9, 3}, 100.0},
        {{8, 3}, {9, 2}, 2200.0},
        {{3, 1}, {9, 2}, 8800.0},
        {{5, 0}, {9, 0}, 8000.0},
        {{4, 0}, {9, 0}, 4000.0},
        {{3, 0}, {9, 0}, 5700.0},
        {{5, 2}, {8, 2}, 14200.0},
        {{4, 2}, {8, 2}, 24300.0},
        {{4, 1}, {8, 2}, 17400.0},
        {{3, 1}, {8, 2}, 16600.0},
        {{6, 1}, {8, 1}, 8100.0},
        {{3, 1}, {8, 1}, 1800.0},
        {{3, 0}, {8, 0}, 2300.0},
        {{2, 2}, {7, 3}, 2700.0},
        {{4, 1}, {7, 2}, 5600.0},
        {{3, 2}, {7, 2}, 22100.0},
        {{2, 2}, {7, 2}, 6700.0},
        {{5, 1}, {7, 1}, 22200.0},
        {{3, 1}, {7, 1}, 2600.0},
        {{4, 0}, {7, 0}, 5000.0},
        {{3, 1}, {7, 0}, 6200.0},
        {{4, 3}, {6, 3}, 9000.0},
        {{3, 2}, {6, 3}, 900.0},
        {{2, 2}, {6, 2}, 12400.0},
        {{3, 0}, {6, 0}, 1000.0},
        {{3, 3}, {5, 3}, 1200.0},
        {{2, 3}, {5, 3}, 900.0},
        {{2, 2}, {5, 3}, 3800.0},
        {{2, 3}, {1, 3}, 1400.0},
        {{2, 2}, {0, 1}, 2500.0},
        {{1, 2}, {0, 2}, 3700.0},
        {{2, 1}, {1, 0}, 3700.0},
        {{2, 1}, {0, 1}, 30100.0},
        {{1, 2}, {0, 1}, 18800.0},
        {{2, 0}, {0, 0}, 3700.0},
        {{1, 1}, {0, 1}, 11200.0},
        {{1, 1}, {0, 0}, 11300.0}
    };

    // Initialize Solution
    int stepNum = allocations.size();
    // std::vector<CoordPair> coordpair ={};


//     CoordPair coordpair1 = {{{0, 2},{1, 2}}};
//     CoordPair coordpair2 = {{{1, 2},{2, 2}}};
//     CoordPair coordpair3 = {{{2, 2},{3, 2}}};
//     CoordPair coordpair4 = {{{3, 2},{4, 2}}};
//     CoordPair coordpair5 = {{{4, 2},{5, 2}}};
//     CoordPair coordpair6 = {{{5, 2},{6, 2}}};
//     CoordPair coordpair7 = {{{6, 2},{7, 2}}};
//     CoordPair coordpair8 = {{{4, 2},{3, 3}}};
//     CoordPair coordpair9 = {{{4, 2},{3, 1}}};
//     CoordPair coordpair10 = {{{5, 2},{5, 1}}};
//     CoordPair coordpair11 = {{{5, 2},{5, 3}}};
//     CoordPair coordpair12 = {{{6, 2},{7, 1}}};
//     CoordPair coordpair13 = {{{7, 1},{8, 1}}};
//     CoordPair coordpair14 = {{{8, 1},{9, 1}}};
//     CoordPair coordpair15 = {{{9, 1},{10, 1}}};

//     //normalize coordpair
//    normalize_pair(coordpair1);
//    normalize_pair(coordpair2);
//    normalize_pair(coordpair3);
//    normalize_pair(coordpair4);
//    normalize_pair(coordpair5);
//    normalize_pair(coordpair6);
//    normalize_pair(coordpair7);
//    normalize_pair(coordpair8);
//     normalize_pair(coordpair9);
//     normalize_pair(coordpair10);
//     normalize_pair(coordpair11);
//     normalize_pair(coordpair12);
//     normalize_pair(coordpair13);
//     normalize_pair(coordpair14);
//     normalize_pair(coordpair15);


//     std::vector<CoordPair> coordpairlist;
//     coordpairlist.push_back(coordpair1);
//     coordpairlist.push_back(coordpair2);
//     coordpairlist.push_back(coordpair3);
//     coordpairlist.push_back(coordpair4);
//     coordpairlist.push_back(coordpair5);
//     coordpairlist.push_back(coordpair6);
//     coordpairlist.push_back(coordpair7);
//     coordpairlist.push_back(coordpair8);
//     coordpairlist.push_back(coordpair9);
//     coordpairlist.push_back(coordpair10);
//     coordpairlist.push_back(coordpair11);
//     coordpairlist.push_back(coordpair12);
//     coordpairlist.push_back(coordpair13);
//     coordpairlist.push_back(coordpair14);
//     coordpairlist.push_back(coordpair15);

//     CoordPair coordpair1 = {{{0, 2},{1, 2}}};
//     CoordPair coordpair2 = {{{1, 2},{2, 2}}};
//     CoordPair coordpair3 = {{{2, 2},{3, 2}}};
//     CoordPair coordpair4 = {{{3, 2},{4, 2}}};
//     CoordPair coordpair5 = {{{4, 2},{5, 2}}};
//     CoordPair coordpair6 = {{{5, 2},{6, 2}}};
//     CoordPair coordpair7 = {{{6, 2},{7, 1}}};
//     CoordPair coordpair8 = {{{7, 1},{8, 1}}};
//     CoordPair coordpair9 = {{{8, 1},{9, 1}}};
//     CoordPair coordpair10 = {{{9, 1},{10, 1}}};

//     //normalize coordpair
//    normalize_pair(coordpair1);
//    normalize_pair(coordpair2);
//    normalize_pair(coordpair3);
//    normalize_pair(coordpair4);
//    normalize_pair(coordpair5);
//    normalize_pair(coordpair6);
//    normalize_pair(coordpair7);
//    normalize_pair(coordpair8);
//     normalize_pair(coordpair9);
//     normalize_pair(coordpair10);

//     std::vector<CoordPair> coordpairlist;
//     coordpairlist.push_back(coordpair1);
//     coordpairlist.push_back(coordpair2);
//     coordpairlist.push_back(coordpair3);
//     coordpairlist.push_back(coordpair4);
//     coordpairlist.push_back(coordpair5);
//     coordpairlist.push_back(coordpair6);
//     coordpairlist.push_back(coordpair7);
//     coordpairlist.push_back(coordpair8);
//     coordpairlist.push_back(coordpair9);
//     coordpairlist.push_back(coordpair10);


    CoordPair coordpair1 = {{{1, 2},{2, 2}}};
    CoordPair coordpair2 = {{{2, 2},{3, 2}}};
    CoordPair coordpair3 = {{{3, 2},{4, 2}}};
    CoordPair coordpair4 = {{{4, 2},{5, 2}}};
    CoordPair coordpair5 = {{{5, 2},{6, 2}}};
    CoordPair coordpair6 = {{{6, 2},{7, 2}}};
    CoordPair coordpair7 = {{{7, 2},{8, 2}}};

    CoordPair coordpair8 = {{{0, 1},{1, 1}}};
    CoordPair coordpair9 = {{{1, 1},{2, 1}}};
    CoordPair coordpair10 = {{{2, 1},{3, 1}}};

    CoordPair coordpair11 = {{{7, 1},{8, 1}}};
    CoordPair coordpair12 = {{{8, 1},{9, 1}}};
    CoordPair coordpair13 = {{{9, 1},{10, 1}}};

    //normalize coordpair
    normalize_pair(coordpair1);
    normalize_pair(coordpair2);
    normalize_pair(coordpair3);
    normalize_pair(coordpair4);
    normalize_pair(coordpair5);
    normalize_pair(coordpair6);
    normalize_pair(coordpair7);
    normalize_pair(coordpair8);
    normalize_pair(coordpair9);
    normalize_pair(coordpair10);
    normalize_pair(coordpair11);
    normalize_pair(coordpair12);
    normalize_pair(coordpair13);

    std::vector<CoordPair> coordpairlist;
    coordpairlist.push_back(coordpair1);
    coordpairlist.push_back(coordpair2);
    coordpairlist.push_back(coordpair3);
    coordpairlist.push_back(coordpair4);
    coordpairlist.push_back(coordpair5);
    coordpairlist.push_back(coordpair6);
    coordpairlist.push_back(coordpair7);
    coordpairlist.push_back(coordpair8);
    coordpairlist.push_back(coordpair9);
    coordpairlist.push_back(coordpair10);
    coordpairlist.push_back(coordpair11);
    coordpairlist.push_back(coordpair12);
    coordpairlist.push_back(coordpair13);

    int roadNum = coordpairlist.size(); 
    Solution initSolution = initializedSolution(stepNum, roadNum, coordpairlist);

    simulated_annealing(allocations, initSolution, soil_amount);

    auto end = std::chrono::high_resolution_clock::now();
    // 経過時間を取得 (ミリ秒)
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "total time: " << duration.count() << " ms" << std::endl;
    return 0;
}
