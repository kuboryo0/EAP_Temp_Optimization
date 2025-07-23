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
#include <stack>

// Objective function parameters
#define GRID_SIZE_X 12
#define GRID_SIZE_Y 4
#define DIRECTIONS_COUNT 8
#define INF std::numeric_limits<double>::infinity()


// #define VELOCITY_ROUGH 24.0
#define VELOCITY_ROUGH 10.0
#define VELOCITY_PAVE 40.0
#define VELOCITY_SHOVEL 5.0
#define TEMP_EFF VELOCITY_ROUGH/VELOCITY_PAVE // Temporary road efficiency
#define GRID_SIZE 150
#define V_TRUCK 40 //[m^3]
#define TRUCK_NUM 1
#define COST_HOUR 2000.0
#define WORK_EFF 0.75
#define CONSTRUCTION_TEMP 17.5
#define CONSTRUCTION_SUB_TEMP 4.0
#define CONSTRUCTION_TEMP_DIFF (CONSTRUCTION_TEMP - CONSTRUCTION_SUB_TEMP)

#define entranceXPostion 0
#define entranceYPostion 2
#define entranceIndex entranceXPostion * GRID_SIZE_Y + entranceYPostion

// Simulated Annealing parameters
#define alpha 0.97
#define max_iter 300
#define initialTemperature 10000.0

struct Coord {
    int x, y;
    bool operator==(const Coord& other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template <>
    struct hash<Coord> {
        size_t operator()(const Coord& c) const {
            return hash<int>()(c.x) ^ (hash<int>()(c.y) << 1);
        }
    };
}

const std::vector<Coord> DIRECTIONS = {
    {-1, -1}, {-1, 0}, {-1, 1},
    {0, -1},          {0, 1},
    {1, -1},  {1, 0}, {1, 1}
};

const std::unordered_map<Coord, int> DIRECTION_TO_INDEX = []() {
    std::unordered_map<Coord, int> m;
    for (int i = 0; i < DIRECTIONS.size(); ++i) {
        m[DIRECTIONS[i]] = i;
    }
    return m;
}();


struct CoordPair {
    Coord coords[2];
};

struct TemporaryRoadStatus {
    int status[2];
    CoordPair coordpair;
};



struct TemporaryRoadbuildStatus {
    CoordPair coordpair;
    std::vector<int> timings; //timing[i][j]はi番目の道路のj番目のステップでの建設時間を表す
};

struct Solution {
    int roadNum;
    int stepNum;
    std::vector<TemporaryRoadbuildStatus> roadbuildStatusList;

    void printParameters() {
        std::cout << "roadNum: " << roadNum << std::endl;
        std::cout << "stepNum: " << stepNum << std::endl;
        for (int i = 0; i < roadNum; i++) {
            std::cout << "coordpair[" << i << "]: (" << roadbuildStatusList[i].coordpair.coords[0].x << ", " << roadbuildStatusList[i].coordpair.coords[0].y << "), ("
                      << roadbuildStatusList[i].coordpair.coords[1].x << ", " << roadbuildStatusList[i].coordpair.coords[1].y << ")" << std::endl;
            for (int j = 0; j < stepNum; j++) {
                std::cout << "  timing[" << j << "]: " << roadbuildStatusList[i].timings[j] << std::endl;
            }
        }
    }

    void printParametersForPlot() {
        // std::cout << "roadNum: " << roadNum << std::endl;
        // std::cout << "stepNum: " << stepNum << std::endl;
        std::cout << "[" << std::endl;
        for (int i = 0; i < roadNum; i++) {
             //もしtimingsが全て0ならば出力しない
            bool allZero = true;
            for (int j = 0; j < stepNum; j++) {
                if (roadbuildStatusList[i].timings[j] != 0) {
                    allZero = false;
                    break;
                }
            }
            if (allZero) {
                continue; // 全て0ならば出力しない
            }

            std::cout << "[[(" << roadbuildStatusList[i].coordpair.coords[0].x << ", " << roadbuildStatusList[i].coordpair.coords[0].y << "), ("
                      << roadbuildStatusList[i].coordpair.coords[1].x << ", " << roadbuildStatusList[i].coordpair.coords[1].y << ")], [";
            std::cout << roadbuildStatusList[i].timings[0]  ;
            for (int j = 1; j < stepNum; j++) {
                std::cout << ", "<<roadbuildStatusList[i].timings[j];
            }
            std::cout << "]]," << std::endl;
        }
        std::cout << "]"<<std::endl;
    }

    void printParametersModified() {
        std::cout << "roadNum: " << roadNum << std::endl;
        std::cout << "stepNum: " << stepNum << std::endl;
        for (int i = 0; i < roadNum; i++) {
            //もしtimingsが全て0ならば出力しない
            bool allZero = true;
            for (int j = 0; j < stepNum; j++) {
                if (roadbuildStatusList[i].timings[j] != 0) {
                    allZero = false;
                    break;
                }
            }
            if (allZero) {
                continue; // 全て0ならば出力しない
            }

            std::cout << "coordpair[" << i << "]: (" << roadbuildStatusList[i].coordpair.coords[0].x << ", " << roadbuildStatusList[i].coordpair.coords[0].y << "), ("
                      << roadbuildStatusList[i].coordpair.coords[1].x << ", " << roadbuildStatusList[i].coordpair.coords[1].y << ")" << std::endl;
            for (int j = 0; j < stepNum; j++) {
                std::cout << "  timing[" << j << "]: " << roadbuildStatusList[i].timings[j] << std::endl;
            }
        }
    }
};

struct TemporaryRoads {
    int roadNum;
    std::vector<TemporaryRoadStatus> roadStatusList;

    void initialize(const Solution & solution) {
        roadNum = solution.roadNum;
        roadStatusList.resize(roadNum);
        for (int i = 0; i < roadNum; i++) {
            roadStatusList[i].coordpair = solution.roadbuildStatusList[i].coordpair;
            roadStatusList[i].status[0] = 0;
            roadStatusList[i].status[1] = 0;
        }
    }
    //道路がすべて建設された状態にする
    void setStatusToOne() {
        for (int i = 0; i < roadNum; i++) {
            roadStatusList[i].status[0] = 1;
            roadStatusList[i].status[1] = 1;
        }
    }

    void printParameters() {
        std::cout << "roadNum: " << roadNum << std::endl;
        for (int i = 0; i < roadNum; i++) {
            std::cout << "coordpair[" << i << "]: (" << roadStatusList[i].coordpair.coords[0].x << ", " << roadStatusList[i].coordpair.coords[0].y << "), (" 
                      << roadStatusList[i].coordpair.coords[1].x << ", " << roadStatusList[i].coordpair.coords[1].y << ")" << std::endl;
            std::cout << "status[" << i << "]: (" << roadStatusList[i].status[0] << ", " << roadStatusList[i].status[1] << ")" << std::endl;
        }
    }
};

struct Allocation {
    Coord start;
    Coord goal;
    double volume;
        bool operator==(const Allocation& other) const {
        return start.x == other.start.x &&
               start.y == other.start.y &&
               goal.x == other.goal.x &&
               goal.y == other.goal.y &&
               volume == other.volume;
    }
};

struct AllocationOrder {
    std::vector <Allocation> formerAllocations;
    std::vector <Allocation> latterAllocations;

    void printParameters() {
        std::cout << "formerAllocations: " << std::endl;
        for (const auto& allocation : formerAllocations) {
            std::cout << "start: (" << allocation.start.x << ", " << allocation.start.y << "), goal: (" 
                      << allocation.goal.x << ", " << allocation.goal.y << "), volume: " << allocation.volume << std::endl;
        }
        std::cout << "latterAllocations: " << std::endl;
        for (const auto& allocation : latterAllocations) {
            std::cout << "start: (" << allocation.start.x << ", " << allocation.start.y << "), goal: (" 
                      << allocation.goal.x << ", " << allocation.goal.y << "), volume: " << allocation.volume << std::endl;
        }
    }

    void printParametersForPlot() {
        std::cout << "[" << std::endl;
        for (const auto& allocation : formerAllocations) {
            std::cout << "[(" << allocation.start.x << ", " << allocation.start.y << "), (" 
                      << allocation.goal.x << ", " << allocation.goal.y << "), " << allocation.volume << "]," << std::endl;
        }
        for (const auto& allocation : latterAllocations) {
            std::cout << "[(" << allocation.start.x << ", " << allocation.start.y << "), (" 
                      << allocation.goal.x << ", " << allocation.goal.y << "), " << allocation.volume << "]," << std::endl;
        }
        std::cout << "]" << std::endl;
    }

    std::vector<Allocation> orderToVector() const {
        std::vector<Allocation> orderedAllocations;
        orderedAllocations.reserve(formerAllocations.size() + latterAllocations.size());
        orderedAllocations.insert(orderedAllocations.end(), formerAllocations.begin(), formerAllocations.end());
        orderedAllocations.insert(orderedAllocations.end(), latterAllocations.begin(), latterAllocations.end());
        return orderedAllocations;
    }
};

struct CoordPairWithTiming {
    CoordPair coordpair;
    std::vector<int> timing;
};

enum RoadType {
    PROHIBITED = -1, // 通行禁止
    NONE = 0, // 道路なし
    UNPAVED = 1,      // 未舗装道路
    PAVED = 2        // 舗装道路
};

struct NeighborInfo {
    Coord direction; // 隣接方向への座標{{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}}
    RoadType type; // 道路の種類
};

struct RoadAroundCenter {
    Coord center; // 中心座標
    std::vector<NeighborInfo> neighbors; // 隣接する座標とその道路の種類
     
    void print() const {
        std::cout << "Center: (" << center.x << ", " << center.y << ")" << std::endl;
        std::cout << "Neighbors: ";
        for (const auto& neighbor : neighbors) {
            std::cout << "(" << neighbor.direction.x << ", " << neighbor.direction.y << ") - Type: " 
                      << static_cast<int>(neighbor.type) << " ";
        }
        std::cout << std::endl;
    }
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
    void print() const {
        std::cout << "coord.size(): " << coord.size() << std::endl;
        std::cout << "Path: ";
        std::cout << "(" << coord[0].x << ", " << coord[0].y << ") ";
        for (size_t i = 1; i < coord.size(); ++i) {
            std::cout << "-> (" << coord[i].x << ", " << coord[i].y << ") ";
        }
        std::cout << std::endl;
    }
};

struct Result {
    int size;  // Number of Steps
    std::vector<Path> path;
    std::vector<std::vector<int>> used_road_flow;  //used_temp_list[stepNum][road]
    std::vector<double> built_length_list;

    void init(const Solution& solution){
        size = solution.stepNum;
        // built_length_list.resize(solution.roadNum,0);
        // used_road_flow.resize(solution.stepNum, std::vector<int>(solution.roadNum, 0)); 
        built_length_list.assign(solution.roadNum, 0); // 初期化済みのベクタを確保
        used_road_flow.assign(solution.stepNum, std::vector<int>(solution.roadNum, 0)); // 二重ベクタを確保
    }

    void printPath() {
        std::cout << "path" << std::endl;
        for (int i = 0; i < size; i++) {
            std::cout << "step[" << i << "]: ";
            std::cout << "(" << path[i].coord[0].x << ", " << path[i].coord[0].y << ")";
            for (int j = 1; j < path[i].size(); j++) {
                std::cout << "->(" << path[i].coord[j].x << ", " << path[i].coord[j].y << ")";
            }
            std::cout << std::endl;
        }
    }

    void printPathForPlot(std::vector<Allocation> allocations) {
        std::cout << "[" << std::endl;
        for (int i = 0; i < size; i++) {

            std::cout << "[[(" << path[i].coord[0].x << ", " << path[i].coord[0].y << ")";
            for (int j = 1; j < path[i].size(); j++) {
                std::cout << ", (" << path[i].coord[j].x << ", " << path[i].coord[j].y << ")";
            }
            std::cout << "], "<< allocations[i].volume << "] ," << std::endl;
        }
        std::cout<<"]" << std::endl;
    }

    void printUsedRoadFlow() {
        std::cout << "used road flow" << std::endl;
        for (int i = 0; i < used_road_flow.size(); i++) {
            std::cout << "step[" << i << "]: ";
            for (int j = 0; j < used_road_flow[i].size(); j++) {
                std::cout << used_road_flow[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }
    void printBuiltLengthList() {
        std::cout << "built_length_list: ";
        for (int i = 0; i < built_length_list.size(); i++) {
            std::cout << built_length_list[i] << " ";
        }
        std::cout << std::endl;
    }

};

struct NodePair{
    int node[2];

    void print(){
        std::cout << "node1:" << node[0] << ", node2:" << node[1] << std::endl;
    }
};

struct connectNetwork {
    std::vector<int> nodeList;  //各ノードを保有してるかを表すビットリスト（例．{0,1,0,1}の場合はノード2と4がつながっている）
    std::vector<std::vector<int>> edgeList; //1つながりのエッジのリスト(例．{{0,0,1},{0,0,0},{0,1,0}}の場合は1-3,3-2のノードがつながっている)

    void printConverted(){
        std::cout << "nodeList(Converted):[";
        std::vector<int> nodeListConverted;
        for (int j = 0;j< nodeList.size(); j++) {
            if(nodeList[j] == 1){
                nodeListConverted.push_back(j);
            }
        }
        for (int j = 0;j< nodeListConverted.size(); j++) {
            std::cout << nodeListConverted[j];
            if (j < nodeListConverted.size() - 1) { // 最後の要素でない場合のみカンマを出力
                std::cout << ", ";
            }
        }
        std::cout << "] edgeList: [";
        std::vector<std::pair<int,int>> edgeListConverted;
        for (int j = 0; j < edgeList.size(); j++) {
            for(int k = 0; k < edgeList[j].size(); k++) {
                if(edgeList[j][k] == 1){
                    edgeListConverted.push_back({j,k});
                }
            }
            }
        for (int j = 0; j < edgeListConverted.size(); j++) {
            std::cout << "(" << edgeListConverted[j].first << "," << edgeListConverted[j].second << ")";
            if (j < edgeListConverted.size() - 1) { // 最後の要素でない場合のみカンマを出力
                std::cout << ", ";
            }
        }
            std::cout << "]" << std::endl;
    }
    
    bool isValid(){
        size_t nodeCount = nodeList.size();

        for (size_t i = 0; i < nodeCount; ++i) {
            if (nodeList[i] == 1) {
                bool hasOutgoingEdge = false;
                for (size_t j = 0; j < nodeCount; ++j) {
                    if (edgeList[i][j] == 1) {
                        hasOutgoingEdge = true;
                        break;
                    }
                }
                if (!hasOutgoingEdge) {
                    return false; // ノードiは接続がない状態
                }
            }
        }
        return true; // すべてのノードが有効
    }
};

struct connectNetworkList {
    int connectNum;
    std::vector<connectNetwork> connectNetworkList;

    // tempsのcoordpair全てが繋がっていない状態で初期化
    void initialize(TemporaryRoads& temps){
        connectNum = temps.roadNum;
        connectNetworkList.resize(connectNum);
        for (int i = 0; i < connectNum; i++) {
            connectNetwork& network = connectNetworkList[i];
            int index1 = temps.roadStatusList[i].coordpair.coords[0].x * GRID_SIZE_Y + temps.roadStatusList[i].coordpair.coords[0].y;
            int index2 = temps.roadStatusList[i].coordpair.coords[1].x * GRID_SIZE_Y + temps.roadStatusList[i].coordpair.coords[1].y;
            // 値が小さい方をindex1に、大きい方をindex2にする
            if (index1 > index2) {
                std::swap(index1, index2);
            }
            //nodelistを0で初期化
            network.nodeList.resize(GRID_SIZE_X * GRID_SIZE_Y, 0);
            network.nodeList[index1] = 1; 
            network.nodeList[index2] = 1;
            network.edgeList.resize(GRID_SIZE_X * GRID_SIZE_Y, std::vector<int>(GRID_SIZE_X * GRID_SIZE_Y, 0));
            network.edgeList[index1][index2] = 1; // index1とindex2がつながっていることを示す
        }
    }

    bool isValid(){
        for(connectNetwork network: connectNetworkList){
            if(network.nodeList.size() != GRID_SIZE_X*GRID_SIZE_Y){
                std::cout << "nodeList size is invalid" << std::endl;
                return false;
            }
            if(!network.isValid()){
                std::cout << "nodeList edgelist is invalid" << std::endl;
                return false;
            }
        }
        
        if(connectNum != connectNetworkList.size()){
            std::cout << "connectNum is invalid" << std::endl;
            return false;
        }


        return true;
    }

    void print() {
        std::cout << "connectNum: " << connectNum << std::endl;
        for (int i = 0; i < connectNum; i++) {
            std::cout << "connectNetwork[" << i << "]: ";
            std::cout << "nodeList:[";
            for (int j = 0;j< connectNetworkList[i].nodeList.size(); j++) {
                std::cout << connectNetworkList[i].nodeList[j];
                if (j < connectNetworkList[i].nodeList.size() - 1) { // 最後の要素でない場合のみカンマを出力
                    std::cout << ",";
                }
            }
            std::cout << "] edgeList: [";
            for (int j = 0; j < connectNetworkList[i].edgeList.size(); j++) {
                for(int k = 0; k < connectNetworkList[i].edgeList[j].size(); k++) {
                    std::cout << connectNetworkList[i].edgeList[j][k];
                    if (k < connectNetworkList[i].edgeList[j].size() - 1) { // 最後の要素でない場合のみカンマを出力
                        std::cout << ",";
                    }
                }
            }
            std::cout << "]" << std::endl;
        }
    }
    void printConverted() {
        std::cout << "connectNum: " << connectNum << std::endl;
        
        for (int i = 0; i < connectNum; i++) {
            std::cout << "connectNetwork[" << i << "]: ";
            std::cout << "nodeList(Converted):[";
            std::vector<int> nodeListConverted;
            for (int j = 0;j< connectNetworkList[i].nodeList.size(); j++) {
                if(connectNetworkList[i].nodeList[j] == 1){
                    nodeListConverted.push_back(j);
                }
            }
            for (int j = 0;j< nodeListConverted.size(); j++) {
                std::cout << nodeListConverted[j];
                if (j < nodeListConverted.size() - 1) { // 最後の要素でない場合のみカンマを出力
                    std::cout << ", ";
                }
            }
            std::cout << "] edgeList: [";
            std::vector<std::pair<int,int>> edgeListConverted;
            for (int j = 0; j < connectNetworkList[i].edgeList.size(); j++) {
                for(int k = 0; k < connectNetworkList[i].edgeList[j].size(); k++) {
                    if(connectNetworkList[i].edgeList[j][k] == 1){
                        edgeListConverted.push_back({j,k});
                    }
                }
            }
            for (int j = 0; j < edgeListConverted.size(); j++) {
                std::cout << "(" << edgeListConverted[j].first << "," << edgeListConverted[j].second << ")";
                if (j < edgeListConverted.size() - 1) { // 最後の要素でない場合のみカンマを出力
                    std::cout << ", ";
                }
            }
            std::cout << "]" << std::endl;
        }
    }
};

//costを計算する際に使用する座標の距離計算関数
//----------------------------------------------
double calculate_distance_3D(const Coord& a, const Coord& b, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    double dz = (soil_amount[a.x][a.y] - soil_amount[b.x][b.y]) / (GRID_SIZE * GRID_SIZE * GRID_SIZE);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double calculate_distance_2D(const Coord& a, const Coord& b, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double heuristic(const Coord& a, const Coord& b, const  double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    return TEMP_EFF * calculate_distance_3D(a, b, soil_amount);
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


double getCost(const Coord& current, const Coord& neighbor, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], const TemporaryRoads& temporary_roads,double temp_eff = TEMP_EFF) {
    double distance = calculate_distance_3D(current, neighbor, soil_amount);
    double current_distance = distance / 2;
    double neighbor_distance = distance / 2;

    CoordPair search_pair = normalize_pair(current, neighbor);
    // auto it = std::find(temporary_roads.coordpair.begin(), temporary_roads.coordpair.end(), search_pair);
    auto it = std::find_if(
        temporary_roads.roadStatusList.begin(),
        temporary_roads.roadStatusList.end(),
        [&search_pair](const TemporaryRoadStatus& road) {
            return (road.coordpair.coords[0].x == search_pair.coords[0].x && road.coordpair.coords[0].y == search_pair.coords[0].y &&
                    road.coordpair.coords[1].x == search_pair.coords[1].x && road.coordpair.coords[1].y == search_pair.coords[1].y);
        });
    if (it != temporary_roads.roadStatusList.end()) {
        size_t index = std::distance(temporary_roads.roadStatusList.begin(), it);
        if (temporary_roads.roadStatusList[index].status[0] == 1 && temporary_roads.roadStatusList[index].status[1] == 1) {
            current_distance *= temp_eff;
            neighbor_distance *= temp_eff;
        } else if (temporary_roads.roadStatusList[index].status[0] == 1 || temporary_roads.roadStatusList[index].status[1] == 1) {
            neighbor_distance *= temp_eff;
        }
    }
    return current_distance + neighbor_distance;
}

double getCost(const Coord& current, const Coord& neighbor, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    double distance = calculate_distance_3D(current, neighbor, soil_amount);
    return distance;
}
//-----------------------------------------------

// 土壌量の合計が0であることを確認する関数
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

//初期化用関数
//----------------------------------------------

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

Solution initializedSolution(const int& stepNum,const std::vector<CoordPair> coordpairlist) {
    // Initialize solution
    Solution solution;
    solution.roadNum = coordpairlist.size();
    solution.stepNum = stepNum;
    for (int i = 0; i < solution.roadNum; i++) {
        TemporaryRoadbuildStatus roadbuildStatus;
        roadbuildStatus.coordpair = coordpairlist[i];
        roadbuildStatus.timings.resize(stepNum, 0);
        solution.roadbuildStatusList.push_back(roadbuildStatus);
    }
    return solution;
}

void setAllTimingtoOne(Solution& solution) {
    for (int i = 0; i < solution.roadNum; i++) {
        for (int j = 0; j < solution.stepNum; j++) {
            solution.roadbuildStatusList[i].timings[j] = 1;
        }
    }
}

TemporaryRoads initializedTemporaryRoads(const int& roadNum,const std::vector<CoordPair> coordpair) {
        // Initialize current_temps
        TemporaryRoads temp;
        temp.roadNum = roadNum;
        for (int i = 0; i < roadNum; i++) {
            TemporaryRoadStatus roadbuildStatus;
            roadbuildStatus.coordpair = coordpair[i];
            roadbuildStatus.status[0] = 0;
            roadbuildStatus.status[1] = 0;
            temp.roadStatusList.push_back(roadbuildStatus);
        }
        return temp;
}
//-------------------------------------------------------------


// 乱数生成関数
//---------------------------------------------
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
//----------------------------------------------

void removeTemporaryRoads(const Allocation& allocation, TemporaryRoads& temps) {
    int startXPosition = allocation.start.x, startYPostion = allocation.start.y;
    int goalXPosition = allocation.goal.x, goalYPostion = allocation.goal.y;

    for (size_t i = 0; i < temps.roadNum; ++i) {
        auto& coordPair = temps.roadStatusList[i].coordpair;
        auto& status = temps.roadStatusList[i].status;

        if ((coordPair.coords[0].x == startXPosition && coordPair.coords[0].y == startYPostion) ||
            (coordPair.coords[0].x == goalXPosition && coordPair.coords[0].y == goalYPostion)) {
            status[0] = 0;
        }
        if ((coordPair.coords[1].x == startXPosition && coordPair.coords[1].y == startYPostion) ||
            (coordPair.coords[1].x == goalXPosition && coordPair.coords[1].y == goalYPostion)) {
            status[1] = 0;
        }
    }
}
void removeTemporaryRoads(const Allocation& allocation, std::vector<RoadAroundCenter>& roadStatusVector) {
    int startindex = allocation.start.x * GRID_SIZE_Y + allocation.start.y;
    int goalindex = allocation.goal.x * GRID_SIZE_Y + allocation.goal.y;
    for(const auto& direction:DIRECTIONS) {
        if( roadStatusVector[startindex].neighbors[DIRECTION_TO_INDEX.at(direction)].type != RoadType::PROHIBITED){
            roadStatusVector[startindex].neighbors[DIRECTION_TO_INDEX.at(direction)].type = RoadType::NONE;}
        if( roadStatusVector[goalindex].neighbors[7-DIRECTION_TO_INDEX.at(direction)].type != RoadType::PROHIBITED){
            roadStatusVector[goalindex].neighbors[7-DIRECTION_TO_INDEX.at(direction)].type = RoadType::NONE;
        }
    }
}


void changeSoil(const Allocation& allocation, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    soil_amount[allocation.start.x][allocation.start.y] -= allocation.volume;
    soil_amount[allocation.goal.x][allocation.goal.y] += allocation.volume;
}

// std::vector<int> buildNewRoad(
//     TemporaryRoads& current_temps, 
//     const int stepNum, 
//     const Solution& solution, 
//     std::vector<double>& built_length_list, 
//     const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], 
//     double& built_length
// ) {
//     std::vector<int> builtRoadList;
//     int road_count = current_temps.roadNum;
//     for (int i = 0; i < road_count; i++) {
//         CoordPair road = solution.roadbuildStatusList[i].coordpair;
//         // 道路の建設が完了しているかどうかを確認
//         if (solution.roadbuildStatusList[i].timings[stepNum] == 1) {
//             double length = calculate_distance_3D(road.coords[0], road.coords[1], soil_amount) / 2;
//             if (current_temps.roadStatusList[i].status[0] == 0 && current_temps.roadStatusList[i].status[1] == 0) {
//                 built_length += length*2;
//                 built_length_list[i] += length*2;
//                 current_temps.roadStatusList[i].status[0] = 1;
//                 current_temps.roadStatusList[i].status[1] = 1;
//                 builtRoadList.push_back(i);
//             }else if (current_temps.roadStatusList[i].status[0] == 0 || current_temps.roadStatusList[i].status[1] == 0) {
//                 built_length += length;
//                 built_length_list[i] += length;
//                 current_temps.roadStatusList[i].status[0] = 1;
//                 current_temps.roadStatusList[i].status[1] = 1;
//                 builtRoadList.push_back(i);
//             }
//         }
//     }
//     return builtRoadList;
// }



std::vector<std::pair<int,int>> buildNewRoad(
    std::vector<RoadAroundCenter>& roadStatusList, 
    const int stepNum, 
    const Solution& solution,  
    std::vector<double>& built_length_list,
    const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], 
    double& built_length
){
    std::vector<std::pair<int,int>> builtRoadList;
    int road_count = solution.roadNum;
    for (int i = 0; i < road_count; i++) {
        CoordPair road = solution.roadbuildStatusList[i].coordpair;
        // 道路の建設が完了しているかどうかを確認
        if (solution.roadbuildStatusList[i].timings[stepNum] == 1) {
            double length = calculate_distance_3D(road.coords[0], road.coords[1], soil_amount) / 2;
            int index1 = road.coords[0].x * GRID_SIZE_Y + road.coords[0].y;
            Coord dir1to2 = {road.coords[1].x - road.coords[0].x, road.coords[1].y - road.coords[0].y};
            int index1to2 = DIRECTION_TO_INDEX.at(dir1to2);
            int index2 = road.coords[1].x * GRID_SIZE_Y + road.coords[1].y;
            int index2to1 = 7-index1to2; // 方向を反転させたインデックス
            //coord[0]->(coord[0]とcoord[1]の中間地点）方向の道路を建てる処理
            if(roadStatusList[index1].neighbors[index1to2].type == RoadType::PROHIBITED||roadStatusList[index2].neighbors[index2to1].type == RoadType::PROHIBITED) {
                continue; // 通行禁止の道路は建設しない
            }

            if (roadStatusList[index1].neighbors[index1to2].type == RoadType::NONE) {
                built_length += length;
                built_length_list[i] += length;
                roadStatusList[index1].neighbors[index1to2].type = RoadType::PAVED; // 舗装道路にする
                builtRoadList.push_back({index1,index1to2});
            }else if(roadStatusList[index1].neighbors[index1to2].type == RoadType::UNPAVED) {
                built_length += length*CONSTRUCTION_TEMP_DIFF/CONSTRUCTION_TEMP; //未舗装道路を仮設道路にする場合のコスト
                built_length_list[i] += length*CONSTRUCTION_TEMP_DIFF/CONSTRUCTION_TEMP;
                roadStatusList[index1].neighbors[index1to2].type = RoadType::PAVED; // 舗装道路にする
                builtRoadList.push_back({index1,index1to2}); 
            }
            //coord[1]->(coord[0]とcoord[1]の中間地点）方向の道路を建てる処理
            if (roadStatusList[index2].neighbors[index2to1].type == RoadType::NONE) {
                built_length += length;
                built_length_list[i] += length;
                roadStatusList[index2].neighbors[index2to1].type = RoadType::PAVED; // 舗装道路にする
                builtRoadList.push_back({index2,index2to1});
            }else if (roadStatusList[index2].neighbors[index2to1].type == RoadType::UNPAVED) {
                built_length += length*CONSTRUCTION_TEMP_DIFF/CONSTRUCTION_TEMP; //未舗装道路を仮設道路にする場合のコスト
                built_length_list[i] += length*CONSTRUCTION_TEMP_DIFF/CONSTRUCTION_TEMP;
                roadStatusList[index2].neighbors[index2to1].type = RoadType::PAVED; // 舗装道路にする
                builtRoadList.push_back({index2,index2to1});
            }
        }
    }
    return builtRoadList;
}



void setCostMatrix(double costMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y], const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    // Initialize costMatrix assuming there is no roads built yet
    for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; ++i) {
        for (int j = 0; j < GRID_SIZE_X * GRID_SIZE_Y; ++j) {
            if(i == j) {
                costMatrix[i][j] = 0; // 自身へのコストは0
            } else {
                // 他のノードへのコストは無限大で初期化
                costMatrix[i][j] = INF;
            }
        }
    }
    //隣接しているノード間のコストを計算
    for (int x = 0; x < GRID_SIZE_X; ++x) {
        for (int y = 0; y < GRID_SIZE_Y; ++y) {
            Coord current = {x, y};
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue;
                    Coord neighbor = {x + dx, y + dy};
                    if (neighbor.x < 0 || neighbor.x >= GRID_SIZE_X || neighbor.y < 0 || neighbor.y >= GRID_SIZE_Y) continue;
                    costMatrix[current.x * GRID_SIZE_Y + current.y][neighbor.x * GRID_SIZE_Y + neighbor.y] = getCost(current, neighbor, soil_amount);
                    costMatrix[neighbor.x * GRID_SIZE_Y + neighbor.y][current.x * GRID_SIZE_Y + current.y] = costMatrix[current.x * GRID_SIZE_Y + current.y][neighbor.x * GRID_SIZE_Y + neighbor.y]; // Assuming undirected graph, set both directions
        
                }
            }
        }
    }
}

bool buildRoads(const TemporaryRoads& temporary_roads, const Coord& coord) {
    // 仮設道路を建設する予定のある座標ならtrueを返す
    for (const auto& roadStatus : temporary_roads.roadStatusList) {
        if ((roadStatus.coordpair.coords[0].x == coord.x && roadStatus.coordpair.coords[0].y == coord.y) ||
            (roadStatus.coordpair.coords[1].x == coord.x && roadStatus.coordpair.coords[1].y == coord.y)) {
            return true; // 仮設道路が建設される予定の座標
        }
    }
    return false;
}

// void updateCostMatrixAfterBuild(
//     double moveMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y], 
//     double buildMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y],
//     const TemporaryRoads& temporary_roads, 
//     const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
//     const std::vector<int>& builtRoadList
// ) {
//     // Update the cost matrix after building roads
//     //update the moveMatrix with the new costs for the built roads
//     for(int roadIndex : builtRoadList) {
//         Coord roadCoord1 = temporary_roads.roadStatusList[roadIndex].coordpair.coords[0];
//         Coord roadCoord2 = temporary_roads.roadStatusList[roadIndex].coordpair.coords[1];
//         int coord1Index = roadCoord1.x * GRID_SIZE_Y + roadCoord1.y;
//         int coord2Index = roadCoord2.x * GRID_SIZE_Y + roadCoord2.y;
//         // Update the cost for the road coordinates
//         moveMatrix[coord1Index][coord2Index] = getCost(roadCoord1, roadCoord2, soil_amount, temporary_roads);
//         moveMatrix[coord2Index][coord1Index] = moveMatrix[coord1Index][coord2Index]; 
//         // Update the buildMatrix for the road coordinates
//         buildMatrix[coord1Index][coord2Index] = 0; // 仮設道路が引かれたところのコストを0にする
//         buildMatrix[coord2Index][coord1Index] = 0; // 仮設道路が引かれたところのコストを0にする
//     }
// }

void updateCostMatrixAfterTerrainChange(double costMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y], const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], const TemporaryRoads& temporary_roads, const Allocation& allocation) {
    // Update the cost matrix of alocation start and goal
    int startIndex = allocation.start.x * GRID_SIZE_Y + allocation.start.y;
    int goalIndex = allocation.goal.x * GRID_SIZE_Y + allocation.goal.y;

    for(int dx=-1; dx<=1; ++dx) {
        for(int dy=-1; dy<=1; ++dy) {
            if(dx == 0 && dy == 0) continue; // 自分自身のコストは更新しない
            Coord startNeighbor = {allocation.start.x + dx, allocation.start.y + dy};
            Coord goalNeighbor = {allocation.goal.x + dx, allocation.goal.y + dy};
            if(startNeighbor.x > 0 && startNeighbor.x < GRID_SIZE_X && startNeighbor.y > 0 && startNeighbor.y < GRID_SIZE_Y) {
                double new_cost1 = getCost(allocation.start, startNeighbor, soil_amount, temporary_roads);
                costMatrix[startIndex][startNeighbor.x * GRID_SIZE_Y + startNeighbor.y] = new_cost1;
                costMatrix[startNeighbor.x * GRID_SIZE_Y + startNeighbor.y][startIndex] = new_cost1; // Assuming undirected graph, set both directions
            }
            if(goalNeighbor.x > 0 && goalNeighbor.x < GRID_SIZE_X && goalNeighbor.y > 0 && goalNeighbor.y < GRID_SIZE_Y) {
                double new_cost2 = getCost(allocation.goal, goalNeighbor, soil_amount, temporary_roads);
                costMatrix[goalIndex][goalNeighbor.x * GRID_SIZE_Y + goalNeighbor.y] = new_cost2;
                costMatrix[goalNeighbor.x * GRID_SIZE_Y + goalNeighbor.y][goalIndex] = new_cost2; // Assuming undirected graph, set both directions
            }
        }
    }
}

void updateCostMatrixAfterShovelMove(double costMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y], const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], const TemporaryRoads& temporary_roads, const Allocation& allocation) {
    int startIndex = allocation.start.x * GRID_SIZE_Y + allocation.start.y;
    int goalIndex = allocation.goal.x * GRID_SIZE_Y + allocation.goal.y;
    // もし仮設道路を建設しないセルであり，土量がゼロになったらアクセスできないようにする
    // std::cout << "buildRoads: " << buildRoads(temporary_roads, allocation.start) << std::endl;
    if(!buildRoads(temporary_roads, allocation.start) && soil_amount[allocation.start.x][allocation.start.y] == 0) {
        // std::cout << "start to be inaccessible" << std::endl;
            // 自分の周りのセルからアクセスできないようにする
        for(int i =-1; i <= 1; ++i) {
            if((allocation.start.x + i) < 0 || (allocation.start.x + i) >= GRID_SIZE_X) continue; // エリア外は除外
            for(int j = -1; j <= 1; ++j) {
                if((i == 0 && j == 0) ||  (allocation.start.y + j) < 0 || (allocation.start.y + j) >= GRID_SIZE_Y) continue; // 自分自身とエリア外は除外
                int neighborIndex = (allocation.start.x + i) * GRID_SIZE_Y + (allocation.start.y + j);
                    costMatrix[startIndex][neighborIndex] = INF; // アクセスできないようにする
                    costMatrix[neighborIndex][startIndex] = INF; // アクセスできないようにする    
            }
        }
    }
    // std::cout << "buildRoads: " << buildRoads(temporary_roads, allocation.goal) << std::endl;
    // std::cout << "soil_amount[goal]: " << soil_amount[allocation.goal.x][allocation.goal.y] << std::endl;
    if(!buildRoads(temporary_roads, allocation.goal) && soil_amount[allocation.goal.x][allocation.goal.y] == 0) {
        // std::cout << "goal to be inaccessible" << std::endl;
        for(int i =-1; i <= 1; ++i) {
            if((allocation.goal.x + i) < 0 || (allocation.goal.x + i) >= GRID_SIZE_X) continue; // エリア外は除外
            for(int j = -1; j <= 1; ++j) {
                if((i == 0 && j == 0) || (allocation.goal.y + j) < 0 || (allocation.goal.y + j) >= GRID_SIZE_Y) continue; 
                int neighborIndex = (allocation.goal.x + i) * GRID_SIZE_Y + (allocation.goal.y + j);
                    costMatrix[goalIndex][neighborIndex] = INF; // アクセスできないようにする
                    costMatrix[neighborIndex][goalIndex] = INF; // アクセスできないようにする
            }
        }
    }
}

std::tuple<double,Path,std::vector<int>> astar(const Allocation& allocation, const TemporaryRoads& temps, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],bool searchFlag = true) {   
    std::vector<int> used_temp_list(temps.roadNum, 0); // Initialize used_temp_list with zeros
    double temp_eff = searchFlag ? TEMP_EFF : 0;  
    std::cout <<"temp_eff: " << temp_eff << std::endl;
    Coord start = allocation.start;
    Coord goal = allocation.goal;
    Path path;
    double total_cost = 0;

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

            double new_cost = cost_so_far[current.x][current.y] + getCost(current, neighbor, soil_amount, temps);

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
        if(searchFlag){
        CoordPair search_pair = normalize_pair(current, next);
        auto it = std::find_if(
            temps.roadStatusList.begin(),
            temps.roadStatusList.end(),
            [&search_pair](const TemporaryRoadStatus& roadStatus) {
                return roadStatus.coordpair.coords[0].x == search_pair.coords[0].x &&
                       roadStatus.coordpair.coords[0].y == search_pair.coords[0].y &&
                       roadStatus.coordpair.coords[1].x == search_pair.coords[1].x &&
                       roadStatus.coordpair.coords[1].y == search_pair.coords[1].y;
            });

        if (it != temps.roadStatusList.end()) {
            size_t index = std::distance(temps.roadStatusList.begin(), it);
            if (temps.roadStatusList[index].status[0] == 1 && temps.roadStatusList[index].status[1] == 1) {
                // std::cout << "index: " << index << " is used(both)" << std::endl;
                used_temp_list[index] = 2;
            } else if 
                (temps.roadStatusList[index].status[0] == 1|| temps.roadStatusList[index].status[1] == 1) {
                // std::cout << "index: " << index << " is used" << std::endl;
                used_temp_list[index] = 1;
            }
            else {
                // std::cout << "index: " << index << " is not used" << std::endl;
                used_temp_list[index] = 0;
            }
        }
    }
        current = next;
    }
    reverse_path.push_back(start);

    // 結果をPathに反映
    path.coord.assign(reverse_path.rbegin(), reverse_path.rend());
    total_cost = cost_so_far[goal.x][goal.y]; //往復回数込みのコスト
    return {total_cost, path, used_temp_list}; 
}


//get_cost関数をcost_matrixを使って計算するようにしたバージョン
std::tuple<double,Path> astar(const Allocation& allocation, const double costMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y],const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {   
    Coord start = allocation.start;
    Coord goal = allocation.goal;
    Path path;
    double total_cost = 0;

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

            double new_cost = cost_so_far[current.x][current.y] + costMatrix[current.x * GRID_SIZE_Y + current.y][neighbor.x * GRID_SIZE_Y + neighbor.y];

            if (new_cost < cost_so_far[neighbor.x][neighbor.y]) {
                cost_so_far[neighbor.x][neighbor.y] = new_cost;
                int priority = new_cost + heuristic(goal, neighbor, soil_amount);
                open_set.push({priority, neighbor});
                came_from[neighbor.x][neighbor.y] = current;
            }
        }
    }
    if (cost_so_far[goal.x][goal.y] == INF) {
        return {INF, Path{}};  // 無限コスト＋空のパスを返す
    }

    // 経路再構築
    std::vector<Coord> reverse_path;
    Coord current = goal;
    while (!(current.x == start.x && current.y == start.y)) {
        reverse_path.push_back(current);
        Coord next = came_from[current.x][current.y];
        current = next;
    }
    reverse_path.push_back(start);

    // 結果をPathに反映
    path.coord.assign(reverse_path.rbegin(), reverse_path.rend());
    total_cost = cost_so_far[goal.x][goal.y]; //往復回数込みのコスト
    return {total_cost, path}; 
}

//usedTempListを求める関数
std::vector<int> usedTempList(const Path& path, const TemporaryRoads& temps) {
    std::vector<int> used_temp_list(temps.roadNum, 0); // Initialize used_temp_list with zeros
    for (size_t i = 0; i < path.size(); ++i) {
        Coord current = path.coord[i];
        if (i + 1 < path.size()) {
            Coord next = path.coord[i + 1];
            CoordPair search_pair = normalize_pair(current, next);
            auto it = std::find_if(
                temps.roadStatusList.begin(),
                temps.roadStatusList.end(),
                [&search_pair](const TemporaryRoadStatus& roadStatus) {
                    return roadStatus.coordpair.coords[0].x == search_pair.coords[0].x &&
                           roadStatus.coordpair.coords[0].y == search_pair.coords[0].y &&
                           roadStatus.coordpair.coords[1].x == search_pair.coords[1].x &&
                           roadStatus.coordpair.coords[1].y == search_pair.coords[1].y;
                });
            if (it != temps.roadStatusList.end()) {
                size_t index = std::distance(temps.roadStatusList.begin(), it);
                if (temps.roadStatusList[index].status[0] == 1 && temps.roadStatusList[index].status[1] == 1) {
                    used_temp_list[index] = 2;
                } else if (temps.roadStatusList[index].status[0] == 1 || temps.roadStatusList[index].status[1] == 1) {
                    used_temp_list[index] = 1;
                } else {
                    used_temp_list[index] = 0;
                }
            }
        }
    }
    
    return used_temp_list;
}

void initializeRoadStatusVector(std::vector<RoadAroundCenter>& roadStatusVector ) {
    // Initialize roadStatusMatrix with zeros
    for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; ++i) {
        RoadAroundCenter roadStatus;
        roadStatus.center = {i / GRID_SIZE_Y, i % GRID_SIZE_Y}; // 中心座標
        for (const Coord & direction : DIRECTIONS) {
            NeighborInfo neighborinfo;
            neighborinfo.direction = direction;
            neighborinfo.type = RoadType::NONE;
            roadStatus.neighbors.push_back(neighborinfo);
        }
        roadStatusVector.push_back(roadStatus);
    }
}

// void updateRoadStatusVectorAfterBuild(std::vector<RoadAroundCenter>& roadStatusVector, const std::vector<CoordPair>& coordPairs, std::vector<int>& builtRoadList) {
//     // Update the roadStatusMatrix after building roads
//     for(int roadIndex : builtRoadList) {
//         CoordPair coordpair = coordPairs[roadIndex];
//         Coord coord1 = coordpair.coords[0];
//         Coord coord2 = coordpair.coords[1];
//         int index1 = coord1.x * GRID_SIZE_Y + coord1.y;
//         int index1to2 = (coord2.x- coord1.x) * 3 + (coord2.y - coord1.y); // index1から見たindex2の相対インデックス
//         int index2 = coord2.x * GRID_SIZE_Y + coord2.y;
//         int index2to1 = (coord1.x - coord2.x) * 3 + (coord1.y - coord2.y);
//         if(roadStatusVector[index1].neighbors[index1to2].type == RoadType::NONE || roadStatusVector[index1].neighbors[index1to2].type == RoadType::UNPAVED) {
//             roadStatusVector[index1].neighbors[index2].type = RoadType::PAVED;
//             roadStatusVector[index2].neighbors[index1].type = RoadType::PAVED;
//         } 
//         if(roadStatusVector[index2].neighbors[index2to1].type == RoadType::NONE || roadStatusVector[index2].neighbors[index2to1].type == RoadType::UNPAVED) {
//             roadStatusVector[index2].neighbors[index1].type = RoadType::PAVED;
//             roadStatusVector[index1].neighbors[index2].type = RoadType::PAVED;
//         }
//     }
// }

std::tuple<int, std::vector<std::pair<int, int>>> updateRoadStatusMatrixandSubRoadsAfterAstar(std::vector<RoadAroundCenter>& roadStatusVector, const Path& path, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],Solution& solution,int stepNum) {
    // path上で道路がない場合，未舗装道路を建てる
    // std::cout << "updateRoadStatusMatrixandSubRoadsAfterAstar" << std::endl;
    std::vector<std::pair<int, int>> builtSubRoadsIndexList; // 未舗装道路を建設した箇所のインデックス情報を記録するリスト
    int totalDistance = 0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        CoordPair coordpair = {path.coord[i], path.coord[i + 1]};
        int distance = calculate_distance_3D(coordpair.coords[0], coordpair.coords[1],soil_amount)/2;
        int index1 = coordpair.coords[0].x * GRID_SIZE_Y + coordpair.coords[0].y;
        Coord dir1to2 = {coordpair.coords[1].x - coordpair.coords[0].x, coordpair.coords[1].y - coordpair.coords[0].y};
        int index1to2 = DIRECTION_TO_INDEX.at(dir1to2); // index1から見たindex2の相対インデックス
        int index2 = coordpair.coords[1].x * GRID_SIZE_Y + coordpair.coords[1].y;
        int index2to1 = 7-index1to2; // index2から見たindex1の相対インデックス
        if(roadStatusVector[index1].neighbors[index1to2].type == RoadType::PROHIBITED || roadStatusVector[index2].neighbors[index2to1].type == RoadType::PROHIBITED) {
            continue; // 通行禁止の道路は建設しない
        }

        if (roadStatusVector[index1].neighbors[index1to2].type == RoadType::NONE) {
            // std::cout << "build sub road: " << index1 << " index1to2"<< index1to2 << std::endl;
            roadStatusVector[index1].neighbors[index1to2].type = RoadType::UNPAVED;
            builtSubRoadsIndexList.push_back({index1, index1to2}); // 未舗装道路を建設した情報を記録
            totalDistance += distance;
        }
        if (roadStatusVector[index2].neighbors[index2to1].type == RoadType::NONE) {
            // std::cout << "build sub road: " << index2 << " index2to1"<< index2to1 << std::endl;
            roadStatusVector[index2].neighbors[index2to1].type = RoadType::UNPAVED;
            builtSubRoadsIndexList.push_back({index2, index2to1}); // 未舗装道路を建設した情報を記録
            totalDistance += distance;
        }
    }

    // solutionに未舗装道路の情報を追加
    for (const auto& roadIndexPair : builtSubRoadsIndexList) {
        Coord coord1 = roadStatusVector[roadIndexPair.first].center;
        Coord coord2 = {roadStatusVector[roadIndexPair.first].center.x + DIRECTIONS[roadIndexPair.second].x, 
                        roadStatusVector[roadIndexPair.first].center.y + DIRECTIONS[roadIndexPair.second].y};
        // solutionにroadbuildStatusList.coordpairのtimingを1にするしたものを追加
        CoordPair coordpair = {coord1, coord2};
        // もしsolutionのroadbuildStatusListに同じcoordpairが存在する場合は、timingを1に更新
        auto it = std::find_if(solution.roadbuildStatusList.begin(), solution.roadbuildStatusList.end(),
                               [&coordpair](const TemporaryRoadbuildStatus& roadStatus) {
                                   return roadStatus.coordpair.coords[0].x == coordpair.coords[0].x &&
                                          roadStatus.coordpair.coords[0].y == coordpair.coords[0].y &&
                                          roadStatus.coordpair.coords[1].x == coordpair.coords[1].x &&
                                          roadStatus.coordpair.coords[1].y == coordpair.coords[1].y;
                               });
        if (it != solution.roadbuildStatusList.end()) {
            // 既に存在する場合はtimingを更新
            it->timings[stepNum] = 1;
        } else {
            // 存在しない場合は新たに追加
            TemporaryRoadbuildStatus newRoadStatus;
            newRoadStatus.coordpair = coordpair;
            newRoadStatus.timings.resize(solution.stepNum, 0); // 初期化
            newRoadStatus.timings[stepNum] = 1; // 現在のステップで建設されたことを示す
            solution.roadbuildStatusList.push_back(newRoadStatus);
            solution.roadNum += 1; // 道路の数を更新
        }
    }
    return {totalDistance, builtSubRoadsIndexList}; 
}
const std::vector<Coord> HALF_DIRECTIONS = {
    {1, 0},     // 右
    {-1, -1},    // 左下
    {0, -1},     // 下
    {1, -11}      // 右下
};

//入り口から作業地点までの最短経路探索する用のコスト行列を作成する関数．道路があるところはコスト０．それ以外は距離
void RoadStatusVectorToCostMatrix(const std::vector<RoadAroundCenter>& roadStatusVector, double costMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y],const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    // Initialize costMatrix
    for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; ++i) {
        for (int j = 0; j < GRID_SIZE_X * GRID_SIZE_Y; ++j) {
            costMatrix[i][j] = INF; // 初期化：すべてのコストを無限大に設定
        }
    }
    //隣接セルのコストを距離に設定.道がある場合はコスト0，ない場合は距離を設定する
    for (int x = 0; x < GRID_SIZE_X; ++x) {
        for (int y = 0; y < GRID_SIZE_Y; ++y) {
            Coord current = {x, y};
            int centerIndex = x * GRID_SIZE_Y + y;
            costMatrix[centerIndex][centerIndex] = 0; // 自身へのコストは0
            for (const Coord& direction : HALF_DIRECTIONS) {
                int neighbor_x = x + direction.x;
                int neighbor_y = y + direction.y;
                if (neighbor_x < 0 || neighbor_x >= GRID_SIZE_X || neighbor_y < 0 || neighbor_y >= GRID_SIZE_Y) continue;

                int neighborDirectionIndex = DIRECTION_TO_INDEX.at(direction);
                int reverseDirectionIndex = DIRECTION_TO_INDEX.at({-direction.x, -direction.y});

                int neighborIndex = neighbor_x * GRID_SIZE_Y + neighbor_y;

                RoadType from_center = roadStatusVector[centerIndex].neighbors[neighborDirectionIndex].type;
                RoadType to_center   = roadStatusVector[neighborIndex].neighbors[reverseDirectionIndex].type;

                double distance = calculate_distance_3D(current, {neighbor_x, neighbor_y}, soil_amount);
                double cost = 0.0;

                if (from_center == RoadType::NONE) cost += distance/2;
                if (to_center   == RoadType::NONE) cost += distance/2;

                costMatrix[centerIndex][neighborIndex] = cost;
                costMatrix[neighborIndex][centerIndex] = cost; // 双方向セット
            }
            
        }
    }
}

std::vector<std::pair<int, int>> connectToEntrance(std::vector<RoadAroundCenter>& roadStatusVector, const Allocation allocation,const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],int stepNum,Solution& subsolution) {
    std::vector<std::pair<int, int>> builtSubRoadIndexList; // 未舗装道路を建設した情報を記録するリスト
    //2つの作業地点から入り口までの接続に必要な道路を，未舗装道路で接続する.追加した道路の情報はroadStatusVector,subsolutionに記録する
    double costMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y];
    RoadStatusVectorToCostMatrix(roadStatusVector, costMatrix, soil_amount);
    Allocation entranceToWorkCoord; // 作業地点から入り口までの経路を求めるためのAllocation
    entranceToWorkCoord.start = allocation.start; // 作業地点の座標
    entranceToWorkCoord.goal = {entranceXPostion, entranceYPostion}; // 入り口の座標
    entranceToWorkCoord.volume = 0; // 作業量は何でもいい
    // 作業地点から入り口までの最短経路を求める
    auto [cost, path] = astar(entranceToWorkCoord, costMatrix, soil_amount); 
    // searchFlagをfalseにして仮設道路を考慮しない
    //path上の座標に道路が建設済みじゃない場合，未舗装道路を建設する
    for (int i = 0; i < path.size() - 1; ++i) {
        Coord coord1 = path.coord[i];
        Coord coord2 = path.coord[i + 1];
        int index1 = coord1.x * GRID_SIZE_Y + coord1.y;
        int index1to2 = DIRECTION_TO_INDEX.at({coord2.x - coord1.x, coord2.y - coord1.y}); // index1から見たindex2の相対インデックス
        int index2 = coord2.x * GRID_SIZE_Y + coord2.y;
        int index2to1 = 7-index1to2; // index2から見たindex1の相対インデックス

        if (roadStatusVector[index1].neighbors[index1to2].type == RoadType::NONE || roadStatusVector[index2].neighbors[index2to1].type == RoadType::NONE) {
            if(roadStatusVector[index1].neighbors[index1to2].type == RoadType::NONE) {
                roadStatusVector[index1].neighbors[index1to2].type = RoadType::UNPAVED;
                builtSubRoadIndexList.push_back({index1, index1to2}); // 未舗装道路を建設した情報を記録
            }
            if(roadStatusVector[index2].neighbors[index2to1].type == RoadType::NONE) {
                roadStatusVector[index2].neighbors[index2to1].type = RoadType::UNPAVED;
                builtSubRoadIndexList.push_back({index2, index2to1}); // 未舗装道路を建設した情報を記録
            }
        }
        // solutionにroadbuildStatusList.coordpairのtimingを1にするしたものを追加
        CoordPair coordpair = {coord1, coord2};
        // もしsolutionのroadbuildStatusListに同じcoordpairが存在する場合は、timingを1に更新
        auto it = std::find_if(subsolution.roadbuildStatusList.begin(), subsolution.roadbuildStatusList.end(),
                               [&coordpair](const TemporaryRoadbuildStatus& roadStatus) {
                                   return roadStatus.coordpair.coords[0].x == coordpair.coords[0].x &&
                                          roadStatus.coordpair.coords[0].y == coordpair.coords[0].y &&
                                          roadStatus.coordpair.coords[1].x == coordpair.coords[1].x &&
                                          roadStatus.coordpair.coords[1].y == coordpair.coords[1].y;
                               });
        if (it != subsolution.roadbuildStatusList.end()) {
            // 既に存在する場合はtimingを更新
            it->timings[stepNum] = 1;
        } else {
            // 存在しない場合は新たに追加
            TemporaryRoadbuildStatus newRoadStatus;
            newRoadStatus.coordpair = coordpair;
            newRoadStatus.timings.resize(subsolution.stepNum, 0); // 初期化
            newRoadStatus.timings[stepNum] = 1; // 現在のステップで建設されたことを示す
            subsolution.roadbuildStatusList.push_back(newRoadStatus);
            subsolution.roadNum += 1; // 道路の数を増やす
        }
    }
    return builtSubRoadIndexList; // 未舗装道路を建設した情報を返す
}

void addMatrix(double costMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y], 
               double moveCostMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y],
               double buildCostMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y],
               double trip_num) {
    // Add the moveCostMatrix and buildCostMatrix to the costMatrix
    double beta = CONSTRUCTION_SUB_TEMP * VELOCITY_ROUGH / (COST_HOUR * TRUCK_NUM); // 未舗装道路の建設コストを考慮した係数
    for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; ++i) {
        for (int j = 0; j < GRID_SIZE_X * GRID_SIZE_Y; ++j) {
            costMatrix[i][j] = moveCostMatrix[i][j] + buildCostMatrix[i][j] * beta;
        }
    }
}

//土砂配分を行った際に新たに建てた未舗装道路の部分のBuildMatrixを更新する関数
void updateBuildMatrixBeforeAstar(
    double buildMatrix[GRID_SIZE_X * GRID_SIZE_Y][GRID_SIZE_X * GRID_SIZE_Y],
    const std::vector<RoadAroundCenter>& roadStatusVector,
    const std::vector<std::pair<int, int>>& updatebuildIndexList,
    const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
) {
    for (const auto& indexpair : updatebuildIndexList) {
        int centerIndex = indexpair.first;
        int directionIndex = indexpair.second;

        if (directionIndex < 0 || directionIndex >= DIRECTIONS.size()) continue;

        const Coord& centerCoord = roadStatusVector[centerIndex].center;
        int neighborX = centerCoord.x + DIRECTIONS[directionIndex].x;
        int neighborY = centerCoord.y + DIRECTIONS[directionIndex].y;

        // 範囲チェック
        if (neighborX < 0 || neighborX >= GRID_SIZE_X || neighborY < 0 || neighborY >= GRID_SIZE_Y) continue;

        int neighborIndex = neighborX * GRID_SIZE_Y + neighborY;

        // 道路の種類取得（中心→隣接方向）
        RoadType forwardType = roadStatusVector[centerIndex].neighbors[directionIndex].type;

        // 道路の種類取得（隣接→中心方向）も確認（反対方向のインデックスを調べる）
        Coord reverseDir = {-DIRECTIONS[directionIndex].x, -DIRECTIONS[directionIndex].y};
        int reverseDirectionIndex = DIRECTION_TO_INDEX.at(reverseDir); // 反対方向のインデックスを取得        

        // 双方向どちらかに道路があればコスト0、なければ距離を使う
        double cost = 0.0;
        double forwardcost = 0.0;
        double reversecost = 0.0;
        if (forwardType == RoadType::NONE ) {
            // 道路が両方向にない → 距離を使う
            Coord neighborCoord = {neighborX, neighborY};
            forwardcost = calculate_distance_3D(centerCoord, neighborCoord, soil_amount)/2;
        }
        if (roadStatusVector[neighborIndex].neighbors[reverseDirectionIndex].type == RoadType::NONE)    {
            // 反対方向の道路がない → 距離を使う
            Coord neighborCoord = {neighborX, neighborY};
            reversecost = calculate_distance_3D(neighborCoord, centerCoord, soil_amount)/2;
        }
        cost = forwardcost + reversecost;

        // コストを双方向に設定
        buildMatrix[centerIndex][neighborIndex] = cost;
        buildMatrix[neighborIndex][centerIndex] = cost;
    }
}



RoadType getRoadType(const std::vector<RoadAroundCenter>& roadStatusVector, int index1, int index2) {
    // Get the road type between two indices
    int index1to2_x = (index2 - index1) / GRID_SIZE_Y; // index1からindex2への相対インデックスのx座標
    int index1to2_y = (index2 - index1) % GRID_SIZE_Y; // index1からindex2への相対インデックスのy座標
    int index1to2 = (1+index1to2_x) * 3 + (1+index1to2_y); // index1から見たindex2の相対インデックス
    if(index1to2 < 0 || index1to2 >= 9) return RoadType::PROHIBITED; // エリア外は除外
    return roadStatusVector[index1].neighbors[index1to2].type;
}

void updateMoveMatrixBeforeAstar(
    double moveMatrix[GRID_SIZE_X * GRID_SIZE_Y][GRID_SIZE_X * GRID_SIZE_Y], 
    const std::vector<RoadAroundCenter>& roadStatusVector,
    const std::vector<std::pair<int, int>>& updateMoveIndexList,
    const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
) {
    for (const auto& indexpair : updateMoveIndexList) {
        int fromIndex = indexpair.first;
        int dirIndex = indexpair.second;

        const Coord fromCoord = roadStatusVector[fromIndex].center;
        const Coord offset = DIRECTIONS[dirIndex];
        const Coord toCoord = {fromCoord.x + offset.x, fromCoord.y + offset.y};

        if (toCoord.x < 0 || toCoord.x >= GRID_SIZE_X || toCoord.y < 0 || toCoord.y >= GRID_SIZE_Y) continue;

        int toIndex = toCoord.x * GRID_SIZE_Y + toCoord.y;

        // 中間地点までの距離
        double halfDistance = calculate_distance_3D(fromCoord, toCoord, soil_amount) / 2.0;

        double fromToHalfCost = 0.0;
        auto roadType = roadStatusVector[fromIndex].neighbors[dirIndex].type;
        if (roadType == RoadType::PROHIBITED) {
            fromToHalfCost = INF;
        } else if (roadType == RoadType::PAVED) {
            fromToHalfCost = halfDistance * TEMP_EFF;
        } else {
            fromToHalfCost = halfDistance;
        }

        // 逆方向の情報を調べる（toCoord → fromCoord）
        Coord reverseOffset = {-offset.x, -offset.y};
        int reverseDirIndex = DIRECTION_TO_INDEX.at(reverseOffset);

        double toToHalfCost = 0.0;
        auto reverseRoadType = roadStatusVector[toIndex].neighbors[reverseDirIndex].type;
        if (reverseRoadType == RoadType::PROHIBITED) {
            toToHalfCost = INF;
        } else if (reverseRoadType == RoadType::PAVED) {
            toToHalfCost = halfDistance * TEMP_EFF;
        } else {
            toToHalfCost = halfDistance;
        }

        double totalCost = (fromToHalfCost == INF || toToHalfCost == INF)
                            ? INF
                            : fromToHalfCost + toToHalfCost;

        moveMatrix[fromIndex][toIndex] = totalCost;
        moveMatrix[toIndex][fromIndex] = totalCost;
    }
}


void addUpdateIndexList(
    std::vector<std::pair<int, int>>& updateBuiltIndexList,
    std::vector<std::pair<int, int>>& updateMoveIndexList,
    const Allocation& allocation
) {
    // for (const auto& direction : DIRECTIONS) {
    //     if(direction.x == 0 && direction.y == 0) continue; // 自分自身の方向は除外
        
    //     updateBuiltIndexList.push_back({allocation.start.x * GRID_SIZE_Y + allocation.start.y, DIRECTION_TO_INDEX.at(direction) });
    //     updateBuiltIndexList.push_back({allocation.goal.x * GRID_SIZE_Y + allocation.goal.y, DIRECTION_TO_INDEX.at(direction) });
    //     //作業地点から隣接セル方向へのmoveコストを更新候補に入れる
    //     updateMoveIndexList.push_back({allocation.start.x * GRID_SIZE_Y + allocation.start.y, DIRECTION_TO_INDEX.at(direction)});
    //     updateMoveIndexList.push_back({allocation.goal.x * GRID_SIZE_Y + allocation.goal.y, DIRECTION_TO_INDEX.at(direction)});
    // }
    //作業地点から隣接セル方向へのbuiltコストを更新候補に入れる.隣接セルから作業地点方向へのコストも更新
    int startIndex = allocation.start.x * GRID_SIZE_Y + allocation.start.y;
    int goalIndex = allocation.goal.x * GRID_SIZE_Y + allocation.goal.y;
    for (const Coord& direction : DIRECTIONS) {
        if(direction.x == 0 && direction.y == 0) continue; // 自分自身の方向は除外
        int neighborIndex = startIndex + direction.x * GRID_SIZE_Y + direction.y; // 隣接セルのインデックスを取得
        if (neighborIndex < 0 || neighborIndex >= GRID_SIZE_X * GRID_SIZE_Y) continue; // エリア外は除外
        updateBuiltIndexList.push_back({startIndex, DIRECTION_TO_INDEX.at(direction)}); // 作業地点から隣接セル方向へのbuiltコストを更新候補に入れる
        updateBuiltIndexList.push_back({neighborIndex, DIRECTION_TO_INDEX.at({-direction.x, -direction.y})}); // 隣接セルから作業地点方向へのbuiltコストも更新候補に入れる
        updateMoveIndexList.push_back({startIndex, DIRECTION_TO_INDEX.at(direction)}); // 作業地点から隣接セル方向へのmoveコストを更新候補に入れる
        updateMoveIndexList.push_back({neighborIndex, DIRECTION_TO_INDEX.at({-direction.x, -direction.y})}); // 隣接セルから作業地点方向へのmoveコストも更新候補に入れる

        neighborIndex = goalIndex + direction.x * GRID_SIZE_Y + direction.y; // 隣接セルのインデックスを取得
        if (neighborIndex < 0 || neighborIndex >= GRID_SIZE_X * GRID_SIZE_Y) continue; // エリア外は除外
        updateBuiltIndexList.push_back({goalIndex, DIRECTION_TO_INDEX.at(direction)}); // 作業地点から隣接セル方向へのbuiltコストを更新候補に入れる
        updateBuiltIndexList.push_back({neighborIndex, DIRECTION_TO_INDEX.at({-direction.x, -direction.y})}); // 隣接セルから作業地点方向へのbuiltコストも更新候補に入れる
        updateMoveIndexList.push_back({goalIndex, DIRECTION_TO_INDEX.at(direction)}); // 作業地点から隣接セル方向へのmoveコストを更新候補に入れる
        updateMoveIndexList.push_back({neighborIndex, DIRECTION_TO_INDEX.at({-direction.x, -direction.y})}); // 隣接セルから作業地点方向へのmoveコストも更新候補に入れる
    }
}

void updateRoadStatusAfterShovelMove(std::vector<RoadAroundCenter>& roadStatusVector, Allocation allocation,double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],std::vector<CoordPair>& roadList) {
    // 作業が終わった地点は立ち入り禁止(道路の建設予定がある場合は除く)
    Coord start = allocation.start;
    if(soil_amount[start.x][start.y] == 0) {
        int startIndex = start.x * GRID_SIZE_Y + start.y;
        // 作業地点に仮設道路を建設予定がある場合は立ち入り禁止にしない
        bool isRoadPlannedOnStart = false;
        for (const CoordPair& roadpair : roadList) {
            if ((roadpair.coords[0].x == start.x && roadpair.coords[0].y == start.y) || (roadpair.coords[1].x == start.x && roadpair.coords[1].y == start.y)) {
                isRoadPlannedOnStart = true; // 仮設道路の建設予定がある場合は立ち入り禁止にしない
                break; // 予定が見つかったらループを抜ける
            }
        }
        if(!isRoadPlannedOnStart){
            for (const Coord& direction : DIRECTIONS) {
                if (direction.x == 0 && direction.y == 0) continue; // 自分自身の方向は除外
                int neighborIndex = startIndex + direction.x * GRID_SIZE_Y + direction.y; // 隣接セルのインデックスを取得
                if (neighborIndex < 0 || neighborIndex >= GRID_SIZE_X * GRID_SIZE_Y) continue; // エリア外は除外
                roadStatusVector[startIndex].neighbors[DIRECTION_TO_INDEX.at(direction)].type = RoadType::PROHIBITED; // 作業地点の周囲のセルを立ち入り禁止にする
                roadStatusVector[neighborIndex].neighbors[DIRECTION_TO_INDEX.at({-direction.x, -direction.y})].type = RoadType::PROHIBITED; // 隣接セルの立ち入り禁止も更新
            }
        }
    }
    Coord goal = allocation.goal;
    if(soil_amount[goal.x][goal.y] == 0) {
        int goalIndex = goal.x * GRID_SIZE_Y + goal.y;
        // 作業地点に仮設道路を建設予定がある場合は立ち入り禁止にしない
        bool isRoadPlannedOnGoal = false;
        for (const CoordPair& roadpair : roadList) {
            if ((roadpair.coords[0].x == goal.x && roadpair.coords[0].y == goal.y) || (roadpair.coords[1].x == goal.x && roadpair.coords[1].y == goal.y)) {
                isRoadPlannedOnGoal = true; // 仮設道路の建設予定がある場合は立ち入り禁止にしない
                break; // 予定が見つかったらループを抜ける
            }
        }
        if(!isRoadPlannedOnGoal){  
            for (const Coord& direction : DIRECTIONS) {
                if (direction.x == 0 && direction.y == 0) continue; // 自分自身の方向は除外
                int neighborIndex = goalIndex + direction.x * GRID_SIZE_Y + direction.y; // 隣接セルのインデックスを取得
                if (neighborIndex < 0 || neighborIndex >= GRID_SIZE_X * GRID_SIZE_Y) continue; // エリア外は除外
                roadStatusVector[goalIndex].neighbors[DIRECTION_TO_INDEX.at(direction)].type = RoadType::PROHIBITED; // 作業地点の周囲のセルを立ち入り禁止にする
                roadStatusVector[neighborIndex].neighbors[DIRECTION_TO_INDEX.at({-direction.x, -direction.y})].type = RoadType::PROHIBITED; // 隣接セルの立ち入り禁止も更新
            }
        }
    }
}

void printRoadStatusVector(const std::vector<RoadAroundCenter>& roadStatusVector) {
    // Print the roadStatusVector for debugging
    for (int i = 0; i < roadStatusVector.size(); ++i) {
        std::cout << "Center: (" << roadStatusVector[i].center.x << ", " << roadStatusVector[i].center.y << ") Neighbors: ";
        for (const auto& neighbor : roadStatusVector[i].neighbors) {
            std::cout << "{" << neighbor.direction.x << ", " << neighbor.direction.y << ": " << static_cast<int>(neighbor.type) << "} ";
        }
        std::cout << std::endl;
    }
}

void printCostMatirix(const double costMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y]) {
    // Print the costMatrix for debugging
    for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; ++i) {
        std::cout << "index " << i << ": ";
        for (int j = 0; j < GRID_SIZE_X * GRID_SIZE_Y; ++j) {
            std::cout << "to["<< j << "]:"<< costMatrix[i][j] << ", ";
        }
        std::cout << std::endl;
    }
}

void printUpdateIndexList(const std::vector<std::pair<int, int>>& updateIndexList) {
    // Print the updateIndexList for debugging
    for (const auto& indexpair : updateIndexList) {
        std::cout << "Index: " << indexpair.first << ", Direction: " << indexpair.second << std::endl;
    }
}

std::tuple<Result,double,double,double,Solution> processAllocations(std::vector<Allocation>& allocations,
                        const Solution& solution,
                        double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
                        ){
    // Initialize TemporaryRoads
    // double roadStatusMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y] = {0}; // [i][j]はセルiの中心からiとjの中間点への道路の状態を表す. 2:仮設道路 1:未舗装道路 0:なし
    std::vector<RoadAroundCenter> roadStatusVector;
    initializeRoadStatusVector(roadStatusVector);
    TemporaryRoads pavedRoads; //仮設道路の建設状況
    Solution subSolution; //未舗装道路の建設位置，施工順序を記録する
    subSolution.stepNum = solution.stepNum; // ソリューションのステップ数を引き継ぐ
    subSolution.roadNum = 0; // 初期化
    //print solution
    // std::cout << "Solution: " << std::endl;

    std::vector<CoordPair> roadList;
    for (const TemporaryRoadbuildStatus& roadstatus : solution.roadbuildStatusList) {
        CoordPair roadpair = roadstatus.coordpair;
        roadList.push_back(roadpair);
    }

    pavedRoads.initialize(solution);
    double operate_cost = 0;
    double built_length = 0;
    double shovel_move_cost = 0;
    auto result = initiallizedResult(solution);
    int stepNum = solution.stepNum;
    //allocation作業時にトラックが移動する際のコストを計算するための行列
    //道路がない場所を通る際には移動コストに加え，未舗装道路の建設コスト(/trip_num)も加算する
    double costMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y];
    double moveCostMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y]; //移動コストの行列
    double buildCostMatrix[GRID_SIZE_X*GRID_SIZE_Y][GRID_SIZE_X*GRID_SIZE_Y]; //未舗装道路の建設コストの行列
    // std::cout << "pavedRoads initialized" << std::endl;
    // pavedRoads.printParameters();
    setCostMatrix(costMatrix, soil_amount);
    //移動コスト、建設コストの行列は初期状態ではコスト行列と同じ
    for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; ++i) {
        for (int j = 0; j < GRID_SIZE_X * GRID_SIZE_Y; ++j) {
            moveCostMatrix[i][j] = costMatrix[i][j];
            buildCostMatrix[i][j] = costMatrix[i][j];
        }
    }

    // std::cout << "Cost matrix initialized" << std::endl;
    // printCostMatirix(costMatrix); // コスト行列の初期状態を表示


    // 流れ
    // 1. solutionに基づき，各ステップで仮設道路を建設
    // 2. ダンプトラックの移動
    //      -A*アルゴリズムでダンプトラックの最短経路を求める
    //      -経路上に道路がない場合，未舗装道路を建てる
    //      -作業地点と入口がつながっていない場合，未舗装道路で結ぶ
    // 3. 地形変化
    //      -作業地点上の仮設道路を撤去する
    //      -土量を更新する
    // 4. ショベルの移動
    //      -ショベルの移動経路を求める
    //      -経路上に道路がない場合，未舗装道路を建てる
    //      -作業が終了した地点に今後立ち入らないようにする
    // 5. 次のステップへ
    std::vector<std::pair<int, int>> updateBuiltIndexList; // BuiltCostMatrixで更新するインデックスを記録する.<centerのindex, neighborへのdirectionのindex>
    std::vector<std::pair<int, int>> updateMoveIndexList; // MoveCostMatrixで更新するインデックスを記録する
    for (int i = 0; i < stepNum; ++i) {
        // std::cout << "step: " << i << std::endl;

        // 1. solutionに基づき，各ステップで仮設道路を建設
        double built_length_i = 0;
        auto buildPavedRoadIndexList = buildNewRoad(roadStatusVector, i, solution, result.built_length_list, soil_amount, built_length_i);
        built_length += built_length_i;
        updateBuiltIndexList.insert(updateBuiltIndexList.end(), buildPavedRoadIndexList.begin(), buildPavedRoadIndexList.end()); // 仮設道路の座標を更新候補に追加
        updateMoveIndexList.insert(updateMoveIndexList.end(), buildPavedRoadIndexList.begin(), buildPavedRoadIndexList.end()); // 仮設道路の座標を更新候補に追加

        // std::cout << "after buildNewRoad" << std::endl;
        // //print roadStatusVector
        // std::cout << "Road Status Vector after building new roads:" << std::endl;
        // printRoadStatusVector(roadStatusVector); // 道路の状態を表示
        // std::cout << "updateBuiltIndexList after building new roads:" << std::endl;
        // printUpdateIndexList(updateBuiltIndexList); // 更新候補のインデックスを表示
        // std::cout << "updateMoveIndexList after building new roads:" << std::endl;
        // printUpdateIndexList(updateMoveIndexList); // 更新候補のインデックスを表示
        // std::cout << std::endl;

        std::vector<std::pair<int, int>> tempUpdateBuiltIndexList; // ステップ内で増えた更新インデックスを一時的に記録する
        std::vector<std::pair<int, int>> tempUpdateMoveIndexList; 
        // 2. ダンプトラックの移動
        updateBuildMatrixBeforeAstar(buildCostMatrix, roadStatusVector,updateBuiltIndexList,soil_amount); 
        updateMoveMatrixBeforeAstar(moveCostMatrix, roadStatusVector, updateMoveIndexList,soil_amount); 
        // std::cout << "moveCostMatrix before Astar" << std::endl;
        // printCostMatirix(moveCostMatrix); // 移動コスト行列の状態を表示
        // std::cout << "buildCostMatrix before Astar" << std::endl;
        // printCostMatirix(buildCostMatrix); // 建設コスト行列の状態を表示

        double trip_num = 2*std::ceil(allocations[i].volume / V_TRUCK / TRUCK_NUM)-1; // 往復回数込みの運搬回数
        //costMatrix(移動コスト＋未舗装道路の建設コスト/trip_num)を更新
        addMatrix(costMatrix, moveCostMatrix,buildCostMatrix, trip_num); 
        // std::cout << "costMatrix after addMatrix" << std::endl;
        // printCostMatirix(costMatrix); // コスト行列の状態を表示
        // std::cout << std::endl;
         //cost: 移動コスト+ 経路上の未舗装道路の建設コスト
        auto [cost,path] = astar(allocations[i], costMatrix, soil_amount);
       
        if(cost == INF) {
            return {result, INF, built_length, shovel_move_cost,subSolution}; // 無限コストの場合は処理を終了
        }
        //経路上で使用された道路の情報を取得
        auto usedTempList_i = usedTempList(path, pavedRoads); //要修正
        result.used_road_flow[i] = usedTempList_i;
        // result.printUsedRoadFlow();
        result.path[i] = path;
        auto [unpavedBuiltLength, builtSubRoadsIndexList] = updateRoadStatusMatrixandSubRoadsAfterAstar(roadStatusVector, path, soil_amount,subSolution,i);
        // std::cout << "builtSubRoadsIndexList: ";
        // for (const auto& indexpair : builtSubRoadsIndexList) {
        //     std::cout << "{" << indexpair.first << ", " << indexpair.second << "} ";
        // }
        // std::cout << std::endl;

        // builtMatrixの更新候補に追加
        tempUpdateBuiltIndexList.insert(tempUpdateBuiltIndexList.end(), builtSubRoadsIndexList.begin(), builtSubRoadsIndexList.end());
        // std::cout << "unpavedBuiltLength: " << unpavedBuiltLength << std::endl;
        auto builtSubRoadsIndexList2 = connectToEntrance(roadStatusVector, allocations[i], soil_amount, i, subSolution); // 作業地点と入口を仮設道路，未舗装道路で結ぶ．roadStatusVector,subSolutionに記録される
        //buildCostMatrixの更新候補に追加
        tempUpdateBuiltIndexList.insert(tempUpdateBuiltIndexList.end(), builtSubRoadsIndexList2.begin(), builtSubRoadsIndexList2.end());
        operate_cost += cost * trip_num; // 往復回数込みの運搬コスト

        //  3. 地形変化
        // Remove temporary roads
        removeTemporaryRoads(allocations[i], roadStatusVector);
        // Change soil
        changeSoil(allocations[i], soil_amount);
        //作業地点をMoveコスト更新の候補に入れる
        addUpdateIndexList(tempUpdateBuiltIndexList, tempUpdateMoveIndexList, allocations[i]);
        // std::cout << "tempUpdateBuiltIndexList after change soil: " << std::endl;
        // printUpdateIndexList(tempUpdateBuiltIndexList); // 更新候補のインデックスを表示
        // updateCostMatrixAfterTerrainChange(costMatrix, soil_amount, pavedRoads, allocations[i]);

        // std::cout << "costMatrix updated" << std::endl;
        // for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; ++i) {
        //     std::cout << "Row " << i << ": ";
        //     for (int j = 0; j < GRID_SIZE_X * GRID_SIZE_Y; ++j) {
        //         std::cout << costMatrix[i][j] << " ";
        //     }
        //     std::cout << std::endl;
        // }

        // 4. ショベルの移動
        //calculate shovel move cost
        if(i == stepNum - 1) continue; // 最後のステップではショベル移動は行わない
        Allocation shovel_move_allocation;
        shovel_move_allocation.start = allocations[i].goal;
        shovel_move_allocation.goal = allocations[i+1].start;
        shovel_move_allocation.volume = 0;

        updateBuildMatrixBeforeAstar(buildCostMatrix, roadStatusVector, tempUpdateBuiltIndexList,soil_amount); // 未舗装道路の建設コスト行列を更新
        updateMoveMatrixBeforeAstar(moveCostMatrix, roadStatusVector, tempUpdateMoveIndexList,soil_amount); // 移動コスト行列を更新
        // std::cout << "roadStatusVector after terrain change" << std::endl;
        // printRoadStatusVector(roadStatusVector); // 道路の状態を表示

        addMatrix(costMatrix, moveCostMatrix, buildCostMatrix, 1); // コスト行列を更新
        // std::cout << "buildcostMatrix before shovel move" << std::endl;
        // printCostMatirix(buildCostMatrix); // 建設コスト行列の状態を表示
        // std::cout << "moveCostMatrix before shovel move" << std::endl;
        // printCostMatirix(moveCostMatrix); // 移動コスト行列の状態を表示
        // std::cout << "costMatrix after update for shovel move" << std::endl;
        // printCostMatirix(costMatrix); // コスト行列の状態を表示
        auto [shovel_move_cost_i, shovel_move_path] = astar(shovel_move_allocation, costMatrix, soil_amount);
        // std::cout << "shovel_move_cost_i: " << shovel_move_cost_i << std::endl;
        // std::cout << "shovel_move_path: ";
        // shovel_move_path.print();
        shovel_move_cost += shovel_move_cost_i;
   
        
        // updateCostMatrixAfterShovelMove(costMatrix, soil_amount, pavedRoads, allocations[i]);
        updateRoadStatusAfterShovelMove(roadStatusVector, allocations[i], soil_amount,roadList); // 作業が終わった地点は立ち入り禁止にする.shovelのルート上の道路の状態を更新する
        //隣接セル周りの更新なので、buildCostMatrix, moveCostMatrixの更新候補には追加しない（既に追加済み）

        updateBuiltIndexList = tempUpdateBuiltIndexList; // 一時的な更新リストを更新
        updateMoveIndexList = tempUpdateMoveIndexList; // 一時的な更新リストを更新
        // std::cout << std::endl;
        // std::cout << std::endl;
        
    }

    // std::cout << "subSolution after all steps:" << std::endl;
    // subSolution.printParameters(); // 未舗装道路の建設位置，施工順序を表示
    // // std::cout << "Total shovel move cost: " << shovel_move_cost << std::endl;
    // // std::cout << std::endl;
    // // std::cout << std::endl;

    return {result, operate_cost, built_length, shovel_move_cost,subSolution};
}

double costCalculation( double operate_cost,double built_length,double shovel_move_cost) {
    // 運搬距離コスト
    double time_average = operate_cost * GRID_SIZE / (VELOCITY_ROUGH * 1000.0); // 所要時間 (h)
    double cost_construction = COST_HOUR * time_average / WORK_EFF;
    // 仮設道路の建設コスト
    double cost_road = built_length * CONSTRUCTION_TEMP * GRID_SIZE;
    double shovel_time_average = shovel_move_cost * GRID_SIZE / (VELOCITY_SHOVEL * 1000.0); // 所要時間 (h)
    double cost_shovel_move = COST_HOUR * shovel_time_average / WORK_EFF;
    return cost_construction + cost_road + cost_shovel_move;
}

// evaluate_design 関数
std::tuple<Result,double,Solution> evaluate_design(
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
    const Solution& solution
) {
    auto [result, cost_operate, built_length, shovel_move_cost,sub_solution] = processAllocations(allocations, solution, soil_amount);
    double total_cost = costCalculation(cost_operate,built_length,shovel_move_cost);
    return {result, total_cost,sub_solution};
}

std::tuple<Result,double> evaluate_design(
    AllocationOrder allocationOrder,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
    const Solution& solution
) {
    std::vector<Allocation> allocations;
    for (const auto& allocation : allocationOrder.formerAllocations) {
       allocations.push_back(allocation);
    }
    for (const auto& allocation : allocationOrder.latterAllocations) {
       allocations.push_back(allocation);
    }
    auto [result, cost_operate, built_length, shovel_move_cost,sub_Solution] = processAllocations(allocations, solution, soil_amount);
    double total_cost = costCalculation(cost_operate,built_length,shovel_move_cost);
    return {result, total_cost};
}

Solution generatePostionForSolution(
    const Solution& current_solution,
    Result& result,
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
    ) {   
    Solution neighbor_solution = current_solution;

    const std::vector<std::pair<int, int>>& DIRECTIONS = {{-1,-1},{-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
    // stepNum 1: Modify coordpair in the solution
    int modification_type = generateRandomInt(0, 2);

    if (modification_type == 0 || neighbor_solution.roadbuildStatusList.empty()) {
        // std::cout << "Addition" << std::endl;
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
                    neighbor_solution.roadbuildStatusList.begin(),
                    neighbor_solution.roadbuildStatusList.end(),
                    [&](const TemporaryRoadbuildStatus& status) {
                    return (status.coordpair.coords[0].x == search_pair.coords[0].x && 
                            status.coordpair.coords[0].y == search_pair.coords[0].y &&
                            status.coordpair.coords[1].x == search_pair.coords[1].x && 
                            status.coordpair.coords[1].y == search_pair.coords[1].y);
                    }
                );
                 if (it == neighbor_solution.roadbuildStatusList.end()) 
                {   
                    TemporaryRoadbuildStatus new_road;
                    new_road.coordpair = search_pair;
                    new_road.timings.resize(allocations.size(), 0);
                    neighbor_solution.roadbuildStatusList.push_back(new_road);
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
        // std::cout << "Removal" << std::endl;
        // Deletion: Remove a random coordinate pair weighted by its usage and length
            std::vector<double> weights(neighbor_solution.roadNum, 0.0);
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
            neighbor_solution.roadbuildStatusList.erase(neighbor_solution.roadbuildStatusList.begin() + to_remove);
            neighbor_solution.roadNum--;  
            for (int i = 0;i<result.size;i++){
            result.used_road_flow[i].erase(result.used_road_flow[i].begin() + to_remove);
            }
            result.built_length_list.erase(result.built_length_list.begin() + to_remove);
    } else if (modification_type == 2) {
        // std::cout << "Modify" << std::endl;
    // Reassignment: Modify a random coordinate pair, weighted by usage and length
        // Calculate weights for each coordinate pair
        std::vector<double> weights(neighbor_solution.roadNum, 0.0);
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
        neighbor_solution.roadbuildStatusList[to_modify].coordpair = coordpair;
        }


    // stepNum 2: Set all timings to 1 and evaluate the solution
    setAllTimingtoOne(neighbor_solution);
    // auto [local_result,local_operate_cost,local_builtlength] = processAllocations(allocations, neighbor_solution, soil_amount);

    return neighbor_solution;
}

void mergeConnectNetwork(connectNetworkList& network) {
    //各connectNetworkに対して，もし他のconnectNetworkと共通のnodeを持っていたら，mergeする
    for(size_t i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; ++i) {
        std::vector<int> sameNetworkIndexList;
        for(size_t j = 0; j < network.connectNum; ++j) {
            if(network.connectNetworkList[j].nodeList[i] == 1) {
                sameNetworkIndexList.push_back(j);
            }
        }
        // std::cout << std::endl;
        if(sameNetworkIndexList.size() > 1) {
            //同じconnectNetworkに属するモノをmergeする
            connectNetwork mergedNetwork;
            mergedNetwork.nodeList.resize(GRID_SIZE_X * GRID_SIZE_Y, 0);
            mergedNetwork.edgeList.resize(GRID_SIZE_X * GRID_SIZE_Y, std::vector<int>(GRID_SIZE_X * GRID_SIZE_Y, 0));
            for(size_t j = 0; j < sameNetworkIndexList.size(); ++j) {
                for(size_t k = 0; k < GRID_SIZE_X * GRID_SIZE_Y; ++k) {
                    if(network.connectNetworkList[sameNetworkIndexList[j]].nodeList[k] == 1) {
                        mergedNetwork.nodeList[k] = 1;
                        //edgelistの値が1のものがあるときはmergednetworkのedgelistも1にする
                        for(size_t l = k+1; l < GRID_SIZE_X * GRID_SIZE_Y; ++l) {
                            if(network.connectNetworkList[sameNetworkIndexList[j]].nodeList[l] == 1) {
                                if(network.connectNetworkList[sameNetworkIndexList[j]].edgeList[k][l] == 1) {
                                    mergedNetwork.edgeList[k][l] = 1;
                                    mergedNetwork.edgeList[l][k] = 1;
                                }
                            }
                        }
                    }
                }
            }
            //mergedNetworkをconnectNetworkListに追加しsameNetworkIndexList内のindexのconnectNetworkを削除する
            network.connectNetworkList.push_back(mergedNetwork);
            network.connectNum++;
            for(size_t j = 0; j < sameNetworkIndexList.size(); ++j) {
                network.connectNetworkList.erase(network.connectNetworkList.begin() + sameNetworkIndexList[j] - j);
                network.connectNum--;
            }
        }     
    }
}



//仮設道路が入り口と接続するよう配置を修正
Solution adjustPostionForEntryConnectivity(
    const Solution& current_solution,
    std::vector<Allocation> allocations,
    const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
    ) {
        if(current_solution.roadNum == 0){
            return current_solution;
        }

        TemporaryRoads temps;
        temps.initialize(current_solution);
        temps.setStatusToOne();

        //step1: 各connectNetworkが入り口と接続しているかを確認するためのconnectNetworkListを作成

        //tempsのcoordpairを基に全ての道路が接続していない状態で初期化
        connectNetworkList network;
        network.initialize(temps);
        // network.printConverted();

        mergeConnectNetwork(network);
        
        // std::cout << "connectNetworkList after merge" << std::endl;
        // network.printConverted();

        //step2: 各connectNetworkに対して，EntranceIndexを持たないモノに対して，connectToEntranceを行う
        for(size_t i = 0; i < network.connectNum; ++i) {
            if(network.connectNetworkList[i].nodeList[entranceIndex] != 1) {
                //connectToEntranceを行う
                //connectNetworkList[i]のnodeListの中で，entranceとの距離が最短のノードを探す
                double minDistance = INF;
                int minNodeIndex = -1;
                for(size_t j = 0; j < network.connectNetworkList[i].nodeList.size(); ++j) {
                    if(network.connectNetworkList[i].nodeList[j] == 0) {
                        continue;
                    }
                    int x = j / GRID_SIZE_Y;
                    int y = j % GRID_SIZE_Y;
                    int distance = calculate_distance_3D({x,y}, {entranceXPostion, entranceYPostion}, soil_amount);
                    if(distance < minDistance) {
                        minDistance = distance; 
                        minNodeIndex = j;
                    }
                }
                //minNodeIndexとentranceIndexを結ぶpathをastarで求める.仮設道路は全て建設済みとし，temp_effは0とする
                Allocation allocation;
                allocation.start.x = entranceXPostion;
                allocation.start.y = entranceYPostion;
                allocation.goal.x = minNodeIndex / GRID_SIZE_Y;
                allocation.goal.y = minNodeIndex % GRID_SIZE_Y;
                allocation.volume = 1.0;

                //要修正dikjstraを使う
                auto [cost, path,_] = astar(allocation, temps, soil_amount, false);
                
                // path.print();
                //path上のnodeをconnectNetworkList[i]のnodeListに追加し，接続しているedgeをconnectNetworkList[i]のedgeListに追加する
                for(size_t j = 0; j < path.coord.size(); ++j) {
                    int nodeIndex = path.coord[j].x * GRID_SIZE_Y + path.coord[j].y;
                    network.connectNetworkList[i].nodeList[nodeIndex] = 1;
                    if(j > 0) {
                        int pathIndex1 = path.coord[j-1].x * GRID_SIZE_Y + path.coord[j-1].y;
                        int pathIndex2 = path.coord[j].x * GRID_SIZE_Y + path.coord[j].y;
                        //小さい方をindex1にする
                        if(pathIndex1 > pathIndex2) {
                            std::swap(pathIndex1, pathIndex2);
                        }
                        NodePair nodepair;
                        nodepair.node[0] = pathIndex1;
                        nodepair.node[1] = pathIndex2;
                        network.connectNetworkList[i].edgeList[pathIndex1][pathIndex2] = 1;
                        network.connectNetworkList[i].edgeList[pathIndex2][pathIndex1] = 1;

                    }
                }
            }
        }

        mergeConnectNetwork(network);
        
        //step3: networkのedgelistを基にsolutionのcoordpairを修正する
        Solution new_solution;
        new_solution.roadNum = 0;
        new_solution.stepNum = current_solution.stepNum;
        for(size_t i=0; i <network.connectNum; ++i) {
            for(size_t j = 0; j < network.connectNetworkList[i].edgeList.size(); ++j) {
                for(size_t k = 0; k < network.connectNetworkList[i].edgeList[j].size(); ++k) {
                    if(network.connectNetworkList[i].edgeList[j][k] == 1  && (j<k)) {
                        //jとkを結ぶcoordpairをsolutionのcoordpairに追加する
                        CoordPair coordpair;
                        coordpair.coords[0].x = j / GRID_SIZE_Y;
                        coordpair.coords[0].y = j % GRID_SIZE_Y;
                        coordpair.coords[1].x = k / GRID_SIZE_Y;
                        coordpair.coords[1].y = k % GRID_SIZE_Y;
                        TemporaryRoadbuildStatus newRoadStatus;
                        newRoadStatus.coordpair = coordpair;
                        newRoadStatus.timings.resize(allocations.size(), 0);
                        new_solution.roadbuildStatusList.push_back(newRoadStatus);
                        new_solution.roadNum++;
                    }
                }
            }
        }
        return new_solution;
    }

//解のタイミングを生成
Solution generateSolutionTiming(
    const Solution& current_solution,
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
    ) {
        Solution neighbor_solution = current_solution;
        setAllTimingtoOne(neighbor_solution);
        auto [local_result,local_operate_cost,local_builtlength,local_shovel_move_cost,local_sub_solution] = processAllocations(allocations, neighbor_solution, soil_amount);
        // Adjust timings based on used_temp_list
        for (size_t i = 0; i < neighbor_solution.stepNum; ++i) {
            for (size_t j = 0; j < neighbor_solution.roadNum; ++j) {
                if (local_result.used_road_flow[i][j] == 0) {
                    neighbor_solution.roadbuildStatusList[j].timings[i] = 0;
                }
                else {
                    neighbor_solution.roadbuildStatusList[j].timings[i] = 1;
                }
            }
        }
        return neighbor_solution;        
    }

struct Edge {
    int to;
    double cost;
};

using Graph = std::vector<std::vector<Edge>>; //temp上の座標のみノードとして，番号を割り当てる

void printGraph(const Graph& graph){
    std::cout << "Graph" << std::endl;
    // for(size_t i = 0;i<graph.size();i++){
    //     std::cout << "graph[" << i << "]:";
    //     std::cout <<"to:" << graph[i][0].to << " cost:" << graph[i][0].cost ;
    //     for(size_t j = 1;j<graph.size(); j++){
    //         std::cout <<", to:" << graph[i][j].to << " cost:" << graph[i][j].cost ;
    //     }
    //     std::cout << std::endl;
    // }
    for(size_t i = 0;i<graph.size();i++){
        std::cout << "graph[" << i << "]:";
        std::cout << graph[i][0].cost ;
        for(size_t j = 1;j<graph.size(); j++){
            std::cout <<", "  << graph[i][j].cost ;
        }
        std::cout << std::endl;
    }
}

struct CoordToNode{
    Coord coord;
    int node;
};

struct RoadToEdge{
    int node1;
    int node2;
};

std::tuple<std::vector<CoordToNode>,std::vector<RoadToEdge>> setGraph(Graph& graph, const TemporaryRoads& temps, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    // std::vector<std::pair<Coord, int>> coordToNodeList;  //座標をノードに変換する際に，対応する座標とノードを記録したリスト
    // std::vector<std::pair<int,int>> tempsToEdgeList; //仮設道路をエッジ（ノードを結ぶもの）として表現したリスト
    std::vector<CoordToNode> coordToNodeList;  //座標をノードに変換する際に，対応する座標とノードを記録したリスト
    std::vector<RoadToEdge> roadToEdgeList; //仮設道路をエッジ（ノードを結ぶもの）として表現したリスト

    //coordToNodeListを作成
    int coordNodeindex = 0;
    for(size_t i = 0; i < temps.roadNum; ++i) {
        RoadToEdge temp_i_nodePair;
        for(size_t j=0;j<2;j++){
            Coord searchCoord = temps.roadStatusList[i].coordpair.coords[j];
            auto it = std::find_if(
                coordToNodeList.begin(),
                coordToNodeList.end(),
                [&searchCoord](const CoordToNode& entry) {
                    return (entry.coord.x == searchCoord.x && entry.coord.y == searchCoord.y);
                });
    
            if (it != coordToNodeList.end()) {
                    if(j==0){temp_i_nodePair.node1 = it->node;}
                    else if(j==1){temp_i_nodePair.node2 = it->node;}
            }else{
                if(j==0){temp_i_nodePair.node1 = coordNodeindex;}
                else if(j==1){temp_i_nodePair.node2 = coordNodeindex;}
                CoordToNode newCoordNode;
                newCoordNode.coord = searchCoord;
                newCoordNode.node = coordNodeindex;
                coordToNodeList.push_back(newCoordNode);
                coordNodeindex++;
            }
        }
        roadToEdgeList.push_back(temp_i_nodePair);
    }

    // std::cout << "coordToNodeList: " << std::endl;
    for(size_t i = 0; i < coordToNodeList.size();i++){
        Coord coord = coordToNodeList[i].coord;
        int node = coordToNodeList[i].node;
        //coordとnodeを出力
        // std::cout << "coord:(" << coord.x << "," << coord.y << ") -> node: " << node << std::endl;
    }

    // for(size_t i=0; i<roadToEdgeList.size();i++){
    //     // std::cout << "temp_road[" << i << "] node1:" << roadToEdgeList[i].node1 << ", node2:" << roadToEdgeList[i].node2 <<std::endl;
    // }

    //作成したcoordToNodeListとtempsToEdgeListを基にGraphを作成
    //初期化
    Graph initialGraph;
    int graphSize = coordToNodeList.size();
    for(size_t i = 0;i <graphSize;i++){
        std::vector<Edge> edgeList;
        for(size_t j= 0;j<graphSize;j++){
            Edge edge;
            edge.to = j;
            edge.cost = INF;
            edgeList.push_back(edge);
        }
        initialGraph.push_back(edgeList);
    } 
    for(size_t i = 0;i<roadToEdgeList.size();i++){
        double cost = calculate_distance_3D(temps.roadStatusList[i].coordpair.coords[0],temps.roadStatusList[i].coordpair.coords[1],soil_amount);
        initialGraph[roadToEdgeList[i].node1][roadToEdgeList[i].node2].cost = cost;
        initialGraph[roadToEdgeList[i].node2][roadToEdgeList[i].node1].cost = cost;
    }
    graph = initialGraph;
    return {coordToNodeList,roadToEdgeList};
}

//仮設道路の情報をもとに建設済み道路のedgeのコストをゼロにする
void updateGraph(Graph& graph,const TemporaryRoads& temps,std::vector<RoadToEdge> roadToEdgeList,double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    int n = temps.roadNum; 
    for (int i = 0; i < n; ++i) {
        if(temps.roadStatusList[i].status[0] == 1 && temps.roadStatusList[i].status[1] == 1) {
            graph[roadToEdgeList[i].node1][roadToEdgeList[i].node2].cost = 0;
            graph[roadToEdgeList[i].node2][roadToEdgeList[i].node1].cost = 0;
        } else if(temps.roadStatusList[i].status[0] == 1 || temps.roadStatusList[i].status[1] == 1) {
            double cost = calculate_distance_3D(temps.roadStatusList[i].coordpair.coords[0],temps.roadStatusList[i].coordpair.coords[1],soil_amount);
            graph[roadToEdgeList[i].node1][roadToEdgeList[i].node2].cost = cost/2;
            graph[roadToEdgeList[i].node2][roadToEdgeList[i].node1].cost = cost/2;
                }
        else {
            double cost = calculate_distance_3D(temps.roadStatusList[i].coordpair.coords[0],temps.roadStatusList[i].coordpair.coords[1],soil_amount);
            graph[roadToEdgeList[i].node1][roadToEdgeList[i].node2].cost = cost;
            graph[roadToEdgeList[i].node2][roadToEdgeList[i].node1].cost = cost;
        }
            
    }
}

// Dijkstra アルゴリズム
std::vector<int> dijkstra(const Graph& graph, int start, int goal) {
    int n = graph.size();
    std::vector<double> dist(n, INF); // 各ノードまでの最短距離を保存
    std::vector<int> prev(n, -1);    // 経路復元のための前ノード
    dist[start] = 0;

    // 優先度付きキュー (最小値が先頭に来る)
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;
    pq.push({0, start});

    while (!pq.empty()) {
        double currentCost = pq.top().first;
        int currentNode = pq.top().second;
        pq.pop();

        // 既に訪問済みでより短い経路が見つかっている場合はスキップ
        if (currentCost > dist[currentNode]) continue;

        // 隣接ノードの探索
        for (const auto& edge : graph[currentNode]) {
            double nextCost = currentCost + edge.cost;
            if (nextCost < dist[edge.to]) {
                dist[edge.to] = nextCost;
                prev[edge.to] = currentNode; // 経路追跡のための更新
                pq.push({nextCost, edge.to});
            }
        }
    }

    // ゴールに到達するルートを復元
    std::vector<int> path;
    for (int at = goal; at != -1; at = prev[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    double totalCost = dist[goal];
    // std::cout << "cost:" << totalCost << std::endl;
    return path; // 最短経路のノードリスト
}


// BFSを使用して接続されているノードを探索
std::vector<int> bfs(const std::vector<int>& nodeList, const std::vector<std::vector<int>>& edgeList, int startNode, std::vector<bool>& visited) {
    std::vector<int> connectedNodes(nodeList.size(), 0);
    std::queue<int> queue;

    queue.push(startNode);
    visited[startNode] = true;

    while (!queue.empty()) {
        int currentNode = queue.front();
        queue.pop();
        connectedNodes[currentNode] = 1;

        // 隣接ノードを探索
        for (size_t i = 0; i < edgeList.size(); ++i) {
            if (i < currentNode && edgeList[i][currentNode] == 1 && !visited[i]) {
                queue.push(i);
                visited[i] = true;
            } else if (i > currentNode && edgeList[currentNode][i] == 1 && !visited[i]) {
                queue.push(i);
                visited[i] = true;
            }
        }
    }

    return connectedNodes;
}

//接続状態を更新する関数（接続しているノードのペアを追加）
void updateConnectNetwork(connectNetworkList& networkList,std::vector<NodePair> nodepairlist) {

    if(!networkList.isValid()){
        std::cerr << "Error: networkList is invalid ." << std::endl;
        throw std::invalid_argument("networkList size is invalid. Please ensure all dimensions match GRID_SIZE_X and GRID_SIZE_Y and connectNum macthes the size of networkList.");
        std::terminate(); // プログラムを即時終了
    }

    for(size_t i = 0; i < nodepairlist.size(); ++i) {
        //nodepairlist[i]のnodeをconnectNetworkListに追加する
        connectNetwork addNetwork;
        addNetwork.nodeList.resize(GRID_SIZE_X * GRID_SIZE_Y, 0);
        addNetwork.edgeList.resize(GRID_SIZE_X * GRID_SIZE_Y, std::vector<int>(GRID_SIZE_X * GRID_SIZE_Y, 0));
        addNetwork.nodeList[nodepairlist[i].node[0]] = 1;
        addNetwork.nodeList[nodepairlist[i].node[1]] = 1;
        addNetwork.edgeList[nodepairlist[i].node[0]][nodepairlist[i].node[1]] = 1;
        addNetwork.edgeList[nodepairlist[i].node[1]][nodepairlist[i].node[0]] = 1;
        //connectNetworkListに追加する
        networkList.connectNetworkList.push_back(addNetwork);
        networkList.connectNum++;
    }

    // networkList.printConverted();

    mergeConnectNetwork(networkList);
}

// 接続を更新する関数（指定したインデックスの接続ネットワークを更新）
void updateConnectNetwork(connectNetworkList& networkList,int removenode1, int removenode2) {
    connectNetworkList updatedNetworksList = networkList;
    auto updateIndexList = std::vector<int>();
    if(removenode1 > removenode2) {
        std::swap(removenode1, removenode2);
    }

    for(size_t i = 0; i < networkList.connectNum; ++i) {
        // if(networkList.connectNetworkList[i].edgeList[removenode1][removenode2] == 1) {
        //     networkList.connectNetworkList[i].edgeList[removenode1][removenode2] = 0;
        //     networkList.connectNetworkList[i].edgeList[removenode2][removenode1] = 0;
        //     updateIndexList.push_back(i);
        // }
        if(networkList.connectNetworkList[i].nodeList[removenode1] == 1 || networkList.connectNetworkList[i].nodeList[removenode2] == 1) {
            for(size_t j = 0; j < networkList.connectNetworkList[i].edgeList.size(); ++j) {
                networkList.connectNetworkList[i].edgeList[removenode1][j] = 0;
                networkList.connectNetworkList[i].edgeList[j][removenode1] = 0;
                networkList.connectNetworkList[i].edgeList[removenode2][j] = 0;
                networkList.connectNetworkList[i].edgeList[j][removenode2] = 0;
            }
            updateIndexList.push_back(i);
        }
    }
    
    // インデックスリストを降順にソート
    std::sort(updateIndexList.rbegin(), updateIndexList.rend());

   
    
    for(auto updateIndex : updateIndexList) {
        // BFSで接続状態を確認
        connectNetwork network = networkList.connectNetworkList[updateIndex];
 
        std::vector<bool> visited(network.nodeList.size(), false);

        for (size_t i = 0; i < network.nodeList.size(); ++i) {
            if (network.nodeList[i] == 1 && !visited[i]) {
                // BFSで接続されたノードを取得
                std::vector<int> connectedNodes = bfs(network.nodeList, network.edgeList, i, visited);
                // std::cout << "connectedNodes" << std::endl;
                // for(size_t j = 0; j < connectedNodes.size(); ++j) {
                //     std::cout << connectedNodes[j] << " " ;
                // }
                // std::cout << std::endl;

                // サブグラフを構築
                connectNetwork newNetwork;
                newNetwork.nodeList = connectedNodes;

                // エッジ行列を作成
                size_t nodeCount = network.nodeList.size();
                newNetwork.edgeList = std::vector<std::vector<int>>(nodeCount, std::vector<int>(nodeCount, 0));
                int edgeCount = 0;
                for (size_t j = 0; j < nodeCount; ++j) {
                    for (size_t k = j + 1; k < nodeCount; ++k) {
                        if (connectedNodes[j] == 1 && connectedNodes[k] == 1) {
                            newNetwork.edgeList[j][k] = network.edgeList[j][k];
                            newNetwork.edgeList[k][j] = network.edgeList[k][j];
                            if (network.edgeList[j][k] == 1 || network.edgeList[k][j] == 1) {
                            edgeCount++; // エッジが存在する場合にカウント
                            }
                        }
                    }
                }
                // std::cout << "newNetwork" << std::endl;
                if(edgeCount > 0){
                    updatedNetworksList.connectNetworkList.push_back(newNetwork);
                    updatedNetworksList.connectNum++;
                }
            }
        }
        //updatedNetworkからupdateIndexを削除
        updatedNetworksList.connectNetworkList.erase(updatedNetworksList.connectNetworkList.begin() + updateIndex);
        updatedNetworksList.connectNum--;
    }
    
    networkList = updatedNetworksList;
}

//要修正（Graphの更新）

//仮説道路が入り口と接続するようタイミングを修正
// Solution adjustTimingForEntryConnectivity(
//     const Solution& solution,
//     Result result,
//     std::vector<Allocation> allocations,
//     double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
//     ) {

//     Solution new_solution = solution;
//     TemporaryRoads temps;
//     temps.initialize(solution);
//     double operate_cost = 0;
//     double built_length = 0;
//     int stepNum = solution.stepNum;
//     connectNetworkList networkList;
//     networkList.connectNum=0;
//     Graph graph;
//     auto [coordToNodeList,roadToEdgeList] = setGraph(graph,temps,soil_amount);
//     // printGraph(graph);
//     //各ステップで運搬に利用する道路が入口と接続しているかを確認
//     for (int i = 0; i < stepNum; ++i) {
//         std::cout <<std::endl;
//         double built_length_i = 0;
//         result.printBuiltLengthList();
//         buildNewRoad(temps, i, solution, result.built_length_list, soil_amount, built_length_i);
//         std::vector<NodePair> nodepairlist = {};

//         //buildした道路をnetworkListに追加
//         for(int j = 0; j < temps.roadNum; ++j) {
//             if(solution.roadbuildStatusList[j].timings[i]==1){
//                 // std::cout << "i:" << i << ", j:" << j << std::endl;
//                 int nodeIndex1 = temps.roadStatusList[j].coordpair.coords[0].x * GRID_SIZE_Y + temps.roadStatusList[j].coordpair.coords[0].y;
//                 int nodeIndex2 = temps.roadStatusList[j].coordpair.coords[1].x * GRID_SIZE_Y + temps.roadStatusList[j].coordpair.coords[1].y;
//                 if(nodeIndex1 > nodeIndex2) {
//                     std::swap(nodeIndex1, nodeIndex2);
//                 }
//                 nodepairlist.push_back({nodeIndex1, nodeIndex2});
//             }
//         }
//         temps.printParameters();

//         // std::cout << std::endl;
//         // std::cout << "nodepairlist" << std::endl;
//         // for(size_t n = 0;n < nodepairlist.size();n++){
//         //     std::cout << "nodepair[" << n <<"]  node1:" << nodepairlist[n].node[0] << ", node2:" << nodepairlist[n].node[1] << std::endl;
//         // }

//         // std::cout << "before update" << std::endl;
//         // networkList.printConverted();

//         updateConnectNetwork(networkList, nodepairlist);
//         // std::cout << "after update1" << std::endl;
//         // networkList.printConverted();
        
//         //運搬に使用された仮設道路が入り口と接続しているかを確認（generateSolutionTimingで利用する直前に道路を建設するようになっている）
//         for(size_t j = 0; j < solution.roadNum; ++j) {
//             if(solution.roadbuildStatusList[j].timings[i] == 1) {
//                 // std::cout << "Processing step " << i << ", road " << j << std::endl;
//                 int node1 = solution.roadbuildStatusList[j].coordpair.coords[0].x * GRID_SIZE_Y + solution.roadbuildStatusList[j].coordpair.coords[0].y;
//                 int node2 = solution.roadbuildStatusList[j].coordpair.coords[1].x * GRID_SIZE_Y + solution.roadbuildStatusList[j].coordpair.coords[1].y;
//                 Coord entranceCoord = {entranceXPostion,entranceYPostion};
//                 Coord coord1 = solution.roadbuildStatusList[j].coordpair.coords[0];
//                 Coord coord2 = solution.roadbuildStatusList[j].coordpair.coords[1];
//                 double distance1 = calculate_distance_3D(solution.roadbuildStatusList[j].coordpair.coords[0],entranceCoord,soil_amount);
//                 double distance2 = calculate_distance_3D(solution.roadbuildStatusList[j].coordpair.coords[1],entranceCoord,soil_amount);
                
//                 if (node1 > node2) {
//                     std::swap(node1, node2);
//                     std::swap(distance1,distance2);
//                     std::swap(coord1,coord2);
//                 }
//                 Coord connectCoord = (distance1 <= distance2)? coord1 : coord2;
//                 //接続しているかを確認
//                 for(size_t k = 0; k < networkList.connectNum; ++k) {
//                     if(networkList.connectNetworkList[k].edgeList[node1][node2] == 1 && networkList.connectNetworkList[k].nodeList[entranceIndex] != 1) {
//                         //接続していない場合は，入り口に接続するために建設するedgeを求める
//                         int startIndex = 0;
//                         //startNodeをnetworkList.connectNetworkList[k].nodeListの中で，入り口に最も近いノードを探す
//                         double minDistance = INF;
//                         for(size_t l = 0; l < networkList.connectNetworkList[k].nodeList.size(); ++l) {
//                             if(networkList.connectNetworkList[k].nodeList[l] == 0) {
//                                 continue;
//                             }
//                             int x = l / GRID_SIZE_Y;
//                             int y = l % GRID_SIZE_Y;
//                             int distance = calculate_distance_3D({x,y}, {entranceXPostion, entranceYPostion}, soil_amount);
//                             if(distance < minDistance) {
//                                 minDistance = distance; 
//                                 startIndex = l;
//                             }
//                         }
//                         //coordToNodeListからstartIndexに対応する座標を求める
//                         Coord startCoord  = {startIndex / GRID_SIZE_Y, startIndex % GRID_SIZE_Y};
//                         // std::cout << "startCoord: (" << startCoord.x << "," << startCoord.y << ")" << std::endl;
//                        //startCoordのnodeをcoordToNodeListから求める
//                        auto it = std::find_if(
//                         coordToNodeList.begin(),
//                         coordToNodeList.end(),
//                         [&startCoord](const CoordToNode& entry) {
//                             return (entry.coord.x == startCoord.x && entry.coord.y == startCoord.y);
//                         });
                    
//                     if (it == coordToNodeList.end()) {
//                         std::cerr << "Error: startCoord not found in coordToNodeList." << std::endl;
//                         // return new_solution; // またはエラーハンドリング
//                     }
//                     //find entranceNode like startNode
//                     Coord entranceCoord = {entranceXPostion, entranceYPostion};
//                     auto it2 = std::find_if(
//                         coordToNodeList.begin(),
//                         coordToNodeList.end(),
//                         [&entranceCoord](const CoordToNode& entry) {
//                             return (entry.coord.x == entranceCoord.x && entry.coord.y == entranceCoord.y);
//                         });
//                     if (it2 == coordToNodeList.end()) {
//                         std::cerr << "Error: entranceCoord not found in coordToNodeList." << std::endl;
//                         // return new_solution; // またはエラーハンドリング
//                     }
//                     int entranceNode = it2->node;
//                     // std::cout << "entranceNode: " << entranceNode << std::endl;


//                     int startNode = it->node;
//                     // std::cout << "startNode: " << startNode << std::endl;
//                     // std::cout << "entranceIndex: " << entranceIndex << std::endl;
//                         updateGraph(graph,temps,roadToEdgeList,soil_amount);
//                         // printGraph(graph);
//                         auto path = dijkstra(graph,startNode,entranceNode);
//                         // std::cout << "path: ";
//                         // for(size_t m = 0; m < path.size(); ++m) {
//                         //     std::cout << path[m] << " ";
//                         // }
//                         std::cout << std::endl;
//                         //pathに含まれているNodeをnetworkに追加する
//                         std::vector<NodePair> pathNodePairList;
//                         for(size_t m=0;m<path.size()-1;m++){
//                             for(size_t l = 0;l<roadToEdgeList.size();l++){
//                                 if((path[m] == roadToEdgeList[l].node1 &&path[m+1] == roadToEdgeList[l].node2)||(path[m] == roadToEdgeList[l].node2 &&path[m+1] == roadToEdgeList[l].node1)){
                            
//                                     new_solution.roadbuildStatusList[l].timings[i] = 1;
//                                     temps.roadStatusList[l].status[0] = 1;
//                                     temps.roadStatusList[l].status[1] = 1;
//                                     NodePair nodepair;
//                                     int nodepair1 =  solution.roadbuildStatusList[l].coordpair.coords[0].x * GRID_SIZE_Y + solution.roadbuildStatusList[l].coordpair.coords[0].y;
//                                     int nodepair2 = solution.roadbuildStatusList[l].coordpair.coords[1].x * GRID_SIZE_Y + solution.roadbuildStatusList[l].coordpair.coords[1].y;
//                                     nodepair.node[0] = nodepair1;
//                                     nodepair.node[1] = nodepair2;
//                                     pathNodePairList.push_back(nodepair);
//                                 }
//                             }
//                         }

//                         updateConnectNetwork(networkList,pathNodePairList);

//                         //tempsにpathNodePairListにある座標のstatusを1にする
//                         for(size_t m=0;m<pathNodePairList.size();m++){
//                             for(size_t l = 0;l<temps.roadNum;l++){
//                                 int nodepair1 = temps.roadStatusList[l].coordpair.coords[0].x * GRID_SIZE_Y + temps.roadStatusList[l].coordpair.coords[0].y;
//                                 int nodepair2 = temps.roadStatusList[l].coordpair.coords[1].x * GRID_SIZE_Y + temps.roadStatusList[l].coordpair.coords[1].y;
//                                 if((pathNodePairList[m].node[0] == nodepair1 && pathNodePairList[m].node[1] == nodepair2)||(pathNodePairList[m].node[0] == nodepair2 && pathNodePairList[m].node[1] == nodepair1)){
//                                     new_solution.roadbuildStatusList[l].timings[i] = 1;
//                                     temps.roadStatusList[l].status[0] = 1;
//                                     temps.roadStatusList[l].status[1] = 1;
//                                 }
//                             }
//                         }
//                         // temps.printParameters();
//                         // networkList.printConverted();
//                         break;   
//                     }
//             }
//         } 
//         // auto [cost,path,usedTempList_i] = astar(allocations[i], temps,soil_amount,true);
//         }
//         removeTemporaryRoads(allocations[i], temps);
//         //networkListを更新する
//         int removenode1 = allocations[i].start.x * GRID_SIZE_Y + allocations[i].start.y;
//         int removenode2 = allocations[i].goal.x * GRID_SIZE_Y + allocations[i].goal.y;
//         updateConnectNetwork(networkList, removenode1, removenode2);
        
//         // std::cout << "temps before update3" << std::endl;
//         // temps.printParameters();
//         // std::cout << "after update3" << std::endl;
//         // networkList.printConverted();
//         // std::cout << "after step" << i << std::endl;
//     }
//     // std::cout << "new_solution" << std::endl;
//     // new_solution.printParameters();
//     // std::cout << std::endl;
//     return new_solution;
// }

Solution generate_randomPostion(
    const Solution& current_solution,
    Result& result,
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
    ) {   
    Solution neighbor_solution = current_solution;

    const std::vector<std::pair<int, int>>& DIRECTIONS = {{-1,-1},{-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
    // stepNum 1: Modify coordpair in the solution
    int modification_type = generateRandomInt(0, 2);

    if (modification_type == 0 || neighbor_solution.roadbuildStatusList.empty()) {
        // std::cout << "Addition" << std::endl;
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
                    neighbor_solution.roadbuildStatusList.begin(),
                    neighbor_solution.roadbuildStatusList.end(),
                    [&](const TemporaryRoadbuildStatus road_status) {
                        const CoordPair& pair = road_status.coordpair;
                        // Check if the coordinates match the search_pair
                    return pair.coords[0].x == search_pair.coords[0].x &&
                        pair.coords[0].y == search_pair.coords[0].y &&
                        pair.coords[1].x == search_pair.coords[1].x &&
                        pair.coords[1].y == search_pair.coords[1].y;
                    }
                );
                 if (it == neighbor_solution.roadbuildStatusList.end()) 
                {   
                    TemporaryRoadbuildStatus search_status;
                    search_status.coordpair = search_pair;
                    search_status.timings = std::vector<int>(allocations.size(), 1); // Initialize timings to 1
                    neighbor_solution.roadbuildStatusList.push_back(search_status); // Update to push back search_status
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
            std::vector<double> weights(neighbor_solution.roadbuildStatusList.size(), 0.0);
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
            neighbor_solution.roadbuildStatusList.erase(neighbor_solution.roadbuildStatusList.begin() + to_remove);
            neighbor_solution.roadNum--;  
            for (int i = 0;i<result.size;i++){
            result.used_road_flow[i].erase(result.used_road_flow[i].begin() + to_remove);
            }
            result.built_length_list.erase(result.built_length_list.begin() + to_remove);
    } else if (modification_type == 2) {
    // Reassignment: Modify a random coordinate pair, weighted by usage and length
        // Calculate weights for each coordinate pair
        std::vector<double> weights(neighbor_solution.roadbuildStatusList.size(), 0.0);
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
        neighbor_solution.roadbuildStatusList[to_modify].coordpair = coordpair;
        }


    // stepNum 2: Set all timings to 1 and evaluate the solution
    setAllTimingtoOne(neighbor_solution);

    return neighbor_solution;
}

// std::tuple<Solution,Solution,double> generate_neighbor(
//     const Solution& current_solution,
//     Result& result,
//     std::vector<Allocation> allocations,
//     const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
//     ) {   

//     double soil_amount_copy[GRID_SIZE_X][GRID_SIZE_Y];
//     for(int i = 0; i < GRID_SIZE_X; ++i) {
//         for (int j = 0; j < GRID_SIZE_Y; ++j) {
//             soil_amount_copy[i][j] = soil_amount[i][j];
//         }
//     }
//     //print soil_amout_copy
//     std::cout << "soil_amount before generate_randomPostion" << std::endl;
//     for (int i = 0; i < GRID_SIZE_X; ++i) {
//         for (int j = 0; j < GRID_SIZE_Y; ++j) {
//             std::cout << soil_amount[i][j] << " ";
//         }
//         std::cout << std::endl;
//     }
//     result.printPath();
//     Solution neighbor_solution1 = generate_randomPostion(current_solution, result, allocations, soil_amount_copy);
//     //print neighbor_solution
//     // std::cout << "neighbor_solution after random postion change" << std::endl;
//     // neighbor_solution1.printParameters();
//     // std::cout << std::endl;

//     //print soil_amount
//     // std::cout << "soil_amount after generate_randomPostion" << std::endl;
//     // for (int i = 0; i < GRID_SIZE_X; ++i) {
//     //     for (int j = 0; j < GRID_SIZE_Y; ++j) {
//     //         std::cout << soil_amount[i][j] << " ";
//     //     }
//     //     std::cout << std::endl;
//     // }
//     // // adjust position to ensure connectivity to the entrance


//     double soil_amount_copy2[GRID_SIZE_X][GRID_SIZE_Y];
//     for(int i = 0; i < GRID_SIZE_X; ++i) {
//         for (int j = 0; j < GRID_SIZE_Y; ++j) {
//             soil_amount_copy2[i][j] = soil_amount[i][j];
//         }
//     }
//     //print soil_amount_copy2
//     // std::cout << "soil_amount before adjustPostionForEntryConnectivity" << std::endl;
//     // for (int i = 0; i < GRID_SIZE_X; ++i) {
//     //     for (int j = 0; j < GRID_SIZE_Y; ++j) {
//     //         std::cout << soil_amount[i][j] << " ";
//     //     }
//     //     std::cout << std::endl;
//     // }
//     auto neighbor_solution2 = adjustPostionForEntryConnectivity(neighbor_solution1, allocations, soil_amount_copy2);
//     //print neighbor_solution after adjustTimingForEntryConnectivity
//     // std::cout << "neighbor_solution after adjustPositionForEntryConnectivity" << std::endl;
//     // neighbor_solution2.printParameters();
//     // // setAllTimingtoOne(neighbor_solution);
//     // std::cout << std::endl;
    
//     double soil_amount_copy3[GRID_SIZE_X][GRID_SIZE_Y];
//     for(int i = 0; i < GRID_SIZE_X; ++i) {
//         for (int j = 0; j < GRID_SIZE_Y; ++j) {
//             soil_amount_copy3[i][j] = soil_amount[i][j];
//         }
//     }
//     //print soil_amount_copy3
//     // std::cout << "soil_amount before generateSolutionTiming" << std::endl;
//     // for (int i = 0; i < GRID_SIZE_X; ++i) {
//     //     for (int j = 0; j < GRID_SIZE_Y; ++j) {
//     //         std::cout << soil_amount[i][j] << " ";
//     //     }
//     //     std::cout << std::endl;
//     // }
//     auto neighbor_solution3 = generateSolutionTiming(neighbor_solution2, allocations, soil_amount_copy3);
//     // //print neighbor_solution after generateSolutionTiming
//     // std::cout << "neighbor_solution after generateSolutionTiming" << std::endl;
//     // neighbor_solution3.printParameters();
//     // std::cout << std::endl;

//     // //print soil_amount
//     // std::cout << "soil_amount after generateSolutionTiming" << std::endl;
//     // for (int i = 0; i < GRID_SIZE_X; ++i) {
//     //     for (int j = 0; j < GRID_SIZE_Y; ++j) {
//     //         std::cout << soil_amount[i][j] << " ";
//     //     }
//     //     std::cout << std::endl;
//     // }
//     // std::cout << std::endl;
//     //print solution roadNum
//     // std::cout << "solution roadNum: " << neighbor_solution3.roadNum << std::endl;
//     result.init(neighbor_solution3);

//     double soil_amount_copy4[GRID_SIZE_X][GRID_SIZE_Y];
//     for(int i = 0; i < GRID_SIZE_X; ++i) {
//         for (int j = 0; j < GRID_SIZE_Y; ++j) {
//             soil_amount_copy4[i][j] = soil_amount[i][j];
//         }
//     }
//     //adjust timings to ensure connectivity to the entrance
//     //print soil_amount_copy4
//     // std::cout << "soil_amount before adjustTimingForEntryConnectivity" << std::endl;
//     // for (int i = 0; i < GRID_SIZE_X; ++i) {
//     //     for (int j = 0; j < GRID_SIZE_Y; ++j) {
//     //         std::cout << soil_amount[i][j] << " ";
//     //     }
//     //     std::cout << std::endl;
//     // }
//     // //print neighbor_solution3
//     // std::cout << "neighbor_solution before adjustTimingForEntryConnectivity" << std::endl;
//     // neighbor_solution3.printParameters();
//     // // std::cout << std::endl;
//     // //print result
//     // // std::cout << "result before adjustTimingForEntryConnectivity" << std::endl;
//     // result.printBuiltLengthList();
//     // // result.printUsedRoadFlow();
//     // std::cout << "result used_road_flow size: " << result.used_road_flow.size() << std::endl;
//     // std::cout << "result used_road_flow size: " << result.used_road_flow[0].size() << std::endl;
//     auto neighbor_solution4 = adjustTimingForEntryConnectivity(neighbor_solution3, result, allocations, soil_amount_copy4);
//     //print neighbor_solution after adjustTimingForEntryConnectivity
//     // std::cout << "neighbor_solution after adjustTimingForEntryConnectivity" << std::endl;
//     // neighbor_solution4.printParameters();
//     // std::cout << std::endl;


//     double soil_amount_copy5[GRID_SIZE_X][GRID_SIZE_Y];
//     for(int i = 0; i < GRID_SIZE_X; ++i) {
//         for (int j = 0; j < GRID_SIZE_Y; ++j) {
//             soil_amount_copy5[i][j] = soil_amount[i][j];
//         }
//     }
//     // Evaluate the new solution
//     auto [new_result,total_cost] = evaluate_design(allocations,soil_amount_copy5, neighbor_solution4);
//     return {neighbor_solution1,neighbor_solution4,total_cost};
// }


bool isRoadAllocated(const Coord& coord, const Solution& solution) {
    for (const auto& road : solution.roadbuildStatusList) {
        if (road.coordpair.coords[0].x == coord.x && road.coordpair.coords[0].y == coord.y ||
            road.coordpair.coords[1].x == coord.x && road.coordpair.coords[1].y == coord.y) {
            return true;
        }
    }
    return false;
}

void customExplore(const Graph& graph, int entranceNode, std::vector<int>& sortedNodes) {
    int numNodes = graph.size();
    std::vector<bool> visited(numNodes, false); // ノードの訪問管理
    std::stack<int> mainStack;                 // メインルートを探索するスタック
    std::queue<int> branchQueue;               // 分岐ルートを保存するキュー

    // 開始ノードをプッシュ
    mainStack.push(entranceNode);
    int loopCount = 0; 
    while (!mainStack.empty() || !branchQueue.empty()) {
        std::cout << "Loop count: " << loopCount++ << std::endl; // デバッグ用
        int currentNode;
        if(currentNode){        
            std::cout << "currentNode: " <<currentNode << std::endl;
        } else {
            std::cout << "currentNode is empty" << std::endl;
        }
        //print mainstack
        // std::cout << "Main stack: ";
        std::stack<int> tempStack = mainStack;
        // while (!tempStack.empty()) {
        //     std::cout << tempStack.top() << " ";
        //     tempStack.pop();
        // }
        // std::cout << std::endl;
        //print branchQueue
        // std::cout << "Branch queue: ";
        // std::queue<int> tempQueue = branchQueue;
        // while (!tempQueue.empty()) {
        //     std::cout << tempQueue.front() << " ";
        //     tempQueue.pop();
        // }
        std::cout << std::endl;
        //print visited
        // std::cout << "Visited nodes: ";
        // for (int i = 0; i < numNodes; ++i) {
        //     if (visited[i]) {
        //         std::cout << i << " ";
        //     }
        // }
        // std::cout << std::endl;

        if (!mainStack.empty()) {
            currentNode = mainStack.top();
            mainStack.pop();
        } else {
            currentNode = branchQueue.front();
            branchQueue.pop();
        }

        // 既に訪問済みのノードはスキップ
        if (visited[currentNode]) continue;

        // ノードを訪問済みにしてソート結果に追加
        visited[currentNode] = true;
        sortedNodes.push_back(currentNode);

        // 隣接ノードを取得して接続順にソート
        std::vector<Edge> neighbors;
        for (const auto& edge : graph[currentNode]) {
            if (edge.cost != std::numeric_limits<double>::infinity()) {
                neighbors.push_back(edge);
            }
        }
        
        std::sort(neighbors.begin(), neighbors.end(), [](const Edge& a, const Edge& b) {
            return a.cost < b.cost; // コスト順でソート
        });


        // 最初のルートをメインに追加、それ以外を分岐キューに追加
        bool firstRoute = true;
        for (const auto& neighbor : neighbors) {
            if (!visited[neighbor.to]) {
                if (firstRoute) {
                    mainStack.push(neighbor.to);
                    firstRoute = false;
                } else {
                    branchQueue.push(neighbor.to);
                }
            }
        }
    }
}

//allocationのリストに対しいてヒューリスティックにallocationの順番を決める関数
AllocationOrder generateAllocationOrderwithHeuristicRule(const std::vector<Allocation>& allocations,const Solution& solution,const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    AllocationOrder allocationOrder;
    //仮設道路が配置されているセルへの土砂配分はformerに入れる
    std::vector<Allocation> formerAllocations;
    std::vector<Allocation> latterAllocations;
    for(const auto& allocation : allocations) {
        //仮設道路が配置されているセルへの土砂配分はformerに入れる
        if (isRoadAllocated(allocation.start, solution) || isRoadAllocated(allocation.goal, solution)) {
            formerAllocations.push_back(allocation);
        } else {
            latterAllocations.push_back(allocation);
        }
    }

    //laterAllocations：入り口セルから遠い順にソート
    std::sort(latterAllocations.begin(), latterAllocations.end(), [&](const Allocation& a, const Allocation& b) {
        int distanceStartA = calculate_distance_2D(a.start, {entranceXPostion, entranceYPostion}, soil_amount);
        int distanceStartB = calculate_distance_2D(b.start, {entranceXPostion, entranceYPostion}, soil_amount);
        int distanceGoalA = calculate_distance_2D(a.goal, {entranceXPostion, entranceYPostion}, soil_amount);
        int distanceGoalB = calculate_distance_2D(b.goal, {entranceXPostion, entranceYPostion}, soil_amount);
        
        int minDistanceA = std::min(distanceStartA, distanceGoalA);
        int minDistanceB = std::min(distanceStartB, distanceGoalB);
        int maxDistanceA = std::max(distanceStartA, distanceGoalA);
        int maxDistanceB = std::max(distanceStartB, distanceGoalB);

       // 大きい方を比較
    if (maxDistanceA != maxDistanceB) {
        return maxDistanceA > maxDistanceB;
    }
    // 小さい方が同じ場合は、大きい方を比較
    return minDistanceA > minDistanceB;
    });

    if(solution.roadNum != 0) {
    //formerAllocations：入り口セルからの仮設道路の接続ネットワークを確認し，入り口から近いセルに土砂配分を行っている順にソート
    //solutionのtemporaryRoadbuildStatusListから入り口セルとの接続を確認
    TemporaryRoads temps;
    temps.initialize(solution);
    temps.setStatusToOne();
    // connectNetworkList network;
    // network.initialize(temps);
    // network.printConverted();
    // mergeConnectNetwork(network);
        // network.printConverted();

    //仮設道路が入り口と接続している前提で，入り口セルから順に各仮設道路がおかれるセルが入り口から近い順（接続順）にソートする
    //例）入り口セルが(0,0),仮設道路が　{(0,0),(1,0)},{(0,0),(0,1)},{(1,0),(2,0)},{(0,1),(0,2)},{(2,0),(3,0)}　に配置されている場合、
    //{(0,0),(1,0)},{(0,0),(0,1)},{(0,1),(0,2)},{(1,0),(2,0)},{(2,0),(3,0)}の順にソートされる(分岐する場合は)
    Graph graph; 
    double soil_amount_fake[GRID_SIZE_X][GRID_SIZE_Y] = {0}; 
    auto [coordToNodeList,roadToEdgeList] = setGraph(graph,temps,soil_amount_fake); 
    printGraph(graph);

    // //print coordToNodeList
    // std::cout << "coordToNodeList:" << std::endl;
    // for (const auto& entry : coordToNodeList) {
    //     std::cout << "Coord: (" << entry.coord.x << ", " << entry.coord.y << "), Node: " << entry.node << std::endl;
    // }
    // entranceのcoordのnodeを求める
    int entranceNode = -1;
    Coord entranceCoord = {entranceXPostion, entranceYPostion};
    for (const auto& entry : coordToNodeList) {
        if (entry.coord.x == entranceCoord.x && entry.coord.y == entranceCoord.y) {
            entranceNode = entry.node;
            break;
        }
    }
    if(entranceNode == -1) {
        std::cerr << "Error: entranceCoord not found in coordToNodeList." << std::endl;
        exit(1); // エラー終了
    }

    //入り口ノードからgraphを基にソートしていく
    std::vector<int> sortedNodes;
    customExplore(graph, entranceNode, sortedNodes);
    //print sortedNodes
    // std::cout << "sortedNodes: ";
    // for (const auto& node : sortedNodes) {
    //     std::cout << node << " ";
    // }
    // std::cout << std::endl;
   
    auto tempFormerAllocations = formerAllocations;
    std::vector<Allocation> tempFormerAllocations2; // 一時的にformerAllocationsを保存するための変数
    //coordToNodelistを基に、sortedNodesの順番でformerAllocationsをソートする
    for(const auto& node : sortedNodes) {
        //nodeに対応するCoordを求める
        Coord coord = coordToNodeList[node].coord;
        //coordに対応するAllocationを求める.複数ある場合は，もう一つの点が入り口から近いもの順にsortする
        std::vector<std::pair<Allocation, int>> matchingAllocations; //startがマッチした場合hは0, goalがマッチした場合は1を格納する
        auto it = tempFormerAllocations.begin();
        while (it != tempFormerAllocations.end()) {
            if (it->start.x == coord.x && it->start.y == coord.y) {
                matchingAllocations.emplace_back(*it, 0);
                it = tempFormerAllocations.erase(it);  // erase は次のイテレータを返す
            } else if (it->goal.x == coord.x && it->goal.y == coord.y) {
                matchingAllocations.emplace_back(*it, 1);
                it = tempFormerAllocations.erase(it);
            } else {
                ++it;
            }
        }
        
        // std::cout << std::endl;
        if(matchingAllocations.size() >1){
            //matchingAllocationsの中から、もう一つの点が入り口から近いもの順にソートする
            std::sort(matchingAllocations.begin(), matchingAllocations.end(), [&](const std::pair<Allocation, int>& a, const std::pair<Allocation, int>& b) {
                int distanceA = (a.second == 1) ? calculate_distance_2D(a.first.start, {entranceXPostion, entranceYPostion}, soil_amount) : calculate_distance_2D(a.first.goal, {entranceXPostion, entranceYPostion}, soil_amount);
                int distanceB = (b.second == 1) ? calculate_distance_2D(b.first.start, {entranceXPostion, entranceYPostion}, soil_amount) : calculate_distance_2D(b.first.goal, {entranceXPostion, entranceYPostion}, soil_amount);
                return distanceA < distanceB;
            });
        }
       

        //matchingAllocationsの中身をformerAllocationsに追加
       for (const auto& alloc : matchingAllocations) {
           tempFormerAllocations2.push_back(alloc.first);
       }
       
    }
    formerAllocations = tempFormerAllocations2; // 最終的にformerAllocationsに格納する
}
    //allocationOrderにformerAllocationsとlaterAllocationsを追加
    allocationOrder.formerAllocations = formerAllocations;
    allocationOrder.latterAllocations = latterAllocations;

    return allocationOrder;
}


//allocationのリストに対しいてヒューリスティックにallocationの順番を決める関数
std::vector<Allocation> initializeAllocation(const std::vector<Allocation>& allocations, const Solution& solution, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    // formerAllocations: 仮設道路が配置されているセルへの配分
    // latterAllocations: その他の配分
    std::vector<Allocation> formerAllocations;
    std::vector<Allocation> latterAllocations;

    for (const auto& allocation : allocations) {
        if (isRoadAllocated(allocation.start, solution) || isRoadAllocated(allocation.goal, solution)) {
            formerAllocations.push_back(allocation);
        } else {
            latterAllocations.push_back(allocation);
        }
    }

    // latterAllocations: 入り口から遠い順にソート
    std::sort(latterAllocations.begin(), latterAllocations.end(), [&](const Allocation& a, const Allocation& b) {
        int distanceStartA = calculate_distance_2D(a.start, {entranceXPostion, entranceYPostion}, soil_amount);
        int distanceStartB = calculate_distance_2D(b.start, {entranceXPostion, entranceYPostion}, soil_amount);
        int distanceGoalA = calculate_distance_2D(a.goal, {entranceXPostion, entranceYPostion}, soil_amount);
        int distanceGoalB = calculate_distance_2D(b.goal, {entranceXPostion, entranceYPostion}, soil_amount);

        int minDistanceA = std::min(distanceStartA, distanceGoalA);
        int minDistanceB = std::min(distanceStartB, distanceGoalB);
        int maxDistanceA = std::max(distanceStartA, distanceGoalA);
        int maxDistanceB = std::max(distanceStartB, distanceGoalB);

        if (maxDistanceA != maxDistanceB) return maxDistanceA > maxDistanceB;
        return minDistanceA > minDistanceB;
    });

    if (solution.roadNum != 0) {
        // formerAllocations: 入り口から近い順にソート
        std::sort(formerAllocations.begin(), formerAllocations.end(), [&](const Allocation& a, const Allocation& b) {
            int distanceStartA = calculate_distance_2D(a.start, {entranceXPostion, entranceYPostion}, soil_amount);
            int distanceStartB = calculate_distance_2D(b.start, {entranceXPostion, entranceYPostion}, soil_amount);
            int distanceGoalA = calculate_distance_2D(a.goal, {entranceXPostion, entranceYPostion}, soil_amount);
            int distanceGoalB = calculate_distance_2D(b.goal, {entranceXPostion, entranceYPostion}, soil_amount);

            int minDistanceA = std::min(distanceStartA, distanceGoalA);
            int minDistanceB = std::min(distanceStartB, distanceGoalB);
            int maxDistanceA = std::max(distanceStartA, distanceGoalA);
            int maxDistanceB = std::max(distanceStartB, distanceGoalB);

            if (minDistanceA != minDistanceB) return minDistanceA < minDistanceB;
            return maxDistanceA < maxDistanceB;
        });
    }

    // former → latter の順に連結
    std::vector<Allocation> allocationOrder;
    allocationOrder.insert(allocationOrder.end(), formerAllocations.begin(), formerAllocations.end());
    allocationOrder.insert(allocationOrder.end(), latterAllocations.begin(), latterAllocations.end());

    return allocationOrder;
}



AllocationOrder initallizedAllocationOrder(const std::vector<Allocation>& allocations, const Solution& solution){
    AllocationOrder newOrder;
    for(const auto& allocation : allocations) {
        //仮設道路が配置されているセルへの土砂配分はformerに入れる
        if (isRoadAllocated(allocation.start, solution) || isRoadAllocated(allocation.goal, solution)) {
            newOrder.formerAllocations.push_back(allocation);
        } else {
            newOrder.latterAllocations.push_back(allocation);
        }
    }
    return newOrder;
}

std::vector<Allocation> randomizedAllocations(const std::vector<Allocation>& allocations, const Solution& solution){
    std::vector<Allocation> newAllocations = allocations;
    //shuffleする
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(newAllocations.begin(), newAllocations.end(), g);
    return newAllocations;
}

AllocationOrder randomizedAllocationOrder(const AllocationOrder& allocationOrder){
    //formerAllocationとlatterAllocationそれぞれをshuffleしてランダム化する
    AllocationOrder randomizedOrder = allocationOrder;
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(randomizedOrder.formerAllocations.begin(), randomizedOrder.formerAllocations.end(), g);
    std::shuffle(randomizedOrder.latterAllocations.begin(), randomizedOrder.latterAllocations.end(), g);
    return randomizedOrder;
}

AllocationOrder generateNeighborAllocationOrder(
    const AllocationOrder& current_order,
    const Solution& solution,
    const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
) {
    //latterAllocationsとformerAllocationsの優先度を崩さずに変更
    AllocationOrder neighbor_order = current_order;
    //ランダムにformerAllocationsとlatterAllocationsのどちらを変更するか決定.
    int modification_type = generateRandomInt(0, 1); // 0: formerAllocations, 1: latterAllocations
    int operation_type = generateRandomInt(0, 2); // 0: swap, 1: insert, 2: reversal
    std::vector<Allocation> targetAllocations = (modification_type == 0) ? neighbor_order.formerAllocations : neighbor_order.latterAllocations;
    if(targetAllocations.empty() || targetAllocations.size() < 2) {
        modification_type = 1-modification_type; // 変更するアロケーションがない場合は、もう一方のリストを変更する
        targetAllocations = (modification_type == 0) ? neighbor_order.formerAllocations : neighbor_order.latterAllocations;
        if(targetAllocations.empty() || targetAllocations.size() < 2) {
            std::cout << "No allocations to modify." << std::endl;
            return neighbor_order; // 変更するアロケーションがない場合はそのまま返す
        }
    }
    if (modification_type == 0) {
        std::cout << "Modifying formerAllocations." << std::endl;
    } else {
        std::cout << "Modifying latterAllocations." << std::endl;
    }
    switch (operation_type) {
        case 0: { // Swap
            std::cout << "Swap operation selected." << std::endl;
            int index1 = generateRandomInt(0, targetAllocations.size() - 1);
            int index2 = generateRandomInt(0, targetAllocations.size() - 1);
            while (index1 == index2) {
                index2 = generateRandomInt(0, targetAllocations.size() - 1);
            }
            // std::cout << "Swapping indices: " << index1 << " and " << index2 << std::endl;
            
            std::swap(targetAllocations[index1], targetAllocations[index2]);
            break;
        }
        case 1: { // Insert
            std::cout << "Insert operation selected." << std::endl;
            int index = generateRandomInt(0, targetAllocations.size() - 1);
            int insertIndex = generateRandomInt(0, targetAllocations.size()-1);
            while (insertIndex == index) {
                insertIndex = generateRandomInt(0, targetAllocations.size()-1);
            }
            // std::cout << "Inserting allocation from index " << index << " to index " << insertIndex << std::endl;
            Allocation allocation = targetAllocations[index];
            targetAllocations.erase(targetAllocations.begin() + index);
            targetAllocations.insert(targetAllocations.begin() + insertIndex, allocation);
            break;
        }
        case 2: { // Reversal
            std::cout << "Reversal operation selected." << std::endl;

            int start = generateRandomInt(0, targetAllocations.size() - 2); // -2 to ensure at least one element can be reversed
            int end = generateRandomInt(start + 2, targetAllocations.size());
            // std::cout << "Reversing from index " << start << " to index " << end << std::endl;
            std::reverse(targetAllocations.begin() + start, targetAllocations.begin() + end);
            break;
        }
        
    }
    //更新されたtargetAllocationsをneighbor_orderに反映
    if (modification_type == 0) {
        neighbor_order.formerAllocations = targetAllocations;
    } else {
        neighbor_order.latterAllocations = targetAllocations;
    }
    return neighbor_order;
}


std::vector<Allocation> generateNeighborAllocations(
    const std::vector<Allocation>& current_allocations,
    const Solution& solution,
    const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
) {
    std::vector<Allocation> neighbor_allocations = current_allocations;
    //ランダムにformerAllocationsとlatterAllocationsのどちらを変更するか決定.
    int operation_type = generateRandomInt(0, 2); // 0: swap, 1: insert, 2: reversal
    if(neighbor_allocations.empty() || neighbor_allocations.size() < 2) {
            std::cout << "No allocations to modify." << std::endl;
            return neighbor_allocations; // 変更するアロケーションがない場合はそのまま返す
    }
    switch (operation_type) {
        case 0: { // Swap
            std::cout << "Swap operation selected." << std::endl;
            int index1 = generateRandomInt(0, neighbor_allocations.size() - 1);
            int index2 = generateRandomInt(0, neighbor_allocations.size() - 1);
            while (index1 == index2) {
                index2 = generateRandomInt(0, neighbor_allocations.size() - 1);
            }
            // std::cout << "Swapping indices: " << index1 << " and " << index2 << std::endl;

            std::swap(neighbor_allocations[index1], neighbor_allocations[index2]);
            break;
        }
        case 1: { // Insert
            std::cout << "Insert operation selected." << std::endl;
            int index = generateRandomInt(0, neighbor_allocations.size() - 1);
            int insertIndex = generateRandomInt(0, neighbor_allocations.size() - 1);
            while (insertIndex == index) {
                insertIndex = generateRandomInt(0, neighbor_allocations.size() - 1);
            }
            // std::cout << "Inserting allocation from index " << index << " to index " << insertIndex << std::endl;
            Allocation allocation = neighbor_allocations[index];
            neighbor_allocations.erase(neighbor_allocations.begin() + index);
            neighbor_allocations.insert(neighbor_allocations.begin() + insertIndex, allocation);
            break;
        }
        case 2: { // Reversal
            std::cout << "Reversal operation selected." << std::endl;
            int start = generateRandomInt(0, neighbor_allocations.size() - 2); // -2 to ensure at least one element can be reversed
            int end = generateRandomInt(start + 2, neighbor_allocations.size());
            // std::cout << "Reversing from index " << start << " to index " << end << std::endl;
            std::reverse(neighbor_allocations.begin() + start, neighbor_allocations.begin() + end);
            break;
        }   
    }
    return neighbor_allocations;
}


// void simulatedAnnealingForOrderOptimization(std::vector<Allocation>& allocations,  Solution& solution, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
//     //allocationの順番を最適化
//     //random initialization
//     Solution current_solution = solution;
//     // AllocationOrder current_order = initallizedAllocationOrder(allocations, current_solution);
//     // current_order = randomizedAllocationOrder(current_order);
//     AllocationOrder current_order = generateAllocationOrderwithHeuristicRule(allocations, solution, soil_amount);

//     auto [current_result,current_score] = evaluate_design(current_order,soil_amount, current_solution);
//     std::cout << "initial_score: " << current_score << std::endl;
//     std::cout << "initial solution:" << std::endl;
//     current_solution.printParameters();
//     std::cout << "initial_order:" << std::endl;
//     current_order.printParameters();
//     if(current_score !=INF) {
//     std::cout << "initial_result:" << std::endl;
//     current_result.printPath();
//     }else{
//         std::cout << "initial_result is infeasible." << std::endl;
//     }
//     AllocationOrder best_order = current_order;
//     double best_score = current_score;
//     Result best_result = current_result;
//     int best_score_loop = 0;
//     Solution best_solution = current_solution;

//     // std::vector<std::pair<double, Solution>> best_score_flow;
//     // std::vector<std::pair<double, Solution>> current_score_flow;
//     // std::vector<std::pair<double, Solution>> neighbor_score_flow;
//     // std::vector<std::pair<double, Solution>> neighbor_solution_flow;

//     int temperature = initialTemperature;
//     for (int iter = 0; iter < max_iter; ++iter) {
//         //reset soil_amount copy
//         double soil_amount_copy[GRID_SIZE_X][GRID_SIZE_Y];
//         for (int i = 0; i < GRID_SIZE_X; i++) {
//             for (int j = 0; j < GRID_SIZE_Y; j++) {
//                 soil_amount_copy[i][j] = soil_amount[i][j];
//             }
//         }
//         temperature *= alpha;

//         //print current solution and result
//         auto neighbor_order = generateNeighborAllocationOrder(current_order, current_solution, soil_amount_copy);
//         std::vector<Allocation> neighbor_allocations;
//         neighbor_allocations.insert(neighbor_allocations.end(), neighbor_order.formerAllocations.begin(), neighbor_order.formerAllocations.end());
//         neighbor_allocations.insert(neighbor_allocations.end(), neighbor_order.latterAllocations.begin(), neighbor_order.latterAllocations.end());
//         auto neighbor_solution = generateSolutionTiming(current_solution, neighbor_allocations, soil_amount_copy);
//         auto [neighbor_result, neighbor_score] = evaluate_design(neighbor_order, soil_amount_copy, neighbor_solution);
//         std::cout << "neighbor_score: " << neighbor_score << std::endl;
//         std::cout << "neithbor_solution:" << std::endl;
//         neighbor_solution.printParameters();
//         std::cout << "neighbor_order:" << std::endl;
//         neighbor_order.printParameters();
//         std::cout << "neighbor_result:" << std::endl;
//         neighbor_result.printPath();
//         std::cout << std::endl;
//         double random_value = generateRandomDouble(0.0, 1.0);

//         if(neighbor_score == INF){
//             std::cout << "Neighbor solution is infeasible, skipping acceptance." << std::endl;
//             continue; // If the neighbor solution is infeasible, skip acceptance
//         }
//         // 受け入れ判定
//         if ((neighbor_score < current_score) || 
//             (random_value < std::exp(-(std::abs(neighbor_score - current_score)) / temperature))) {
//             current_order = neighbor_order;
//             current_score = neighbor_score;
//             current_result = neighbor_result;
//             current_solution = neighbor_solution;
//         }
        
//         std::cout <<"current _score: " << current_score << std::endl;
//         std::cout << "current_order:" << std::endl;
//         current_order.printParameters();
//         std::cout << "current_result:" << std::endl;
//         current_result.printPath();

//         // ベスト解の更新
//         if (current_score < best_score) {
//             best_order = current_order;
//             best_score = current_score;
//             best_score_loop = iter;
//             best_result = current_result;
//             best_solution = current_solution;
//         }
//         std::cout <<std::endl;
//         std::cout <<std::endl;
//     }


//     //print best solution and result
//     std::cout << "best_solution:" << std::endl;
//     best_solution.printParametersForPlot();
//     std::cout << "best_order:" << std::endl;
//     best_order.printParametersForPlot();
//     std::cout << "best_score: " << best_score << std::endl;
//     best_result.printPathForPlot(best_order.orderToVector());
//     std::cout << "best_score_loop: " << best_score_loop << std::endl;
// }

// void simulatedAnnealingForOrderOptimizationWithNoRestriction(std::vector<Allocation>& allocations, Solution& solution, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
//     //allocationの順番を最適化
//     //formerAllocationsとlatterAllocationsの優先度は崩してもよい条件で
//     //random initialization
//     Solution current_solution = solution;
//     std::cout << "before randomizedAllocations" << std::endl;
//     auto current_allocations = randomizedAllocations(allocations, current_solution);

//     //initialization with heuristic rule
//     // AllocationOrder current_order = generateAllocationOrderwithHeuristicRule(allocations, solution, soil_amount);
//     // std::vector<Allocation> current_allocations = current_order.formerAllocations;
//     // current_allocations.insert(current_allocations.end(), current_order.latterAllocations.begin(), current_order.latterAllocations.end());
    
//     std::cout << "before generateSolutionTiming" << std::endl;
//     current_solution = generateSolutionTiming(current_solution, current_allocations, soil_amount);
    
//     std::cout << "after generateSolutionTiming" << std::endl;
//     auto [current_result,current_score] = evaluate_design(current_allocations,soil_amount, solution);
//     std::cout << "after evaluate_design" << std::endl;
//     std::cout << "initial_score: " << current_score << std::endl;
//     std::cout << "initial_order:" << std::endl;
//     for(const auto& alloc : current_allocations) {
//         std::cout << "Allocation: start(" << alloc.start.x << "," << alloc.start.y << ") goal(" << alloc.goal.x << "," << alloc.goal.y << ") volume: " << alloc.volume << std::endl;
//     }
//     if(current_score !=INF) {
//     std::cout << "initial_result:" << std::endl;
//     current_result.printPath();
//     }else{
//         std::cout << "initial_result is infeasible." << std::endl;
//     }
//     auto  best_allocations = current_allocations;
//     auto best_solution = current_solution;
//     double best_score = current_score;
//     Result best_result = current_result;
//     int best_score_loop = 0;

//     // std::vector<std::pair<double, Solution>> best_score_flow;
//     // std::vector<std::pair<double, Solution>> current_score_flow;
//     // std::vector<std::pair<double, Solution>> neighbor_score_flow;
//     // std::vector<std::pair<double, Solution>> neighbor_solution_flow;

//     int temperature = initialTemperature;
//     for (int iter = 0; iter < max_iter; ++iter) {
//         //reset soil_amount copy
//         double soil_amount_copy[GRID_SIZE_X][GRID_SIZE_Y];
//         for (int i = 0; i < GRID_SIZE_X; i++) {
//             for (int j = 0; j < GRID_SIZE_Y; j++) {
//                 soil_amount_copy[i][j] = soil_amount[i][j];
//             }
//         }
//         temperature *= alpha;

//         //print current solution and result
//         auto neighbor_allocations = generateNeighborAllocations(current_allocations, current_solution, soil_amount_copy);
//         auto neighbor_solution = generateSolutionTiming(current_solution, neighbor_allocations, soil_amount_copy);
//         auto [neighbor_result, neighbor_score] = evaluate_design(neighbor_allocations, soil_amount_copy, neighbor_solution);
//         // std::cout << "neighbor_score: " << neighbor_score << std::endl;
//         // std::cout << "neighbor_allocations:" << std::endl;
//         // for(const auto& alloc : neighbor_allocations) {
//         //     std::cout << "Allocation: start(" << alloc.start.x << "," << alloc.start.y << ") goal(" << alloc.goal.x << "," << alloc.goal.y << ") volume: " << alloc.volume << std::endl;
//         // }
//         // std::cout << "neighbor_result:" << std::endl;
//         // neighbor_result.printPath();
//         // std::cout << std::endl;
//         // std::cout << "neighbor_solution:" << std::endl;
//         // neighbor_solution.printParameters();
//         double random_value = generateRandomDouble(0.0, 1.0);

//         if(neighbor_score == INF){
//             std::cout << "Neighbor solution is infeasible, skipping acceptance." << std::endl;
//             continue; // If the neighbor solution is infeasible, skip acceptance
//         }
//         // 受け入れ判定
//         if ((neighbor_score < current_score) || 
//             (random_value < std::exp(-(std::abs(neighbor_score - current_score)) / temperature))) {
//             if(neighbor_score < current_score) {
//                 std::cout << "Accepting neighbor solution with better score." << std::endl;
//             } 
//             current_allocations = neighbor_allocations;
//             current_score = neighbor_score;
//             current_result = neighbor_result;
//             current_solution = neighbor_solution;
//         }
        
//         // std::cout <<"current _score: " << current_score << std::endl;
//         // std::cout << "current_allocations:" << std::endl;
//         // for(const auto& alloc : current_allocations) {
//         //     std::cout << "Allocation: start(" << alloc.start.x << "," << alloc.start.y << ") goal(" << alloc.goal.x << "," << alloc.goal.y << ") volume: " << alloc.volume << std::endl;
//         // }
//         // std::cout << "current_result:" << std::endl;
//         // current_result.printPath();

//         // ベスト解の更新
//         if (current_score < best_score) {
//             best_allocations = current_allocations;
//             best_score = current_score;
//             best_score_loop = iter;
//             best_result = current_result;
//             best_solution = current_solution;
//         }
//         std::cout <<std::endl;
//         std::cout <<std::endl;
//     }

//     //print best solution and result
//     std::cout << "best_solution:" << std::endl;
//     best_solution.printParametersForPlot();
//     std::cout << "best_allocations:" << std::endl;
//     for(const auto& alloc : best_allocations) {
//         std::cout << "Allocation: start(" << alloc.start.x << "," << alloc.start.y << ") goal(" << alloc.goal.x << "," << alloc.goal.y << ") volume: " << alloc.volume << std::endl;
//     }
//     std::cout << "best_score: " << best_score << std::endl;
//     best_result.printPathForPlot(best_allocations);
//     std::cout << "best_score_loop: " << best_score_loop << std::endl;
// }

std::tuple<Solution,std::vector<Allocation>> generateNeighborAllocationsAndTiming(
    const Solution& current_solution,
    const std::vector<Allocation>& current_allocations
) {
    Solution neighbor_solution= current_solution;
    auto neighbor_allocations = current_allocations;

    int operationType = generateRandomInt(0, 2); // 0: timing, 1: allocation, 2: both

    switch (operationType) {
        case 0: {
            // 🎯 タイミングのみを反転（0 <-> 1）
            int numChanges = generateRandomInt(1, 2); // 1～2個の道路を変更
            for (int i = 0; i < numChanges; ++i) {
                int roadIdx = generateRandomInt(0, neighbor_solution.roadNum - 1);
                int stepIdx = generateRandomInt(0, neighbor_solution.roadbuildStatusList[roadIdx].timings.size() - 1);
                int& timing = neighbor_solution.roadbuildStatusList[roadIdx].timings[stepIdx];
                timing = (timing == 0) ? 1 : 0; // 反転
            }
            break;
        }

        case 1: {
            // 🔁 Allocation順をシャッフル（2点スワップ）
            int i = generateRandomInt(0, neighbor_allocations.size() - 1);
            int j = generateRandomInt(0, neighbor_allocations.size() - 1);
            std::swap(neighbor_allocations[i], neighbor_allocations[j]);
            break;
        }

        case 2: {
            // 🔗 タイミングとアロケーションの両方を連動させる

            // Allocationのスワップ
            int i = generateRandomInt(0, neighbor_allocations.size() - 1);
            int j = generateRandomInt(0, neighbor_allocations.size() - 1);
            std::swap(neighbor_allocations[i], neighbor_allocations[j]);

            // 各道路の i と j の timing を入れ替える（ステップ単位）
            for (int roadIdx = 0; roadIdx < neighbor_solution.roadNum; ++roadIdx) {
                auto& timings = neighbor_solution.roadbuildStatusList[roadIdx].timings;
                if (i < timings.size() && j < timings.size()) {
                    std::swap(timings[i], timings[j]);
                }
            }

            break;
        }
    }

    return {neighbor_solution, neighbor_allocations};
}

void simulatedAnnealingForAllOptimization(std::vector<Allocation>& allocations, Solution& solution, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    Solution current_solution = solution;

    auto [current_result, current_score,current_sub_solution] = evaluate_design(allocations, soil_amount, current_solution);
    std::cout << "initial_score: " << current_score << std::endl;
    std::cout << "initial solution:" << std::endl;
    current_solution.printParameters();

    if (current_score != INF) {
        std::cout << "initial_result:" << std::endl;
        current_result.printPath();
    } else {
        std::cout << "initial_result is infeasible." << std::endl;
    }

    std::vector<Allocation> current_allocations = allocations;
    auto best_allocations = current_allocations;
    double best_score = current_score;
    Result best_result = current_result;
    Solution best_solution = current_solution;
    Solution best_sub_solution = current_sub_solution;

    int temperature = initialTemperature;

    for (int iter = 0; iter < max_iter; ++iter) {
        std::cout << "=== Iteration " << iter << " ===" << std::endl;
        temperature *= alpha;

        double soil_amount_copy[GRID_SIZE_X][GRID_SIZE_Y];
        for (int i = 0; i < GRID_SIZE_X; i++)
            for (int j = 0; j < GRID_SIZE_Y; j++)
                soil_amount_copy[i][j] = soil_amount[i][j];

        // 新しいポジション生成
        auto neighbor_solution = generate_randomPostion(current_solution, current_result, current_allocations, soil_amount_copy);

        // 初期の割当順序生成
        auto neighbor_allocations = initializeAllocation(current_allocations, neighbor_solution, soil_amount_copy);

        // 初期のタイミング生成
        neighbor_solution = generateSolutionTiming(neighbor_solution, neighbor_allocations, soil_amount_copy);

        // 内部 SA ループ
        int inter_temperature = initialTemperature;
        int inter_max_iter = neighbor_solution.roadNum * 200;
        Solution local_solution = neighbor_solution;
        std::vector<Allocation> local_allocations = neighbor_allocations;
        auto [local_result, local_score,local_sub_solution] = evaluate_design(local_allocations, soil_amount_copy, local_solution);
        double beta = 0.90;

        for (int k = 0; k < inter_max_iter; ++k) {
            std::cout << "=== Inner Iteration " << k << "=== " << local_score << std::endl;
            inter_temperature *= beta;

            double soil_amount_local[GRID_SIZE_X][GRID_SIZE_Y];
            for (int i = 0; i < GRID_SIZE_X; i++)
                for (int j = 0; j < GRID_SIZE_Y; j++)
                    soil_amount_local[i][j] = soil_amount[i][j];

            // 近傍生成
            auto [ neighbor_sol,neighbor_alloc] = generateNeighborAllocationsAndTiming(local_solution, local_allocations);

            // 評価
            auto [neighbor_result, neighbor_score,neighbor_sub_solution] = evaluate_design(neighbor_alloc, soil_amount_local, neighbor_sol);

            if (neighbor_score == INF) continue;

            double random_value = generateRandomDouble(0.0, 1.0);
            if (neighbor_score < local_score || random_value < std::exp(-(std::abs(neighbor_score - local_score)) / inter_temperature)) {
                local_solution = neighbor_sol;
                local_allocations = neighbor_alloc;
                local_score = neighbor_score;
                local_result = neighbor_result;
                local_sub_solution = neighbor_sub_solution;
            }
        }

        // outer loop に反映
        if (local_score < current_score) {
            current_solution = local_solution;
            current_allocations = local_allocations;
            current_score = local_score;
            current_result = local_result;
            current_sub_solution = local_sub_solution;
        }

        std::cout << "[Iter " << iter << "] Score: " << current_score << std::endl;

        if (current_score < best_score) {
            best_score = current_score;
            best_solution = current_solution;
            best_result = current_result;
            best_allocations = current_allocations;
            best_sub_solution = current_sub_solution;
        }
    }

    std::cout << "=== Best Result ===" << std::endl;
    best_solution.printParameters();
    best_result.printPath();
    //print best allocations
    std::cout << "Best Allocations:" << std::endl;
    for (const auto& alloc : best_allocations) {
        std::cout << "Allocation: start(" << alloc.start.x << "," << alloc.start.y << ") goal(" << alloc.goal.x << "," << alloc.goal.y << ") volume: " << alloc.volume << std::endl;
    }
    std::cout << "Best Score: " << best_score << std::endl;
    std::cout << "Best Sub Solution:" << std::endl;
    best_sub_solution.printParameters();
}


int main() {
    auto start = std::chrono::high_resolution_clock::now();
    
    //Initialize soil_amount
    // double soil_amount[GRID_SIZE_X][GRID_SIZE_Y] =
    // {
    //     {-126200.0, -62600.0, -3700.0, 0.0},
    //     {-3700.0, 22500.0, 22500.0, -1400.0},
    //     {3700.0, 33800.0, 28100.0, 2300.0},
    //     {9000.0, 36000.0, 23000.0, 1200.0},
    //     {9000.0, 23000.0, 24300.0, 9000.0},
    //     {8000.0, 22200.0, 14200.0, -5900.0},
    //     {-1000.0, 8100.0, -12400.0, -9900.0},
    //     {-11200.0, -24800.0, -34400.0, -2700.0}
    // };

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
    // double soil_amount[GRID_SIZE_X][GRID_SIZE_Y] =
    // {
    //     {1,0,-1,0},
    //     {0,-1,0,-1},
    //     {-1,0,2,0},
    //     {0,0,1,0},
    // };
    //   double soil_amount[GRID_SIZE_X][GRID_SIZE_Y] =
    // {
    //     {1,0,-1,0},
    //     {0,0,2,-1},
    //     {1,-1,1,-1},
    //     {0,-1,-1,1},
    // };
    // double soil_amount[GRID_SIZE_X][GRID_SIZE_Y] =
    // {
    //     {0,0,0,0},
    //     {0,0,0,0},
    //     {0,0,2,0},
    //     {0,0,1,0},
    //     {0,0,0,0},
    //     {0,0,0,0},
    //     {0,0,0,0},
    //     {0,-1,-3,-1}
    // };
    // double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]  =
    // {
    //     {1,0},
    //     {0,-1}
    // };
    //check if sum of soil_amount is zero
    checkSoilAmountTotal(soil_amount);
    // Initialize allocations
    //   std::vector<Allocation> allocations = 
    // {
    //     {{0, 0}, {2, 0}, 1},
    //     {{2, 2}, {0, 2}, 1},
    //     {{2, 2}, {1, 1}, 1},
    //     {{3, 2}, {1, 3}, 1},
    // };

    // std::vector<Allocation> allocations = 
    // {
    //     {{2, 2}, {2, 3}, 1},
    //     {{2, 0}, {2, 1}, 1},
    //     {{3, 3}, {1, 3}, 1},
    //     {{0, 0}, {0, 2}, 1},
    //     {{1, 2}, {3, 1}, 1},
    //     {{1, 2}, {3, 2}, 1}
    // };

    // std::vector<Allocation> allocations = 
    // {
    //     {{2, 2}, {7, 3}, 2700.0},
    //     {{4, 1}, {7, 2}, 5600.0},
    //     {{3, 2}, {7, 2}, 22100.0},
    //     {{2, 2}, {7, 2}, 6700.0},
    //     {{5, 1}, {7, 1}, 22200.0},
    //     {{3, 1}, {7, 1}, 2600.0},
    //     {{4, 0}, {7, 0}, 5000.0},
    //     {{3, 1}, {7, 0}, 6200.0},
    //     {{4, 3}, {6, 3}, 9000.0},
    //     {{3, 2}, {6, 3}, 900.0},
    //     {{2, 2}, {6, 2}, 12400.0},
    //     {{3, 0}, {6, 0}, 1000.0},
    //     {{3, 3}, {5, 3}, 1200.0},
    //     {{2, 3}, {5, 3}, 900.0},
    //     {{2, 2}, {5, 3}, 3800.0},
    //     {{2, 3}, {1, 3}, 1400.0},
    //     {{2, 2}, {0, 1}, 2500.0},
    //     {{1, 2}, {0, 2}, 3700.0},
    //     {{2, 1}, {1, 0}, 3700.0},
    //     {{2, 1}, {0, 1}, 30100.0},
    //     {{1, 2}, {0, 1}, 18800.0},
    //     {{2, 0}, {0, 0}, 3700.0},
    //     {{1, 1}, {0, 1}, 11200.0},
    //     {{1, 1}, {0, 0}, 11300.0}
    // };

std::vector<Allocation> allocations = {
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
    CoordPair coordpair1 = {{{0, 2},{1, 2}}};
    CoordPair coordpair2 = {{{1, 2},{2, 2}}};
    CoordPair coordpair3 = {{{2, 2},{3, 2}}};
    // CoordPair coordpair4 = {{{3, 2},{4, 2}}};
    // CoordPair coordpair5 = {{{4, 2},{5, 2}}};
    // CoordPair coordpair6 = {{{5, 2},{6, 2}}};
    // CoordPair coordpair7 = {{{6, 2},{7, 2}}};
    // CoordPair coordpair8 = {{{2, 1},{3, 1}}};
    // CoordPair coordpair9 = {{{3, 1},{4, 1}}};
    // CoordPair coordpair10 = {{{4, 1},{5, 1}}};
    // CoordPair coordpair11 = {{{5, 1},{6, 1}}};
    // CoordPair coordpair12 = {{{6, 1},{7, 1}}};
    // CoordPair coordpair13 = {{{7, 1},{8, 1}}};
    // CoordPair coordpair14 = {{{8, 1},{9, 1}}};
    // CoordPair coordpair15 = {{{5, 2},{6, 1}}};



        //normalize coordpair
    normalize_pair(coordpair1);
    normalize_pair(coordpair2);
    normalize_pair(coordpair3);
    // normalize_pair(coordpair4);
    // normalize_pair(coordpair5);
    // normalize_pair(coordpair6);
    // normalize_pair(coordpair7);
    // normalize_pair(coordpair8);
    // normalize_pair(coordpair9);
    // normalize_pair(coordpair10);
    // normalize_pair(coordpair11);
    // normalize_pair(coordpair12);
    // normalize_pair(coordpair13);
    // normalize_pair(coordpair14);
    // normalize_pair(coordpair15);





    std::vector<CoordPair> coordpairlist;
    // coordpairlist.push_back(coordpair1);
    // coordpairlist.push_back(coordpair2);
    // coordpairlist.push_back(coordpair3);
    // coordpairlist.push_back(coordpair4);
    // coordpairlist.push_back(coordpair5);
    // coordpairlist.push_back(coordpair6);
    // coordpairlist.push_back(coordpair7);
    // coordpairlist.push_back(coordpair8);
    // coordpairlist.push_back(coordpair9);
    // coordpairlist.push_back(coordpair10);
    // coordpairlist.push_back(coordpair11);
    // coordpairlist.push_back(coordpair12);
    // coordpairlist.push_back(coordpair13);
    // coordpairlist.push_back(coordpair14);
    // coordpairlist.push_back(coordpair15);

    Solution initSolution = initializedSolution(stepNum,coordpairlist);
    TemporaryRoads temps;
    temps.initialize(initSolution);
    
    setAllTimingtoOne(initSolution);
    double soil_amount_copy[GRID_SIZE_X][GRID_SIZE_Y];
        for (int i = 0; i < GRID_SIZE_X; i++) {
            for (int j = 0; j < GRID_SIZE_Y; j++) {
                soil_amount_copy[i][j] = soil_amount[i][j];
            }
        }

    // //test generateAllocationOrderwithHeuristicRule
    // auto allocationOrder = generateAllocationOrderwithHeuristicRule(allocations, initSolution, soil_amount);
    // std::cout << "Allocation Order:" << std::endl;
    // std::cout << "Former Allocations:" << std::endl;
    // for (const auto& allocation : allocationOrder.formerAllocations) {
    //     std::cout << "  From: (" << allocation.start.x << ", " << allocation.start.y << ") "
    //               << "To: (" << allocation.goal.x << ", " << allocation.goal.y << ") "
    //               << "Amount: " << allocation.volume << std::endl;
    // }
    // std::cout << "Latter Allocations:" << std::endl;
    // for (const auto& allocation : allocationOrder.latterAllocations) {
    //     std::cout << "  From: (" << allocation.start.x << ", " << allocation.start.y << ") "
    //               << "To: (" << allocation.goal.x << ", " << allocation.goal.y << ") "
    //               << "Amount: " << allocation.volume << std::endl;
    // }

    double soil_amount_temp[GRID_SIZE_X][GRID_SIZE_Y] = {0};
    TemporaryRoads temps_copy;
    temps_copy.initialize(initSolution);
    temps_copy.setStatusToOne();

    // initSolution = generateSolutionTiming(initSolution, allocations, soil_amount_temp);

    initSolution.printParameters();
    //test astar
    int roadNum = initSolution.roadNum;
    // test simulatedAnnealingForOrderOptimization
    // simulatedAnnealingForOrderOptimization(allocations, initSolution, soil_amount_copy);
    // simulatedAnnealingForOrderOptimizationWithNoRestriction(allocations, initSolution, soil_amount_copy);

    // //test processAllocation
    // auto [result,operate_cost,builtlength,shovel_move_cost] = processAllocations(allocations, initSolution, soil_amount_copy);
    // std::cout << "Result:" << std::endl;
    // result.printPath();
    // std::cout << "Operate Cost: " << operate_cost << std::endl;
    // std::cout << "Built Length: " << builtlength << std::endl;
    // std::cout << "Shovel Move Cost: " << shovel_move_cost << std::endl;

    //test simulatedAnnealingForAllOptimization
    simulatedAnnealingForAllOptimization(allocations, initSolution, soil_amount_copy);
    return 0;
}
