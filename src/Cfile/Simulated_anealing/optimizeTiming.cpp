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
#include <cassert>

// Objective function parameters
#define GRID_SIZE_X 4
#define GRID_SIZE_Y 4
#define DIRECTIONS_COUNT 8
#define INF std::numeric_limits<double>::infinity()


// #define VELOCITY_ROUGH 24.0
#define VELOCITY_ROUGH 10.0  //km/h
#define VELOCITY_PAVE 40.0  //km/h
#define TEMP_EFF VELOCITY_ROUGH/VELOCITY_PAVE // Temporary road efficiency
#define GRID_SIZE 150 //m
#define TRUCK_CAPACITY 40.0  //m^3
#define TRUCK_NUM 1 
#define COST_HOUR 2000.0  // $/h 
#define WORK_EFF 0.75  
#define CONSTRUCTION_TEMP 17.5  // $/m

#define entranceXPostion 0
#define entranceYPostion 2
#define entranceIndex entranceXPostion * GRID_SIZE_Y + entranceYPostion

// Simulated Annealing parameters
#define alpha 0.95
#define max_iter 10
#define initialTemperature 1000.0

struct Coord {
    int x, y;
};

struct CoordPair {
    Coord coords[2];
};

struct TemporaryRoadStatus {
    int status[2];
    CoordPair coordpair;
};



struct TemporaryRoadbuildStatus {
    CoordPair coordpair;
    std::vector<int> timings;
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
};

struct CoordPairWithTiming {
    CoordPair coordpair;
    std::vector<int> timing;
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
    std::vector<Path> path; // 各ステップの経路
    std::vector<std::vector<int>> roadTransportedVolumeByStep;  //roadTransportedVolumeByStep[stepNum][road]: // 各ステップでの各道路がどれだけ土量を運んだか
    std::vector<double> built_length_list;

    void init(const Solution& solution){
        size = solution.stepNum;
        // built_length_list.resize(solution.roadNum,0);
        // roadTransportedVolumeByStep.resize(solution.stepNum, std::vector<int>(solution.roadNum, 0)); 
        built_length_list.assign(solution.roadNum, 0); // 初期化済みのベクタを確保
        roadTransportedVolumeByStep.assign(solution.stepNum, std::vector<int>(solution.roadNum, 0)); // 二重ベクタを確保
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
        for (int i = 0; i < roadTransportedVolumeByStep.size(); i++) {
            std::cout << "step[" << i << "]: ";
            for (int j = 0; j < roadTransportedVolumeByStep[i].size(); j++) {
                std::cout << roadTransportedVolumeByStep[i][j] << " ";
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

Result initiallizedResult(Solution solution){
    Result result;
    result.size = solution.stepNum;
    result.path.resize(solution.stepNum);
    result.roadTransportedVolumeByStep.resize(solution.stepNum);
    for (int i = 0; i < solution.stepNum; i++) {
        result.roadTransportedVolumeByStep[i].resize(solution.roadNum);
        for (int j = 0; j < solution.roadNum; j++) {
            result.roadTransportedVolumeByStep[i][j] = 0;
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


double heuristic(const Coord& a, const Coord& b, const  double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    return TEMP_EFF * calculate_distance_3D(a, b, soil_amount);
}


void changeSoil(const Allocation& allocation, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    soil_amount[allocation.start.x][allocation.start.y] -= allocation.volume;
    soil_amount[allocation.goal.x][allocation.goal.y] += allocation.volume;
}

//あるステップにおいて道路を建設した場合,current_tempsの状態を更新し、built_lengthに建設した道路の長さを追加する
//道路建設機械の移動コストは無視
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
    // Coord entranceCoord = {entranceXPostion, entranceYPostion};
    // Coord closestCoord;
    // double closestDistance = INF;
    for (int i = 0; i < road_count; i++) {
        CoordPair road = solution.roadbuildStatusList[i].coordpair;
        if (solution.roadbuildStatusList[i].timings[stepNum] == 1) {
            // for (const Coord& coord : road.coords) {
            //     double distance = calculate_distance_2D(coord, entranceCoord, soil_amount);
            //     if (distance < closestDistance) {
            //         closestDistance = distance;
            //         closestCoord = coord;
            //     }
            // }

            double length = calculate_distance_3D(road.coords[0], road.coords[1], soil_amount)/2;
            if (current_temps.roadStatusList[i].status[0] == 0&&current_temps.roadStatusList[i].status[1] == 0) {
                built_length += length*2;
                built_length_list[i] += length*2;
                current_temps.roadStatusList[i].status[0] = 1;
                current_temps.roadStatusList[i].status[1] = 1;
                builtRoadList.push_back(i);
            }else if (current_temps.roadStatusList[i].status[1] == 0||current_temps.roadStatusList[i].status[1] == 0) {
                built_length += length;
                built_length_list[i] += length;
                current_temps.roadStatusList[i].status[0] = 1;
                current_temps.roadStatusList[i].status[1] = 1;
                builtRoadList.push_back(i);
            }

        }
    }

    // if(closestDistance !=INF){
    //     //entranceCoordからclosestCoordへの経路距離をbuilt_lengthに追加
    //     std::tuple<double, Path, std::vector<int>> entranceToClosestResult = astar(
    // {entranceCoord, closestCoord, 0.0}, 
    // current_temps, 
    // soil_amount, 
    // true
    // ); // searchFlagをfalseにして、temporary_roadsの状態を考慮しない

    // double entranceToClosestCost = std::get<0>(entranceToClosestResult);
    // built_length += entranceToClosestCost;
    // }

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
            neighbor_distance *= temp_eff;
        } else if (temporary_roads.roadStatusList[index].status[0] == 1 || temporary_roads.roadStatusList[index].status[1] == 1) {
            neighbor_distance *= temp_eff;
        }
    }
    return current_distance + neighbor_distance;
}

std::tuple<double,Path,std::vector<int>> astar(const Allocation& allocation, const TemporaryRoads& temps, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],bool searchFlag = true) {   
    std::vector<int> transportedVolumeList(temps.roadNum, 0); // Initialize used_temp_list with zeros
    double temp_eff = searchFlag ? TEMP_EFF : 0;   
    //searchFlagがtrueの場合は移動経路を計算する用．falseの場合はconnectNetwork用(建ててある道路の建設コストは0とする)
    Coord start = allocation.start;
    Coord goal = allocation.goal;
    double volume = allocation.volume;
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
            // if (temps.roadStatusList[index].status[0] == 1 && temps.roadStatusList[index].status[1] == 1) {
            //     // std::cout << "index: " << index << " is used(both)" << std::endl;
            //     used_temp_list[index] = 2;
            // } else if 
            //     (temps.roadStatusList[index].status[0] == 1|| temps.roadStatusList[index].status[1] == 1) {
            //     // std::cout << "index: " << index << " is used" << std::endl;
            //     used_temp_list[index] = 1;
            // }
            // else {
            //     // std::cout << "index: " << index << " is not used" << std::endl;
            //     used_temp_list[index] = 0;
            // }

            if (temps.roadStatusList[index].status[0] == 1 || temps.roadStatusList[index].status[1] == 1) {
                // std::cout << "index: " << index << " is used(both)" << std::endl;
                transportedVolumeList[index] = allocation.volume; // 道路が使用されている場合、運搬量を記録
            } 
            else {
                // std::cout << "index: " << index << " is not used" << std::endl;
                transportedVolumeList[index] = 0;
            }
        }
    }

        current = next;
    }
    reverse_path.push_back(start);

    // 結果をPathに反映
    path.coord.assign(reverse_path.rbegin(), reverse_path.rend());
    // total_cost = cost_so_far[goal.x][goal.y] * volume/ TRUCK_CAPACITY / TRUCK_NUM; //往復回数込みのコスト
    total_cost = cost_so_far[goal.x][goal.y] * volume; //往復回数込みのコスト
    return {total_cost, path, transportedVolumeList}; 
}



std::tuple<Result,double,double> processAllocations(std::vector<Allocation>& allocations,
                        const Solution& solution,
                        double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
                        ) {
    // Initialize TemporaryRoads
    TemporaryRoads temps;
    temps.initialize(solution);
    double operate_cost = 0;
    double built_length = 0;
    auto result = initiallizedResult(solution);
    int stepNum = solution.stepNum;

    for (int i = 0; i < stepNum; ++i) {
        // std::cout << "step: " << i << std::endl;
        double built_length_i = 0;
        buildNewRoad(temps, i, solution, result.built_length_list, soil_amount, built_length_i);
        // std::cout<<"after build"<<std::endl;
        built_length += built_length_i;
        // temps.printParameters();
        auto [cost,path,transportedVolumeList_i] = astar(allocations[i], temps,soil_amount,true);
        // std::cout<<"after astar"<<std::endl;

        result.roadTransportedVolumeByStep[i] = transportedVolumeList_i;
        // result.printUsedRoadFlow();
        result.path[i] = path;
        operate_cost += cost;
        // Remove temporary roads
        removeTemporaryRoads(allocations[i], temps);
        // std::cout<<"after remove"<<std::endl;

        // Change soil
        changeSoil(allocations[i], soil_amount);
    }
    return {result, operate_cost, built_length};
}

double costCalculation( double operate_cost,double built_length) {
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


// evaluate_design 関数
std::tuple<Result,double> evaluate_design(
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
    const Solution& solution
) {
    auto [result, cost_operate, built_length] = processAllocations(allocations, solution, soil_amount);
    double total_cost = costCalculation(cost_operate,built_length);
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
            result.roadTransportedVolumeByStep[i].push_back(0);
        }
        result.built_length_list.push_back(0);
    } else if (modification_type == 1) {
        // std::cout << "Removal" << std::endl;
        // Deletion: Remove a random coordinate pair weighted by its usage and length
            std::vector<double> weights(neighbor_solution.roadNum, 0.0);
            double total_weight = 0.0;
            for (int i = 0; i < neighbor_solution.roadNum; ++i) {
                double usage = result.roadTransportedVolumeByStep[0][i];
                for (int j = 1; j < result.roadTransportedVolumeByStep.size(); ++j) {
                    usage += result.roadTransportedVolumeByStep[j][i];
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
            result.roadTransportedVolumeByStep[i].erase(result.roadTransportedVolumeByStep[i].begin() + to_remove);
            }
            result.built_length_list.erase(result.built_length_list.begin() + to_remove);
    } else if (modification_type == 2) {
        // std::cout << "Modify" << std::endl;
    // Reassignment: Modify a random coordinate pair, weighted by usage and length
        // Calculate weights for each coordinate pair
        std::vector<double> weights(neighbor_solution.roadNum, 0.0);
        double total_weight = 0.0;
            for (int i = 0; i < neighbor_solution.roadNum; ++i) {
                double usage = result.roadTransportedVolumeByStep[0][i];
                for (int j = 1; j < result.roadTransportedVolumeByStep.size(); ++j) {
                    usage += result.roadTransportedVolumeByStep[j][i];}
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

        //接続している道路を同じconnectNetworkにまとめる
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
                //要修正（heuristic関数temp_effを0にする必要あり？）
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

        // std::cout <<"neighbor_solution";
        // neighbor_solution.printParameters();

        auto [local_result,local_operate_cost,local_builtlength] = processAllocations(allocations, neighbor_solution, soil_amount);

        // Print local_result.roadTransportedVolumeByStep
        // local_result.printPath();
        // std::cout << "roadTransportedVolumeByStep" << std::endl;
        // local_result.printUsedRoadFlow();

        // stepNum 3: Adjust timings based on used_temp_list
        for (size_t i = 0; i < neighbor_solution.stepNum; ++i) {
            for (size_t j = 0; j < neighbor_solution.roadNum; ++j) {
                if (local_result.roadTransportedVolumeByStep[i][j] == 0) {
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
    for(size_t i = 0;i<graph.size();i++){
        std::cout << "graph[" << i << "]:";
        std::cout <<"to:" << graph[i][0].to << " cost:" << graph[i][0].cost ;
        for(size_t j = 1;j<graph.size(); j++){
            std::cout <<", to:" << graph[i][j].to << " cost:" << graph[i][j].cost ;
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
Solution adjustTimingForEntryConnectivity(
    const Solution& solution,
    Result result,
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
    ) {
    Solution new_solution = solution;
    TemporaryRoads temps;
    temps.initialize(solution);
    double operate_cost = 0;
    double built_length = 0;
    int stepNum = solution.stepNum;
    connectNetworkList networkList;
    networkList.connectNum=0;
    Graph graph;
    auto [coordToNodeList,roadToEdgeList] = setGraph(graph,temps,soil_amount);
    // printGraph(graph);
    //各ステップで運搬に利用する道路が入口と接続しているかを確認
    for (int i = 0; i < stepNum; ++i) {
        std::cout <<std::endl;
        double built_length_i = 0;
        result.printBuiltLengthList();
        buildNewRoad(temps, i, solution, result.built_length_list, soil_amount, built_length_i);
        std::vector<NodePair> nodepairlist = {};

        //buildした道路をnetworkListに追加
        for(int j = 0; j < temps.roadNum; ++j) {
            if(solution.roadbuildStatusList[j].timings[i]==1){
                // std::cout << "i:" << i << ", j:" << j << std::endl;
                int nodeIndex1 = temps.roadStatusList[j].coordpair.coords[0].x * GRID_SIZE_Y + temps.roadStatusList[j].coordpair.coords[0].y;
                int nodeIndex2 = temps.roadStatusList[j].coordpair.coords[1].x * GRID_SIZE_Y + temps.roadStatusList[j].coordpair.coords[1].y;
                if(nodeIndex1 > nodeIndex2) {
                    std::swap(nodeIndex1, nodeIndex2);
                }
                nodepairlist.push_back({nodeIndex1, nodeIndex2});
            }
        }
        temps.printParameters();

        // std::cout << std::endl;
        // std::cout << "nodepairlist" << std::endl;
        // for(size_t n = 0;n < nodepairlist.size();n++){
        //     std::cout << "nodepair[" << n <<"]  node1:" << nodepairlist[n].node[0] << ", node2:" << nodepairlist[n].node[1] << std::endl;
        // }

        // std::cout << "before update" << std::endl;
        // networkList.printConverted();

        updateConnectNetwork(networkList, nodepairlist);
        // std::cout << "after update1" << std::endl;
        // networkList.printConverted();
        
        //運搬に使用された仮設道路が入り口と接続しているかを確認（generateSolutionTimingで利用する直前に道路を建設するようになっている）
        for(size_t j = 0; j < solution.roadNum; ++j) {
            if(solution.roadbuildStatusList[j].timings[i] == 1) {
                // std::cout << "Processing step " << i << ", road " << j << std::endl;
                int node1 = solution.roadbuildStatusList[j].coordpair.coords[0].x * GRID_SIZE_Y + solution.roadbuildStatusList[j].coordpair.coords[0].y;
                int node2 = solution.roadbuildStatusList[j].coordpair.coords[1].x * GRID_SIZE_Y + solution.roadbuildStatusList[j].coordpair.coords[1].y;
                Coord entranceCoord = {entranceXPostion,entranceYPostion};
                Coord coord1 = solution.roadbuildStatusList[j].coordpair.coords[0];
                Coord coord2 = solution.roadbuildStatusList[j].coordpair.coords[1];
                double distance1 = calculate_distance_3D(solution.roadbuildStatusList[j].coordpair.coords[0],entranceCoord,soil_amount);
                double distance2 = calculate_distance_3D(solution.roadbuildStatusList[j].coordpair.coords[1],entranceCoord,soil_amount);
                
                if (node1 > node2) {
                    std::swap(node1, node2);
                    std::swap(distance1,distance2);
                    std::swap(coord1,coord2);
                }
                Coord connectCoord = (distance1 <= distance2)? coord1 : coord2;
                //接続しているかを確認
                for(size_t k = 0; k < networkList.connectNum; ++k) {
                    if(networkList.connectNetworkList[k].edgeList[node1][node2] == 1 && networkList.connectNetworkList[k].nodeList[entranceIndex] != 1) {
                        //接続していない場合は，入り口に接続するために建設するedgeを求める
                        int startIndex = 0;
                        //startNodeをnetworkList.connectNetworkList[k].nodeListの中で，入り口に最も近いノードを探す
                        double minDistance = INF;
                        for(size_t l = 0; l < networkList.connectNetworkList[k].nodeList.size(); ++l) {
                            if(networkList.connectNetworkList[k].nodeList[l] == 0) {
                                continue;
                            }
                            int x = l / GRID_SIZE_Y;
                            int y = l % GRID_SIZE_Y;
                            int distance = calculate_distance_3D({x,y}, {entranceXPostion, entranceYPostion}, soil_amount);
                            if(distance < minDistance) {
                                minDistance = distance; 
                                startIndex = l;
                            }
                        }
                        //coordToNodeListからstartIndexに対応する座標を求める
                        Coord startCoord  = {startIndex / GRID_SIZE_Y, startIndex % GRID_SIZE_Y};
                        // std::cout << "startCoord: (" << startCoord.x << "," << startCoord.y << ")" << std::endl;
                       //startCoordのnodeをcoordToNodeListから求める
                       auto it = std::find_if(
                        coordToNodeList.begin(),
                        coordToNodeList.end(),
                        [&startCoord](const CoordToNode& entry) {
                            return (entry.coord.x == startCoord.x && entry.coord.y == startCoord.y);
                        });
                    
                    if (it == coordToNodeList.end()) {
                        std::cerr << "Error: startCoord not found in coordToNodeList." << std::endl;
                        // return new_solution; // またはエラーハンドリング
                    }
                    //find entranceNode like startNode
                    Coord entranceCoord = {entranceXPostion, entranceYPostion};
                    auto it2 = std::find_if(
                        coordToNodeList.begin(),
                        coordToNodeList.end(),
                        [&entranceCoord](const CoordToNode& entry) {
                            return (entry.coord.x == entranceCoord.x && entry.coord.y == entranceCoord.y);
                        });
                    if (it2 == coordToNodeList.end()) {
                        std::cerr << "Error: entranceCoord not found in coordToNodeList." << std::endl;
                        // return new_solution; // またはエラーハンドリング
                    }
                    int entranceNode = it2->node;
                    // std::cout << "entranceNode: " << entranceNode << std::endl;


                    int startNode = it->node;
                    // std::cout << "startNode: " << startNode << std::endl;
                    // std::cout << "entranceIndex: " << entranceIndex << std::endl;
                        updateGraph(graph,temps,roadToEdgeList,soil_amount);
                        // printGraph(graph);
                        auto path = dijkstra(graph,startNode,entranceNode);
                        // std::cout << "path: ";
                        // for(size_t m = 0; m < path.size(); ++m) {
                        //     std::cout << path[m] << " ";
                        // }
                        std::cout << std::endl;
                        //pathに含まれているNodeをnetworkに追加する
                        std::vector<NodePair> pathNodePairList;
                        for(size_t m=0;m<path.size()-1;m++){
                            for(size_t l = 0;l<roadToEdgeList.size();l++){
                                if((path[m] == roadToEdgeList[l].node1 &&path[m+1] == roadToEdgeList[l].node2)||(path[m] == roadToEdgeList[l].node2 &&path[m+1] == roadToEdgeList[l].node1)){
                            
                                    new_solution.roadbuildStatusList[l].timings[i] = 1;
                                    temps.roadStatusList[l].status[0] = 1;
                                    temps.roadStatusList[l].status[1] = 1;
                                    NodePair nodepair;
                                    int nodepair1 =  solution.roadbuildStatusList[l].coordpair.coords[0].x * GRID_SIZE_Y + solution.roadbuildStatusList[l].coordpair.coords[0].y;
                                    int nodepair2 = solution.roadbuildStatusList[l].coordpair.coords[1].x * GRID_SIZE_Y + solution.roadbuildStatusList[l].coordpair.coords[1].y;
                                    nodepair.node[0] = nodepair1;
                                    nodepair.node[1] = nodepair2;
                                    pathNodePairList.push_back(nodepair);
                                }
                            }
                        }

                        updateConnectNetwork(networkList,pathNodePairList);

                        //tempsにpathNodePairListにある座標のstatusを1にする
                        for(size_t m=0;m<pathNodePairList.size();m++){
                            for(size_t l = 0;l<temps.roadNum;l++){
                                int nodepair1 = temps.roadStatusList[l].coordpair.coords[0].x * GRID_SIZE_Y + temps.roadStatusList[l].coordpair.coords[0].y;
                                int nodepair2 = temps.roadStatusList[l].coordpair.coords[1].x * GRID_SIZE_Y + temps.roadStatusList[l].coordpair.coords[1].y;
                                if((pathNodePairList[m].node[0] == nodepair1 && pathNodePairList[m].node[1] == nodepair2)||(pathNodePairList[m].node[0] == nodepair2 && pathNodePairList[m].node[1] == nodepair1)){
                                    new_solution.roadbuildStatusList[l].timings[i] = 1;
                                    temps.roadStatusList[l].status[0] = 1;
                                    temps.roadStatusList[l].status[1] = 1;
                                }
                            }
                        }
                        // temps.printParameters();
                        // networkList.printConverted();
                        break;   
                    }
            }
        } 
        // auto [cost,path,usedTempList_i] = astar(allocations[i], temps,soil_amount,true);
        }
        removeTemporaryRoads(allocations[i], temps);
        //networkListを更新する
        int removenode1 = allocations[i].start.x * GRID_SIZE_Y + allocations[i].start.y;
        int removenode2 = allocations[i].goal.x * GRID_SIZE_Y + allocations[i].goal.y;
        updateConnectNetwork(networkList, removenode1, removenode2);
        
        // std::cout << "temps before update3" << std::endl;
        // temps.printParameters();
        // std::cout << "after update3" << std::endl;
        // networkList.printConverted();
        // std::cout << "after step" << i << std::endl;
    }
    // std::cout << "new_solution" << std::endl;
    // new_solution.printParameters();
    // std::cout << std::endl;
    return new_solution;
}

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
            result.roadTransportedVolumeByStep[i].push_back(0);
        }
        result.built_length_list.push_back(0);
    } else if (modification_type == 1) {
        // Deletion: Remove a random coordinate pair weighted by its usage and length
            std::vector<double> weights(neighbor_solution.roadbuildStatusList.size(), 0.0);
            double total_weight = 0.0;
            for (int i = 0; i < neighbor_solution.roadNum; ++i) {
                double usage = result.roadTransportedVolumeByStep[0][i];
                for (int j = 1; j < result.roadTransportedVolumeByStep.size(); ++j) {
                    usage += result.roadTransportedVolumeByStep[j][i];
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
            result.roadTransportedVolumeByStep[i].erase(result.roadTransportedVolumeByStep[i].begin() + to_remove);
            }
            result.built_length_list.erase(result.built_length_list.begin() + to_remove);
    } else if (modification_type == 2) {
    // Reassignment: Modify a random coordinate pair, weighted by usage and length
        // Calculate weights for each coordinate pair
        std::vector<double> weights(neighbor_solution.roadbuildStatusList.size(), 0.0);
        double total_weight = 0.0;
            for (int i = 0; i < neighbor_solution.roadNum; ++i) {
                double usage = result.roadTransportedVolumeByStep[0][i];
                for (int j = 1; j < result.roadTransportedVolumeByStep.size(); ++j) {
                    usage += result.roadTransportedVolumeByStep[j][i];}
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

struct RoadAndStepPair {
    int roadIndex;
    int stepIndex;
};

std::tuple<Solution,bool> generateRandomTiming(
const Solution& current_solution,
    std::vector<Allocation> allocations,
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
    const Result& result
    ) {
    Solution new_solution = current_solution;
    bool isBelowThresholdExist = false;
    std::vector<RoadAndStepPair> belowThresholdRoadsList;
    // resultのroadTransportedVolumeByStepが閾値以下のものの中で，ランダムに選びtimingを0にする
    double thereholdRate = 1.1;
    double therehold = thereholdRate * VELOCITY_PAVE*VELOCITY_ROUGH*CONSTRUCTION_TEMP*TRUCK_NUM*TRUCK_CAPACITY*1000/((VELOCITY_PAVE - VELOCITY_ROUGH)*COST_HOUR);

    std::cout << "therehold: " << therehold << std::endl;
    //print result.roadTransportedVolumeByStep
    std::cout << "result.roadTransportedVolumeByStep:" << std::endl;
    for(size_t i = 0; i < result.size; ++i) {
        for(size_t j = 0; j < new_solution.roadNum; ++j) {
            std::cout << result.roadTransportedVolumeByStep[i][j] << " ";
        }
        std::cout << std::endl;
    }

    for(size_t i = 0; i < new_solution.roadNum; ++i) {
        for(size_t j = 0; j < new_solution.stepNum; ++j) {
            double transportedVolume = result.roadTransportedVolumeByStep[j][i];
            if (transportedVolume !=0 && transportedVolume < therehold) {
                isBelowThresholdExist = true;
                RoadAndStepPair roadStepPair;
                roadStepPair.roadIndex = i;
                roadStepPair.stepIndex = j;
                belowThresholdRoadsList.push_back(roadStepPair);
            }
        }
    }
    if(!isBelowThresholdExist) {
        std::cout << "No roads below threshold." << std::endl;
        return {new_solution, isBelowThresholdExist};
    }
    // Randomly select a road and step from belowThresholdRoadsList
    int randomIndex = generateRandomInt(0, belowThresholdRoadsList.size() - 1);
    RoadAndStepPair selectedRoadStep = belowThresholdRoadsList[randomIndex];
    int selectedRoadIndex = selectedRoadStep.roadIndex;
    int selectedStepIndex = selectedRoadStep.stepIndex;
    // Set the timing of the selected road and step to 0
    new_solution.roadbuildStatusList[selectedRoadIndex].timings[selectedStepIndex] = 0;

    return {new_solution,isBelowThresholdExist};
}

std::tuple<Solution,Solution,double> generate_neighbor(
    const Solution& current_solution,
    Result& result,
    std::vector<Allocation> allocations,
    const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]
    ) {   

    double soil_amount_copy[GRID_SIZE_X][GRID_SIZE_Y];
    for(int i = 0; i < GRID_SIZE_X; ++i) {
        for (int j = 0; j < GRID_SIZE_Y; ++j) {
            soil_amount_copy[i][j] = soil_amount[i][j];
        }
    }
    //print soil_amout_copy
    std::cout << "soil_amount before generate_randomPostion" << std::endl;
    for (int i = 0; i < GRID_SIZE_X; ++i) {
        for (int j = 0; j < GRID_SIZE_Y; ++j) {
            std::cout << soil_amount[i][j] << " ";
        }
        std::cout << std::endl;
    }
    result.printPath();
    Solution neighbor_solution1 = generate_randomPostion(current_solution, result, allocations, soil_amount_copy);
    //print neighbor_solution
    // std::cout << "neighbor_solution after random postion change" << std::endl;
    // neighbor_solution1.printParameters();
    // std::cout << std::endl;

    //print soil_amount
    // std::cout << "soil_amount after generate_randomPostion" << std::endl;
    // for (int i = 0; i < GRID_SIZE_X; ++i) {
    //     for (int j = 0; j < GRID_SIZE_Y; ++j) {
    //         std::cout << soil_amount[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // // adjust position to ensure connectivity to the entrance


    double soil_amount_copy2[GRID_SIZE_X][GRID_SIZE_Y];
    for(int i = 0; i < GRID_SIZE_X; ++i) {
        for (int j = 0; j < GRID_SIZE_Y; ++j) {
            soil_amount_copy2[i][j] = soil_amount[i][j];
        }
    }
    //print soil_amount_copy2
    // std::cout << "soil_amount before adjustPostionForEntryConnectivity" << std::endl;
    // for (int i = 0; i < GRID_SIZE_X; ++i) {
    //     for (int j = 0; j < GRID_SIZE_Y; ++j) {
    //         std::cout << soil_amount[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    auto neighbor_solution2 = adjustPostionForEntryConnectivity(neighbor_solution1, allocations, soil_amount_copy2);
    //print neighbor_solution after adjustTimingForEntryConnectivity
    // std::cout << "neighbor_solution after adjustPositionForEntryConnectivity" << std::endl;
    // neighbor_solution2.printParameters();
    // // setAllTimingtoOne(neighbor_solution);
    // std::cout << std::endl;
    
    double soil_amount_copy3[GRID_SIZE_X][GRID_SIZE_Y];
    for(int i = 0; i < GRID_SIZE_X; ++i) {
        for (int j = 0; j < GRID_SIZE_Y; ++j) {
            soil_amount_copy3[i][j] = soil_amount[i][j];
        }
    }
    //print soil_amount_copy3
    // std::cout << "soil_amount before generateSolutionTiming" << std::endl;
    // for (int i = 0; i < GRID_SIZE_X; ++i) {
    //     for (int j = 0; j < GRID_SIZE_Y; ++j) {
    //         std::cout << soil_amount[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    auto neighbor_solution3 = generateSolutionTiming(neighbor_solution2, allocations, soil_amount_copy3);
    // //print neighbor_solution after generateSolutionTiming
    // std::cout << "neighbor_solution after generateSolutionTiming" << std::endl;
    // neighbor_solution3.printParameters();
    // std::cout << std::endl;

    // //print soil_amount
    // std::cout << "soil_amount after generateSolutionTiming" << std::endl;
    // for (int i = 0; i < GRID_SIZE_X; ++i) {
    //     for (int j = 0; j < GRID_SIZE_Y; ++j) {
    //         std::cout << soil_amount[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;
    //print solution roadNum
    // std::cout << "solution roadNum: " << neighbor_solution3.roadNum << std::endl;
    result.init(neighbor_solution3);

    double soil_amount_copy4[GRID_SIZE_X][GRID_SIZE_Y];
    for(int i = 0; i < GRID_SIZE_X; ++i) {
        for (int j = 0; j < GRID_SIZE_Y; ++j) {
            soil_amount_copy4[i][j] = soil_amount[i][j];
        }
    }
    //adjust timings to ensure connectivity to the entrance
    //print soil_amount_copy4
    // std::cout << "soil_amount before adjustTimingForEntryConnectivity" << std::endl;
    // for (int i = 0; i < GRID_SIZE_X; ++i) {
    //     for (int j = 0; j < GRID_SIZE_Y; ++j) {
    //         std::cout << soil_amount[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
    // //print neighbor_solution3
    // std::cout << "neighbor_solution before adjustTimingForEntryConnectivity" << std::endl;
    // neighbor_solution3.printParameters();
    // // std::cout << std::endl;
    // //print result
    // // std::cout << "result before adjustTimingForEntryConnectivity" << std::endl;
    // result.printBuiltLengthList();
    // // result.printUsedRoadFlow();
    // std::cout << "result roadTransportedVolumeByStep size: " << result.roadTransportedVolumeByStep.size() << std::endl;
    // std::cout << "result roadTransportedVolumeByStep size: " << result.roadTransportedVolumeByStep[0].size() << std::endl;
    auto neighbor_solution4 = adjustTimingForEntryConnectivity(neighbor_solution3, result, allocations, soil_amount_copy4);
    //print neighbor_solution after adjustTimingForEntryConnectivity
    // std::cout << "neighbor_solution after adjustTimingForEntryConnectivity" << std::endl;
    // neighbor_solution4.printParameters();
    // std::cout << std::endl;


    double soil_amount_copy5[GRID_SIZE_X][GRID_SIZE_Y];
    for(int i = 0; i < GRID_SIZE_X; ++i) {
        for (int j = 0; j < GRID_SIZE_Y; ++j) {
            soil_amount_copy5[i][j] = soil_amount[i][j];
        }
    }
    // Evaluate the new solution
    auto [new_result,total_cost] = evaluate_design(allocations,soil_amount_copy5, neighbor_solution4);
    return {neighbor_solution1,neighbor_solution4,total_cost};
}

void simulated_annealing(std::vector<Allocation>& allocations, const Solution& initSolution, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    Solution current_solution = initSolution;
    Solution current_modified_solution = initSolution;
    //setAllTimingtoOne(current_solution);
    double soil_amount_copy[GRID_SIZE_X][GRID_SIZE_Y];
        for (int i = 0; i < GRID_SIZE_X; i++) {
            for (int j = 0; j < GRID_SIZE_Y; j++) {
                soil_amount_copy[i][j] = soil_amount[i][j];
            }
        }
    auto [current_result,current_score] = evaluate_design(allocations,soil_amount_copy, current_solution);

    Solution best_solution = current_solution;
    Solution best_modified_solution = current_modified_solution;
    double best_score = current_score;
    Result best_result = current_result;
    int best_score_loop = 0;

    // std::vector<std::pair<double, Solution>> best_score_flow;
    // std::vector<std::pair<double, Solution>> current_score_flow;
    // std::vector<std::pair<double, Solution>> neighbor_score_flow;
    // std::vector<std::pair<double, Solution>> neighbor_solution_flow;

    int temperature = initialTemperature;
    for (int iter = 0; iter < max_iter; ++iter) {
        std::cout << "Iteration: " << iter << std::endl;
        //reset soil_amount copy
        double soil_amount_copy[GRID_SIZE_X][GRID_SIZE_Y];
        for (int i = 0; i < GRID_SIZE_X; i++) {
            for (int j = 0; j < GRID_SIZE_Y; j++) {
                soil_amount_copy[i][j] = soil_amount[i][j];
            }
        }
        //print soil_amount_copy
        std::cout << "soil_amount before generate_neighbor" << std::endl;
        for (int i = 0; i < GRID_SIZE_X; ++i) {
            for (int j = 0; j < GRID_SIZE_Y; ++j) {
                std::cout << soil_amount_copy[i][j] << " ";
            }
            std::cout << std::endl;
        }
        auto neighbor_result = current_result;
        temperature *= alpha;

        //print current solution and result
        auto [neighbor_solution,neighbor_modified_solution,neighbor_score] = generate_neighbor(current_solution,neighbor_result, allocations, soil_amount_copy);
        std::cout << "generate_neighbor called" << std::endl;
        std::cout << "neighbor_solution after generate_neighbor" << std::endl;
        neighbor_solution.printParametersModified();
        std::cout << std::endl;
        std::cout << "neighbor_modified_solution after generate_neighbor" << std::endl;
        neighbor_modified_solution.printParametersModified();
        std::cout << std::endl;
        double random_value = generateRandomDouble(0.0, 1.0);
        // 受け入れ判定
        if ((neighbor_score < current_score) || 
            (random_value < std::exp(-(std::abs(neighbor_score - current_score)) / temperature))) {
            current_solution = neighbor_solution;
            current_score = neighbor_score;
            current_result = neighbor_result;
            current_modified_solution = neighbor_modified_solution;
        }

        // ベスト解の更新
        if (current_score < best_score) {
            best_solution = current_solution;
            best_score = current_score;
            best_score_loop = iter;
            best_result = current_result;
            best_modified_solution = current_modified_solution;
        }
        std::cout << "iteration done" << std::endl;
    }

    //print best solution and result
    std::cout << "best_score: " << best_score << std::endl;
    // best_solution.printParameters();
    best_modified_solution.printParametersForPlot();
    best_result.printPathForPlot(allocations);
    std::cout << "best_score_loop: " << best_score_loop << std::endl;
}


void simulatedAnnealingforTimingOptimization(std::vector<Allocation>& allocations, const Solution& initSolution, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]) {
    Solution current_solution = initSolution;
    Solution current_modified_solution = initSolution;
    //setAllTimingtoOne(current_solution);
    double soil_amount_copy[GRID_SIZE_X][GRID_SIZE_Y];
        for (int i = 0; i < GRID_SIZE_X; i++) {
            for (int j = 0; j < GRID_SIZE_Y; j++) {
                soil_amount_copy[i][j] = soil_amount[i][j];
            }
        }
    auto [current_result,current_score] = evaluate_design(allocations,soil_amount_copy, current_solution);
    std::cout << "initial score: " << current_score << std::endl;
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
        std::cout << "Iteration: " << iter << std::endl;
        //reset soil_amount copy
        double soil_amount_copy[GRID_SIZE_X][GRID_SIZE_Y];
        for (int i = 0; i < GRID_SIZE_X; i++) {
            for (int j = 0; j < GRID_SIZE_Y; j++) {
                soil_amount_copy[i][j] = soil_amount[i][j];
            }
        }
        //print soil_amount_copy
        std::cout << "soil_amount before generate_neighbor" << std::endl;
        for (int i = 0; i < GRID_SIZE_X; ++i) {
            for (int j = 0; j < GRID_SIZE_Y; ++j) {
                std::cout << soil_amount_copy[i][j] << " ";
            }
            std::cout << std::endl;
        }
        auto neighbor_result = current_result;
        temperature *= alpha;
        std::cout << std::endl;
        std::cout << "current_solution before generate_neighbor" << std::endl;
        current_solution.printParameters();
        //print current solution and result
        auto [neighbor_solution,isBelowThresholdExist] = generateRandomTiming(current_solution, allocations, soil_amount_copy, current_result);
        std::cout << "neighbor_solution after generate_neighbor" << std::endl;
        neighbor_solution.printParameters();
        std::cout << std::endl;
        if(!isBelowThresholdExist) {
            std::cout << "No roads below threshold." << std::endl;
            break; // Stop the iteration if no roads below threshold
        }
        double random_value = generateRandomDouble(0.0, 1.0);

        auto [temp_result, neighbor_score] = evaluate_design(allocations, soil_amount_copy, neighbor_solution);
        std::cout << "neighbor_score after evaluate_design: " << neighbor_score << std::endl;
        neighbor_result = temp_result;
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
        std::cout << "iteration done" << std::endl;
    }

    //print best solution and result
    std::cout << "best_score: " << best_score << std::endl;
    // best_solution.printParameters();
    best_result.printPathForPlot(allocations);
    std::cout << "best_score_loop: " << best_score_loop << std::endl;
}

int main() {
    auto start = std::chrono::high_resolution_clock::now();
    
    //Initialize soil_amount
    // double soil_amount[GRID_SIZE_X][GRID_SIZE_Y] =
    // {
    //     {-15000.0, -62600.0, -3700.0, 0.0},
    //     {-3700.0, 22500.0, 22500.0, -1400.0},
    //     {3700.0, 33800.0, 28100.0, 2300.0},
    //     {9000.0, 36000.0, 23000.0, 1200.0},
    //     {9000.0, 23000.0, 24300.0, 9000.0},
    //     {8000.0, 22200.0, 14200.0, -5900.0},
    //     {-1000.0, 8100.0, -12400.0, -9900.0},
    //     {-11200.0, -24800.0, -34400.0, -2700.0},
    //     {-2300.0, -9900.0, -72500.0, 2300.0},
    //     {-22000.0, -2200.0, -28500.0, -100.0},
    //     {-6900.0, 14300.0, -2500.0, 0.0},
    //     {11200.0, 7900.0, 0.0, 0.0}
    // };
    double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]  =
    {
        {1,0,0,0},
        {0,-1,0,0},
        {0,0,1,0},
        {0,0,0,-1}
    };
    //check if sum of soil_amount is zero
    checkSoilAmountTotal(soil_amount);
    // Initialize allocations

     std::vector<Allocation> allocations = 
    {
        {{0,0}, {1,1},1},
        {{2,2}, {3,3},1}
    };


    // std::vector<Allocation> allocations = 
    // {
    //     {{11, 1}, {10, 2}, 2500.0},
    //     {{11, 1}, {9, 2}, 5400.0},
    //     {{11, 0}, {10, 0}, 6900.0},
    //     {{11, 0}, {9, 0}, 4300.0},
    //     {{10, 1}, {9, 2}, 12100.0},
    //     {{10, 1}, {9, 1}, 2200.0},
    //     {{8, 3}, {9, 3}, 100.0},
    //     {{8, 3}, {9, 2}, 2200.0},
    //     {{3, 1}, {9, 2}, 8800.0},
    //     {{5, 0}, {9, 0}, 8000.0},
    //     {{4, 0}, {9, 0}, 4000.0},
    //     {{3, 0}, {9, 0}, 5700.0},
    //     {{5, 2}, {8, 2}, 14200.0},
    //     {{4, 2}, {8, 2}, 24300.0},
    //     {{4, 1}, {8, 2}, 17400.0},
    //     {{3, 1}, {8, 2}, 16600.0},
    //     {{6, 1}, {8, 1}, 8100.0},
    //     {{3, 1}, {8, 1}, 1800.0},
    //     {{3, 0}, {8, 0}, 2300.0},
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

    // Initialize Solution
    int stepNum = allocations.size();

    CoordPair coordpair1 = {{{0, 0},{1, 1}}};
    CoordPair coordpair2 = {{{1, 1},{1, 2}}};
    CoordPair coordpair3 = {{{1, 2},{2, 2}}};
    CoordPair coordpair4 = {{{2, 2},{3, 3}}};

    normalize_pair(coordpair1);
    normalize_pair(coordpair2);
    normalize_pair(coordpair3);
    normalize_pair(coordpair4);

    std::vector<CoordPair> coordpairlist;
    coordpairlist.push_back(coordpair1);
    coordpairlist.push_back(coordpair2);
    coordpairlist.push_back(coordpair3);
    coordpairlist.push_back(coordpair4);

//     CoordPair coordpair1 = {{{0, 2},{1, 2}}};
//     CoordPair coordpair2 = {{{1, 2},{2, 2}}};
//     CoordPair coordpair3 = {{{2, 2},{3, 2}}};
//     CoordPair coordpair4 = {{{3, 2},{4, 2}}};
//     CoordPair coordpair5 = {{{4, 2},{5, 2}}};
//     CoordPair coordpair6 = {{{5, 2},{6, 2}}};
//     CoordPair coordpair7 = {{{6, 2},{7, 2}}};
//     CoordPair coordpair8 = {{{2, 1},{3, 1}}};
//     CoordPair coordpair9 = {{{3, 1},{4, 1}}};
//     CoordPair coordpair10 = {{{4, 1},{5, 1}}};
//     CoordPair coordpair11 = {{{5, 1},{6, 1}}};
//     CoordPair coordpair12 = {{{6, 1},{7, 1}}};
//     CoordPair coordpair13 = {{{7, 1},{8, 1}}};
//     CoordPair coordpair14 = {{{8, 1},{9, 1}}};
//     CoordPair coordpair15 = {{{5, 2},{6, 1}}};


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

    Solution initSolution = initializedSolution(stepNum,coordpairlist);
    TemporaryRoads temps;
    temps.initialize(initSolution);

    initSolution.printParameters();
    
    setAllTimingtoOne(initSolution);
    double soil_amount_copy[GRID_SIZE_X][GRID_SIZE_Y];
        for (int i = 0; i < GRID_SIZE_X; i++) {
            for (int j = 0; j < GRID_SIZE_Y; j++) {
                soil_amount_copy[i][j] = soil_amount[i][j];
            }
        }

    initSolution = generateSolutionTiming(initSolution, allocations, soil_amount_copy);
    std::cout << "initSolution after generateSolutionTiming: " << std::endl;
    initSolution.printParameters();

    auto [result,operate_cost,built_length] = processAllocations(allocations, initSolution, soil_amount_copy);

    //test simulatedAnnealingforTimingOptimization
    simulatedAnnealingforTimingOptimization(allocations, initSolution, soil_amount_copy);
    return 0;
}
