#include <iostream>
#include <cstdlib> 
#include <vector>
#include <ctime> // time関数を使用するために必要

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
    std::vector<std::vector<int>> used_road_flow;  //used_temp_list[step][road]
    std::vector<double> built_length_list;
};
int main(){
Solution neighbor_solution;
Result result;
int to_remove = 1;
//neighbor_solutionを初期化
neighbor_solution.count = 3;
neighbor_solution.step = 2;
CoordPair coordpair[3] = {{{{0, 0}, {1, 1}}}, {{{2, 2}, {3, 3}}}, {{{1, 0}, {0, 1}}}};
for (int i = 0; i < neighbor_solution.count; i++) {
neighbor_solution.coordpair.push_back(coordpair[i]);
}
neighbor_solution.timings = {{1, 0}, {1, 0}, {1, 0}};
//resultを初期化
result.size = neighbor_solution.step;
result.path.resize(neighbor_solution.step);
result.used_road_flow.resize(neighbor_solution.step);
for (int i = 0; i < neighbor_solution.step; i++) {
result.used_road_flow[i].resize(neighbor_solution.count);
    for (int j = 0; j < neighbor_solution.count; j++) {
    result.used_road_flow[i][j] = 0;
    }
}
//print neighbor solution
std::cout << "neighbor_solution before" << std::endl;
for (int i = 0; i < neighbor_solution.count; ++i) {
std::cout << "neighbor_solution.coordpair[" << i << "]: (" << neighbor_solution.coordpair[i].coords[0].x << ", "
<< neighbor_solution.coordpair[i].coords[0].y << "), (" << neighbor_solution.coordpair[i].coords[1].x << ", "
<< neighbor_solution.coordpair[i].coords[1].y << ")\n";
for (int j = 0; j < neighbor_solution.step; ++j) {
std::cout << "  timing[" << j << "]: " << neighbor_solution.timings[i][j] << "\n";
}
}
std::cout << neighbor_solution.count << std::endl;
std::cout << neighbor_solution.step << std::endl;
std::cout << std::endl;
//print result
std::cout << "result before" << std::endl;
std::cout << "result.size: " << result.size << std::endl;
for (int i = 0; i < result.size; i++) {
std::cout << "result.path[" << i << "]: " << result.path[i].size() << std::endl;
for (int j = 0; j < result.used_road_flow[i].size(); j++) {
std::cout << "  result.used_road_flow[" << i << "][" << j << "]: " << result.used_road_flow[i][j] << std::endl;
}
}
std::cout << "result.built_length_list" << std::endl;
for(int i = 0; i < result.built_length_list.size(); i++){
std::cout << "result.built_length_list[" << i << "]: " << result.built_length_list[i] << std::endl;
}

std::cout << std::endl;

neighbor_solution.coordpair.erase(neighbor_solution.coordpair.begin() + to_remove);
neighbor_solution.timings.erase(neighbor_solution.timings.begin() + to_remove);  
neighbor_solution.count--;  
// result.used_road_flow.erase(result.used_road_flow.begin() + to_remove);
// result.built_length_list.erase(result.built_length_list.begin() + to_remove);
// result.size--;
for (size_t i = 0;i<result.size;i++){
result.used_road_flow[i].erase(result.used_road_flow[i].begin() + to_remove);
}
result.built_length_list.erase(result.built_length_list.begin() + to_remove);

//print neighbor solution
std::cout << "neighbor_solution after" << std::endl;
for (int i = 0; i < neighbor_solution.count; ++i) {
std::cout << "neighbor_solution.coordpair[" << i << "]: (" << neighbor_solution.coordpair[i].coords[0].x << ", "
<< neighbor_solution.coordpair[i].coords[0].y << "), (" << neighbor_solution.coordpair[i].coords[1].x << ", "
<< neighbor_solution.coordpair[i].coords[1].y << ")\n";
for (int j = 0; j < neighbor_solution.step; ++j) {
std::cout << "  timing[" << j << "]: " << neighbor_solution.timings[i][j] << "\n";
}}
//print result
std::cout << "result after" << std::endl;
std::cout << "result.size: " << result.size << std::endl;
for (int i = 0; i < result.size; i++) {
std::cout << "result.path[" << i << "]: " << result.path[i].size() << std::endl;
for (int j = 0; j < result.used_road_flow[i].size(); j++) {
std::cout << "  result.used_road_flow[" << i << "][" << j << "]: " << result.used_road_flow[i][j] << std::endl;
}
}
}

