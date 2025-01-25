#ifndef TEMPORARY_ROADS_H
#define TEMPORARY_ROADS_H

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

struct Path {
    std::vector<Coord> coord;

    void resize(size_t new_size) {
        coord.resize(new_size);
    }
    size_t size() const {
        return coord.size();
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

double calculate_distance_3D(const Coord& a, const Coord& b, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]);
void remove_temporary_roads(const Allocation& allocation, TemporaryRoads& temps);
double heuristic(const Coord& a, const Coord& b, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]);
void change_soil(const Allocation& allocation, double soil_amount[GRID_SIZE_X][GRID_SIZE_Y]);
void temp_in_step(
    TemporaryRoads& current_temps,
    const int step,
    const Solution& solution,
    std::vector<double>& built_length_list,
    const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
    double& built_length
);
CoordPair normalize_pair(const Coord& a, const Coord& b);
double get_cost(const Coord& current, const Coord& neighbor, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], const TemporaryRoads& temporary_roads);
void astar(const Allocation& allocation, const TemporaryRoads& temps, const double soil_amount[GRID_SIZE_X][GRID_SIZE_Y], std::vector<int>& used_temp_list, double& total_cost, Path& path);
void process_allocations(std::vector<Allocation>& allocations,
                        const Solution& solution,
                        double soil_amount[GRID_SIZE_X][GRID_SIZE_Y],
                        std::vector<double>& built_length_list,
                        std::vector<std::vector<int>>& used_temp_list,
                        std::vector<Path>& path,
                        double& total_cost, double& built_length);
#endif // TEMPORARY_ROADS_H
