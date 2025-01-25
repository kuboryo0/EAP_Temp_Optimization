#include "header.h"



int main() {
    std::cout << "test" << std::endl;
    int count = 3;
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
        {{0, 0}, {3, 3}, 5.0},
        {{0, 0}, {3, 3}, 1.0},
        {{2, 2}, {0, 1}, 3.0},
        {{1, 1}, {3, 0}, 2.0}};
    CoordPair coordpair[count] = {{{{0, 0}, {1, 1}}}, {{{2, 2}, {3, 3}}}, {{{1, 0}, {0, 1}}}};
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
    current_temps.statuslist.resize(3, {{0, 1}});
    // Initialize solution
    Solution solution;
    solution.count = count;
    solution.step = 3;
    for (int i = 0; i < count; i++) {
        solution.coordpair.push_back(coordpair[i]);
    }
    solution.timings = {{1, 0, 1}, {1, 1, 1}, {0, 1, 0}};

    // Initialize path and used_temp_list
    std::vector<Path> path(solution.step, Path{std::vector<Coord>()});
    std::vector<std::vector<int>> used_temp_list(solution.step, std::vector<int>(solution.count, 0));

    double total_cost = 0;
    std::vector<double> built_length_list(solution.count, 0.0);

    
    double built_length = 0;
    process_allocations(allocations, solution, soil_amount, built_length_list, used_temp_list, path, total_cost, built_length);
    // std::cout << "total_cost: " << total_cost << std::endl;

    return 0;
}
