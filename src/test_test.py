import copy
import numpy as np
DIRECTIONS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
grid_size_x = 4
grid_size_y = 4
cut_indices = [[(1, 0),1],[(1, 2),1],[(2, 0),1],[(2, 1),1],[(2, 2),1],[(2, 3),1],[(3, 0),1],[(3, 1),1],[(3, 3),2]]
#盛土の座標と土量
fill_indices = [[(0, 0),2],[(0, 1),1],[(0, 2),1],[(0, 3),1],[(1, 3),2],[(1, 1),1],[(3, 2),2]]

soil_amount = np.zeros((grid_size_x, grid_size_y))
for [(i, j),k] in cut_indices:
    soil_amount[int(i), int(j)] = k
for [(i, j),k] in fill_indices:
    soil_amount[int(i),int(j)] = -k

def temp_in_step(current_temps, solution, step, soil_amount):
    """
    Update road construction state for the given step based on the solution.

    Args:
        current_temps (list): Current state of temporary roads.
        solution (list): Construction schedule for roads.
        step (int): Current step of the process.
        soil_amount (float): Soil amount for calculation.

    Returns:
        tuple: Updated temporary roads and the total built length.
    """
    # Create a copy of current_temps to avoid modifying the input directly
    current_temps_copy = copy.deepcopy(current_temps)
    built_length = 0

    # Helper function to calculate road segment length
    def calculate_segment_length(coord1, coord2):
        dx, dy = coord2[0] - coord1[0], coord2[1] - coord1[1]
        return ((dx**2 + dy**2)**0.5) / 2
    print("solution",solution)
    # Generate built road sets from the solution for the current step
    built_road_set = []
    for temp in solution:
        built_road_set_in_temp = {}
        for coord_list in temp:
            coord_pair, coord_built_step_list = coord_list
            if coord_built_step_list[step] == 1:
                coord1, coord2 = coord_pair
                coord1_dir = DIRECTIONS.index((coord2[0] - coord1[0], coord2[1] - coord1[1]))
                coord2_dir = DIRECTIONS.index((coord1[0] - coord2[0], coord1[1] - coord2[1]))

                # Add directions to the set
                if coord1 not in built_road_set_in_temp:
                    built_road_set_in_temp[coord1] = set()
                if coord2 not in built_road_set_in_temp:
                    built_road_set_in_temp[coord2] = set()

                built_road_set_in_temp[coord1].add(coord1_dir)
                built_road_set_in_temp[coord2].add(coord2_dir)

        built_road_set.append(built_road_set_in_temp)

    # Update the current_temps_copy based on built_road_set
    for i, built_road in enumerate(built_road_set):
        current_temp = current_temps[i]
        if current_temp is None:
            current_temps_copy[i] = built_road
            built_length += sum(
                calculate_segment_length(coord1, coord2)
                for coord1, directions in built_road.items()
                for coord2 in [coord1 + DIRECTIONS[dir] for dir in directions]
            )
        else:
            for coord, new_directions in built_road.items():
                if coord not in current_temp:
                    current_temp[coord] = set()
                for direction in new_directions:
                    if direction not in current_temp[coord]:
                        built_length += calculate_segment_length(coord, coord + DIRECTIONS[direction])
                        print("direction",direction)
                        print("current_temp",current_temp)
                        print("current_temp[coord]",current_temp[coord])
                        print("current_temp_copy",current_temps_copy)
                        current_temps_copy[i][coord].add(direction)

    return current_temps_copy, built_length


temporary_roads = [
    {(1, 2): {6, 4, 7}, (2, 2): {1}, (1, 3): {3}, (2, 3): {0}},
    {(1, 0): {7}, (2, 1): {0}},
]
allocation = [[(1, 2), (2, 3)], [(1, 0), (2, 1)]]
total_step = len(allocation)
temporary_roads_pairs =[[[(1, 2), (2, 3)], [(1, 2), (1, 3)], [(1, 2), (2, 2)]], [[(1, 0), (2, 1)]]]
# temporary_roads = convert_to_directions(temporary_roads_pairs, DIRECTIONS)
solution = [
            [[[(1, 2), (2, 3)], [1, 0]], [[(1, 2), (1, 3)], [0, 1]], [[(1, 2), (2, 2)], [0, 1]]], 
            [[[(1, 0), (2, 1)], [1, 1]]]
            ]
print("solution",solution)

current_temp = [None] * len(temporary_roads)
current_temp[1] = {(1, 0): {7}}
print("current_temp",current_temp)
total_built_length = 0
for i in range(2):
    current_temp,built_length_i = temp_in_step(current_temp,solution,i,soil_amount)
    total_built_length += built_length_i
    print("current_temp",current_temp)
    print("built_length",total_built_length)