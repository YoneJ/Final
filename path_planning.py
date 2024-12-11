import numpy as np
import heapq
from pose_manager import PoseManager

def astar(grid_map, start, goal):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    open_set = []
    heapq.heappush(open_set, (0, start))
    g_costs = {start: 0}
    f_costs = {start: heuristic(start, goal)}
    came_from = {}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])

            if not (0 <= neighbor[0] < grid_map.shape[1] and 0 <= neighbor[1] < grid_map.shape[0]):
                continue
            if grid_map[neighbor[1], neighbor[0]] == 1:
                continue

            tentative_g_cost = g_costs[current] + (1.414 if direction[0] != 0 and direction[1] != 0 else 1)

            if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                came_from[neighbor] = current
                g_costs[neighbor] = tentative_g_cost
                f_costs[neighbor] = tentative_g_cost + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_costs[neighbor], neighbor))

    return None

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

# Reconstruct path
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def to_grid_indices(x, y, x_min, y_min, resolution):
    grid_x = int((x - x_min) / resolution)
    grid_y = int((y - y_min) / resolution)
    return grid_x, grid_y

def plan_path(grid_map_file, x_min, y_min, resolution, current_pose, goal_coords):
    grid_map = np.load(grid_map_file)

    start = to_grid_indices(current_pose[0], current_pose[1], x_min, y_min, resolution)
    goal = to_grid_indices(goal_coords[0], goal_coords[1], x_min, y_min, resolution)

    path = astar(grid_map, start, goal)

    if path:
        return path
    else:
        print("No path found from start to goal.")
        return None

# Example usage
grid_map_file = 'map.npy'  # Replace with your .npy file path
x_min = 0  # Minimum x-coordinate in real-world units
y_min = 0  # Minimum y-coordinate in real-world units
resolution = 0.03  # Resolution in real-world units per grid cell

# Replace these with your actual start and goal coordinates in real-world units
current_pose = PoseManager().get_pose()

goal_coords = (3.0, 3.0)   # Example goal position (x, y)

# Run the path planning function
path = plan_path(grid_map_file, x_min, y_min, resolution, current_pose, goal_coords)

if path:
    print("Path found:", path)
else:
    print("No path found.")
