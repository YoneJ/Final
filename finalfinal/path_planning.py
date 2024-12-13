import numpy as np
import heapq
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)

        # Parameters for map and resolution
        self.grid_map = np.load('mapfinalfinal.npy')  # Replace with your actual map file
        self.x_min = -2.0  # Minimum x-coordinate in real-world units
        self.y_min = -0.5  # Minimum y-coordinate in real-world units
        self.resolution = 0.03  # Resolution in real-world units per grid cell

        self.start_coords = (0.0, 0.0)  # Example start position (x, y)
        self.goal_coords = (1.2, 0.5)  # Example goal position (x, y)

        self.get_logger().info("Path Planning Node Initialized")
        self.plan_path_once()

    def to_grid_indices(self, x, y):
        grid_x = int((x - self.x_min) / self.resolution)
        grid_y = int((y - self.y_min) / self.resolution)
        return grid_x, grid_y

    def plan_path_once(self):
        start = self.to_grid_indices(self.start_coords[0], self.start_coords[1])
        goal = self.to_grid_indices(self.goal_coords[0], self.goal_coords[1])

        self.get_logger().info(f"Start point: {self.start_coords} -> {start}")
        self.get_logger().info(f"Goal point: {self.goal_coords} -> {goal}")

        path = self.astar(self.grid_map, start, goal)

        if path:
            self.get_logger().info(f"Path found: {path}")
            self.publish_path(path)
        else:
            self.get_logger().error("No path found!")

        # Shutdown the node after the task is done
        rclpy.shutdown()

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        path_data = []

        for (grid_x, grid_y) in path:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = grid_x * self.resolution + self.x_min
            pose_stamped.pose.position.y = grid_y * self.resolution + self.y_min
            path_msg.poses.append(pose_stamped)

            path_data.append([grid_x, grid_y])

        self.path_publisher.publish(path_msg)

        np.save('pathasta.npy', np.array(path_data))  # Save path to file
        self.get_logger().info("Path saved to 'pathasta.npy'")

    def astar(self, grid_map, start, goal):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        open_set = []
        heapq.heappush(open_set, (0, start))
        g_costs = {start: 0}
        f_costs = {start: self.heuristic(start, goal)}
        came_from = {}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

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
                    f_costs[neighbor] = tentative_g_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_costs[neighbor], neighbor))

        return None

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()


if __name__ == '__main__':
    main()
