import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from sklearn.neighbors import NearestNeighbors
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped
from tf_transformations import quaternion_from_euler
import csv

class LidarICPNode(Node):
    def __init__(self):
        super().__init__('lidar_icp_node')

        # Subscriber to receive Lidar data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Publisher to publish the pose as PoseStamped
        self.pose_publisher = self.create_publisher(PoseStamped, '/robot_pose', 10)

        # Timer to periodically publish the pose
        self.timer = self.create_timer(0.1, self.publish_pose)  # 10 Hz (0.1 seconds interval)

        # Pose tracking
        self.prev_scan = None  # Store the previous scan
        self.pose = np.array([0.0, 0.0, 1.57])  # Initial pose: [x, y, theta]

        # CSV file to store poses
        self.csv_file = open('robot_pose.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y', 'theta'])  # Write CSV header
        self.get_logger().info('Lidar ICP Node has started.')

        # Publisher for the map
        self.map_publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        
        self.path_publisher_ = self.create_publisher(Path, '/path', 10)
        # Timer to periodically publish the map (every 1 second)
        self.timer = self.create_timer(1.0, self.publish)

        # Load the map grid from the .npy file
        self.grid = np.load('merged_grid.npy')
        self.path_points = np.load('pathAsta.npy')


    def lidar_callback(self, msg):
        """
        This callback processes the incoming lidar scan data.
        """
        # Convert LaserScan data to Cartesian coordinates
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)
        points = np.array([
            ranges * np.cos(angles),
            ranges * np.sin(angles)
        ]).T

        # Remove invalid points (e.g., range = 0)
        points = points[np.linalg.norm(points, axis=1) > 0]

        if self.prev_scan is not None:
            # Apply ICP to calculate the relative transformation
            delta_pose = self.icp(self.prev_scan, points)
            self.pose = self.update_pose(self.pose, delta_pose)
            self.get_logger().info(f'Updated Pose: {self.pose}')
            # Save the pose to the CSV file
            self.csv_writer.writerow(self.pose)

        self.prev_scan = points

    def icp(self, source, target, max_iterations=200, tolerance=1e-15):
        """
        Perform ICP between source and target point clouds.
        Args:
            source: np.ndarray of shape (N, 2) - previous scan
            target: np.ndarray of shape (M, 2) - current scan
            max_iterations: Maximum number of ICP iterations
            tolerance: Convergence tolerance
        Returns:
            np.ndarray: Estimated transformation [dx, dy, dtheta]
        """
        source_copy = source.copy()
        transformation = np.eye(3)

        for i in range(max_iterations):
            # Find the nearest neighbors between the source and target
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(target)
            distances, indices = nbrs.kneighbors(source_copy)

            # Compute the centroids of the matched points
            target_matched = target[indices[:, 0]]
            source_centroid = np.mean(source_copy, axis=0)
            target_centroid = np.mean(target_matched, axis=0)

            # Subtract centroids to align the points
            source_centered = source_copy - source_centroid
            target_centered = target_matched - target_centroid

            # Compute the optimal rotation using SVD
            H = np.dot(source_centered.T, target_centered)
            U, _, Vt = np.linalg.svd(H)
            R_opt = np.dot(Vt.T, U.T)

            # Ensure R_opt is a proper rotation matrix
            if np.linalg.det(R_opt) < 0:
                Vt[1, :] *= -1
                R_opt = np.dot(Vt.T, U.T)

            # Compute the translation
            t_opt = target_centroid - np.dot(source_centroid, R_opt)

            # Update the transformation matrix
            current_transform = np.eye(3)
            current_transform[:2, :2] = R_opt
            current_transform[:2, 2] = t_opt
            transformation = np.dot(current_transform, transformation)

            # Apply the transformation to the source points
            source_copy = (np.dot(R_opt, source_copy.T).T + t_opt)

            # Check for convergence
            mean_error = np.mean(distances)
            if mean_error < tolerance:
                break

        # Extract translation and rotation (angle) from the final transformation
        dx = transformation[0, 2]
        dy = transformation[1, 2]
        dtheta = np.arctan2(transformation[1, 0], transformation[0, 0])

        return np.array([-dx, -dy, -dtheta])

    def update_pose(self, pose, delta_pose):
        """
        Update the robot's pose based on the delta pose.
        Args:
            pose: Current pose [x, y, theta]
            delta_pose: Change in pose [dx, dy, dtheta]
        Returns:
            np.ndarray: Updated pose [x, y, theta]
        """
        dx, dy, dtheta = delta_pose
        theta = pose[2]

        # Update the pose with respect to the robot's current orientation
        x_new = pose[0] + dx * np.cos(theta) - dy * np.sin(theta)
        y_new = pose[1] + dx * np.sin(theta) + dy * np.cos(theta)
        theta_new = pose[2] + dtheta

        return np.array([x_new, y_new, theta_new])

    def publish_pose(self):
        """
        Publish the current robot pose as a PoseStamped message.
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # Use 'map' or 'odom' depending on your setup
        resolution = 0.4
        # Fill in the pose message with the robot's pose
        pose_msg.pose.position.x = self.pose[0]*resolution
        pose_msg.pose.position.y = self.pose[1] * resolution
        pose_msg.pose.position.z = 0.0  # Assuming the robot is on the ground (z=0)

        # Convert the orientation (theta) into a quaternion for the PoseStamped message
        quaternion = R.from_euler('z', self.pose[2]).as_quat()
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        # Publish the pose message
        self.pose_publisher.publish(pose_msg)

    def destroy_node(self):
        # Close the CSV file when shutting down
        self.csv_file.close()
        super().destroy_node()

    def create_occupancy_grid(self, grid, resolution=0.02, origin=(0.0, 0.0)): 
    
        map_msg = OccupancyGrid()

    # Rescale the grid values to fit the expected range for OccupancyGrid
    # 0.0 -> 0 (free space), 1.0 -> 100 (occupied space)
        grid_rescaled = np.clip(grid * 100, 0, 100).astype(int)

    # Set the header information
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'

    # Set the map metadata
        map_msg.info.resolution = resolution  # More detailed resolution (smaller value)
        map_msg.info.width = grid.shape[1]
        map_msg.info.height = grid.shape[0]

    # Set the origin (position and orientation) using Pose
        map_msg.info.origin = Pose()
        map_msg.info.origin.position.x = origin[0]
        map_msg.info.origin.position.y = origin[1]
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0  # No rotation

    # Flatten the 2D grid into a 1D array and set the data
        map_msg.data = grid_rescaled.flatten().tolist()

        return map_msg

    def create_path_msg(self):
        path_msg = Path()

    # Set the header information for the path
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # Use the 'map' frame (adjust if needed)

    # Define the map resolution (same as for the map)
        resolution = 0.02  # This is the same resolution used for your map grid

    # Create PoseStamped messages for each waypoint in the path
        for point in self.path_points:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header

        # Apply scaling to the path points to match the grid's resolution
            pose_stamped.pose.position.x = float(point[1]) * resolution  # Scale by resolution
            pose_stamped.pose.position.y = float(point[0]) * resolution  # Scale by resolution
            pose_stamped.pose.position.z = 0.0  # Assuming we're working in 2D, z = 0

        # Set orientation (you can adjust this depending on your robot's requirements)
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0  # No rotation (default)

        # Add the pose to the path
            path_msg.poses.append(pose_stamped)

        return path_msg


    def publish(self):
        """
        Publish both the occupancy grid (map) and the path.
        """
        # Create the map message
        map_msg = self.create_occupancy_grid(self.grid)

        # Publish the map
        self.map_publisher_.publish(map_msg)
        self.get_logger().info('Publishing map...')

        # Create the path message
        path_msg = self.create_path_msg()

        # Publish the path
        self.path_publisher_.publish(path_msg)
        self.get_logger().info(f'Publishing path with {len(path_msg.poses)} waypoints...')

def main(args=None):
    rclpy.init(args=args)
    node = LidarICPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()