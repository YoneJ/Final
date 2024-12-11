import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan

from sklearn.neighbors import NearestNeighbors
from pose_manager import PoseManager
from rclpy.node import Node

class RobotLocalizationNode(Node):
    def __init__(self):
        super().__init__('robot_localization_node')
        
        # Initialize the robot's pose: [x, y, theta]
        self.pose = np.array([0.0, 0.0, 0.0])  # Initial pose at origin facing forward
        self.prev_scan = None  # To store the previous scan data
        self.current_scan = None  # To store the current scan data
        # Add other initialization as needed (subscribers, publishers, etc.)
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            
            '/scan',  # Replace with your lidar topic
            self.lidar_callback,
            10
        )
        self.get_logger().info("Robot Localization Node Initialized")

    def lidar_callback(self, msg):
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
            self.current_scan = points

            if self.prev_scan is not None:
                # Apply ICP to calculate the relative transformation
                delta_pose = self.icp(self.prev_scan, points)
                self.pose = self.update_pose(self.pose, delta_pose)
                
                self.get_logger().info(f'Updated Pose: {self.pose}')
                
                PoseManager().set_pose(self.pose)

                print(f"Robot's Position: x={self.pose[0]:.2f}, y={self.pose[1]:.2f}, theta={self.pose[2]:.2f}")

            self.prev_scan = self.current_scan

    def icp(self, source, target, max_iterations=25, tolerance=1e-3):
        """
        Perform ICP between source and target point clouds.
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

        return np.array([-dx, dy, -dtheta])

    def update_pose(self, pose, delta_pose):
        """
        Update the robot's pose based on the delta pose.
        """
        dx, dy, dtheta = delta_pose
        theta = pose[2]

        # Update the pose with respect to the robot's current orientation
        x_new = pose[0] + dx * np.cos(theta) - dy * np.sin(theta)
        y_new = pose[1] + dx * np.sin(theta) + dy * np.cos(theta)
        theta_new = (pose[2] + dtheta + np.pi) % (2 * np.pi) - np.pi  # update and limit angle to [-pi, pi]

        return np.array([x_new, y_new, theta_new])

def main(args=None):
    rclpy.init(args=args)
    node = RobotLocalizationNode()

    # Spin the node to process callbacks (like lidar_callback)
    rclpy.spin(node)

    # Clean up and shut down when the node is stopped
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
