import pybullet as p
import pybullet_data
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
import time
import random


class SelfAvoidance:
    def __init__(self, urdf_path, joint_indices=None, gui=True, obstacle_paths=None, obstacle_scale=(1, 1, 1)):
        """
        Initialize the SelfAvoidance planner.

        :param urdf_path: str, path to the URDF file of the robot.
        :param joint_indices: list, optional list of joint indices to control.
        :param gui: bool, if True, enables PyBullet GUI mode.
        :param obstacle_paths: list of str, paths to obstacle STL files. If None, no obstacles are loaded.
        :param obstacle_scale: tuple, scaling factors for the obstacle meshes.
        """
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        self.gui = gui  # Store the GUI flag to control visualization later
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Set logging level to ERROR
        p.setPhysicsEngineParameter(enableFileCaching=0)
        p.setInternalSimFlags(0)


        # Load robot from URDF
        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True)

        # Load obstacles from STL files
        self.obstacle_ids = []
        if obstacle_paths:
            for obstacle_path in obstacle_paths:
                collision_shape_id = p.createCollisionShape(
                    shapeType=p.GEOM_MESH,
                    fileName=obstacle_path,
                    meshScale=obstacle_scale
                )
                # Set the obstacle's position to the origin (0, 0, 0) with no rotation
                obstacle_id = p.createMultiBody(
                    baseMass=0,
                    baseCollisionShapeIndex=collision_shape_id,
                    basePosition=(0, 0, 0),
                    baseOrientation=(0, 0, 0, 1)
                )
                self.obstacle_ids.append(obstacle_id)

        # Define joint indices to control
        self.joint_indices = joint_indices or [
            i for i in range(p.getNumJoints(self.robot_id)) 
            if p.getJointInfo(self.robot_id, i)[2] == p.JOINT_REVOLUTE
        ]
         # get jointlimitfromurdf
        self.joint_limits = [
        (p.getJointInfo(self.robot_id, i)[8], p.getJointInfo(self.robot_id, i)[9])
        for i in self.joint_indices
        ]

        # Flag to check if connected to physics server
        self.connected_to_server = True

    def check_collision_with_objects(self, joint_configuration):
        """
        Check if the robot collides with any objects or itself given a joint configuration.

        :param joint_configuration: list of float, the joint positions for the robot.
        :return: bool, True if there is a collision, False otherwise.
        """
        # Set robot joint positions to the specified configuration
        for joint_index, joint_value in zip(self.joint_indices, joint_configuration):
            p.resetJointState(self.robot_id, joint_index, joint_value)

        # Step the simulation to apply changes
        p.stepSimulation()

        # Check self-collision
        if p.getContactPoints(bodyA=self.robot_id, bodyB=self.robot_id):
            return True

        # Check collision with each object
        for object_id in self.obstacle_ids:
            if p.getContactPoints(bodyA=self.robot_id, bodyB=object_id):
                return True

        # No collision detected
        return False

    def set_joint_positions(self, joint_positions):
        for idx, pos in zip(self.joint_indices, joint_positions):
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=pos
            )
        p.stepSimulation()

    def check_collision(self):
        contact_points = []
        contact_points_self = p.getContactPoints(bodyA=self.robot_id, bodyB=self.robot_id)
        contact_points += [(pt[5], pt[6], pt[7]) for pt in contact_points_self]
        if self.obstacle_ids is not None:
            for object_id in self.obstacle_ids:
                contact_points_obstacle = p.getContactPoints(bodyA=self.robot_id, bodyB=object_id)
                contact_points += [(pt[5], pt[6], pt[7]) for pt in contact_points_obstacle]
        return len(contact_points) > 0, contact_points

    def rrt_connect_plan(self, start, target, max_iterations=1000):

        path = [start]
        for _ in range(max_iterations):
            random_goal = target if random.random() > 0.5 else [
                random.uniform(limit[0], limit[1]) for limit in self.joint_limits
            ]
            if self.steer(path[-1], random_goal):
                path.append(random_goal)
                if np.linalg.norm(np.array(random_goal) - np.array(target)) < 0.1:
                    path.append(target)
                    return path  # Path successfully found
        return None

    def steer(self, from_config, to_config, step_size=0.1):
        direction = np.array(to_config) - np.array(from_config)
        distance = np.linalg.norm(direction)
        num_steps = int(distance / step_size)
        for i in range(1, num_steps + 1):
            intermediate = from_config + (direction / distance) * i * step_size
            self.set_joint_positions(intermediate)
            in_collision, _ = self.check_collision()
            if in_collision:
                return False
        return True

    def chomp_optimize_path(self, path, max_iterations=20, smooth_factor=0.1):
        optimized_path = path[:]
        for _ in range(max_iterations):
            for i in range(1, len(path) - 1):
                prev, current, next_ = path[i - 1], path[i], path[i + 1]
                optimized_point = current + smooth_factor * (np.array(prev) + np.array(next_) - 2 * np.array(current))
                optimized_path[i] = optimized_point
                self.set_joint_positions(optimized_point)
                if self.check_collision()[0]:
                    return path
        return optimized_path

    def continuous_collision_detection(self, start, end, steps=100):
        for t in np.linspace(0, 1, steps):
            intermediate = (1 - t) * np.array(start) + t * np.array(end)
            self.set_joint_positions(intermediate)
            if self.check_collision()[0]:
                return False
        return True

    def plan_path(self, start, target, retries=30):
        for attempt in range(retries):
            print(f"Planning attempt {attempt + 1}")
            path = self.rrt_connect_plan(start, target)
            if path is None:
                print("RRT-Connect failed.")
                continue
            
            if not self.continuous_collision_detection(path[0], path[-1]):
                print("Path fails continuous collision detection.")
                continue
            
            smooth_factor = 0.1 + 0.05 * attempt
            optimized_path = self.chomp_optimize_path(path, smooth_factor=smooth_factor)
            
            if self.continuous_collision_detection(optimized_path[0], optimized_path[-1]):
                print(f"Path found on attempt {attempt + 1}")
                shortened_path = self.shorten_path(optimized_path)
                interpolated_path = self.interpolate_path(shortened_path, total_steps=400)
                return interpolated_path
            else:
                print("Optimized path fails collision detection. Retrying with a new approach.")
        
        print("No collision-free path could be found after all retries.")
        return None

    def shorten_path(self, path):
        shortened_path = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self.continuous_collision_detection(path[i], path[j]):
                    shortened_path.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                i += 1
                shortened_path.append(path[i])
        return shortened_path

    def interpolate_path(self, path, total_steps=200):
        interpolated_path = []
        for i in range(len(path) - 1):
            segment_start = np.array(path[i])
            segment_end = np.array(path[i + 1])
            num_steps = max(1, total_steps // (len(path) - 1))
            for t in np.linspace(0, 1, num_steps, endpoint=False):
                interpolated_path.append((1 - t) * segment_start + t * segment_end)
        interpolated_path.append(path[-1])
        return interpolated_path

    def visualize_path(self, path, delay=0.05):
        print("Visualizing path...")
        for step, config in enumerate(path):
            print(f"Step {step + 1}: {config}")
            self.set_joint_positions(config)
            time.sleep(delay)

    def close(self):
        self.connected_to_server = False
        p.disconnect()


if __name__ == "__main__":
    # Initialize the ROS 2 node
    rclpy.init()
    node = rclpy.create_node('robot_path_planner')

    # Get package paths
    package_path = get_package_share_directory('wp5_planner')
    urdf_path = package_path + "/../wp5-robotic-arms/urdf/ur5_tool_sf.urdf"
    obstacle_paths = [
        package_path + "/data/meshes/pointcloud_target_transformed.stl",
        package_path + "/../wp5-robotic-arms/meshes/ridgeback.stl",
        package_path + "/../wp5-robotic-arms/meshes/ridgebackMount.stl"
    ]

    planner = SelfAvoidance(
        urdf_path=urdf_path,
        obstacle_paths=obstacle_paths,
        obstacle_scale=(1, 1, 1)
    )

    start_configuration = [2.2893900871276855, -1.2440274397479456, 2.257026195526123, -4.511027757321493, -2.331780974064962, 1.4161226749420166]
    target_configuration = [0.0, -1.57, 0.0, -1.57, 0.0, -0.0]

    print("Collision check: ", planner.check_collision_with_objects(target_configuration))

    path = planner.plan_path(start_configuration, target_configuration)

    if path is not None:
        print("Collision-free path found!")
        if planner.gui:
            planner.visualize_path(path, delay=0.01)
    else:
        print("No collision-free path could be found.")

    planner.close()

    # Shut down the ROS 2 node
    rclpy.shutdown()