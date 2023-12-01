from project.utils import *
from project.Robot import UR5Arm
import numpy as np
import time

from Arm_Lib import Arm_Device

Arm = Arm_Device()


class PIDController:
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        # Proportional term
        P_term = self.Kp * error
        # Integral term
        self.integral += error * dt
        I_term = self.Ki * self.integral
        # Derivative term
        derivative = (error - self.previous_error) / dt
        D_term = self.Kd * derivative
        # Update previous error
        self.previous_error = error
        # Total output
        return P_term + I_term + D_term


# Step 1: Generate a joint space path
def generate_joint_space_path(start_angles, end_angles, steps):
    path = [np.linspace(start_, end_, steps) for start_, end_ in zip(start_angles, end_angles)]
    return np.transpose(path)


# Step 2: Command the arm to follow the path
def command_arm_to_follow_path(robot, path, s_time):
    for joint_angles in path:
        # Convert angles from radians to degrees for the robot's actuators
        angles_in_degrees = joint_angles * (180 / np.pi)
        # Assuming Arm_Device is instantiated and the function Arm_serial_servo_write6 is available
        # to send the joint angles to the robot. Replace 'Arm_Device' with your actual object.
        robot.Arm_serial_servo_write6(angles_in_degrees[0], angles_in_degrees[1], angles_in_degrees[2],
                                      angles_in_degrees[3], angles_in_degrees[4], 90, 1500)
        time.sleep(s_time)  # Wait for some time interval before sending the next set of angles


def command_arm_to_follow_path_with_feedback(robot, path, s_time, pid_controllers, gripper):
    for joint_angles in path:
        actual_joint_angles = measure_actual_position(robot)
        error = joint_angles - actual_joint_angles
        adjusted_angles = []

        for i in range(len(joint_angles)):
            dt = 0.1  # Assuming 0.1 seconds as the time interval
            adjustment = pid_controllers[i].update(error[i], dt)
            adjusted_angles.append(joint_angles[i] + adjustment)

        angles_in_degrees = np.array(adjusted_angles) * (180 / np.pi)
        robot.Arm_serial_servo_write6(*angles_in_degrees, gripper, 1500)
        time.sleep(dt)


# Step 3: Measure the actual position (this function should be replaced with actual sensor readout)
def measure_actual_position(robot):
    joint_angles = []
    for i in range(1, 6):
        if robot.Arm_serial_servo_read(i) is not None:
            time.sleep(0.1)
            joint_angles.append(robot.Arm_serial_servo_read(i))
        else:
            time.sleep(0.1)
            joint_angles.append(0)
    print(joint_angles)
    # Convert the servo positions to angles, if necessary
    # joint_angles = [convert_position_to_angle(pos) for pos in joint_angles]
    time.sleep(0.1)
    return np.array(joint_angles)


# Step 4: Evaluate the error
def evaluate_path_following_error(robot, desired_path):
    error_path = []
    for desired_joint_angles in desired_path:
        actual_joint_angles = measure_actual_position(robot)
        error = np.array(desired_joint_angles) - np.array(actual_joint_angles)
        error_path.append(error)
    return error_path


def rotation_matrix_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])


def rotation_matrix_y(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])


def rotation_matrix_x(theta):
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])


def calculate_rotation_matrix(angles):
    q1, q2, q3, q4, q5 = np.radians(angles)  # Convert degrees to radians
    R01 = rotation_matrix_z(q1)
    R12 = rotation_matrix_y(-q2)
    R23 = rotation_matrix_y(-q3)
    R34 = rotation_matrix_y(-q4)
    R45 = rotation_matrix_x(-q5)
    R0T = R01 @ R12 @ R23 @ R34 @ R45  # Matrix multiplication
    return R0T


def return_rotations(angles):
    q1, q2, q3, q4, q5 = np.radians(angles)  # Convert degrees to radians
    R01 = rotation_matrix_z(q1)
    R12 = rotation_matrix_y(-q2)
    R23 = rotation_matrix_y(-q3)
    R34 = rotation_matrix_y(-q4)
    R45 = rotation_matrix_x(-q5)
    rotations = [R01, R12, R23, R34, R45]
    return rotations


def return_pd(rotations, points):
    pd = points[0] + rotations[0] @ rotations[1] @ (
            points[1] + rotations[2] @ (points[2] + rotations[3] @ rotations[4] @ points[3]))
    return pd


def path_planner(N, q_0, q_dest, k_p, k_i, k_d, gripper):
    s_time = 1500  # milliseconds
    ex = np.array([1, 0, 0])
    ey = np.array([0, 1, 0])
    ez = np.array([0, 0, 1])

    # tol = np.array(np.transpose([[0.02, 0.02, 0.02, 0.001, 0.001, 0.001]]))
    # Nmax = 200
    # alpha = 0.1

    # Define all the joint lengths [m]
    l0 = 61 * 10 ** -3
    l1 = 43.5 * 10 ** -3
    l2 = 82.85 * 10 ** -3
    l3 = 82.85 * 10 ** -3
    l4 = 73.85 * 10 ** -3
    l5 = 54.57 * 10 ** -3
    # Define the position vectors from i-1 -> i
    P01 = (l0 + l1) * ez
    P12 = np.zeros(3)
    P23 = l2 * ez
    P34 = l3 * ez
    P45 = np.zeros(3)
    P5T = (l4 + l5) * ez

    H = np.array([ez, ez, ez, ez, ez]).T
    P = np.array([P01, P12, P23, P34, P45, P5T]).T
    joint_type = [0, 0, 0, 0, 0]
    robot = Arm
    # Define your start and end angles here (in radians)
    start_angles = np.array(q_0) * np.pi / 180
    end_angles = np.array(q_dest) * np.pi / 180
    steps = N  # Define the number of steps you want in the path

    pid_controllers = [PIDController(k_p, k_i, k_d) for _ in range(N)]

    # Generate the path
    joint_space_path = generate_joint_space_path(start_angles, end_angles, steps)

    # Command the arm to follow the path
    command_arm_to_follow_path_with_feedback(robot, joint_space_path, s_time, pid_controllers, gripper)

    # Measure the error
    path_following_error = evaluate_path_following_error(robot, joint_space_path)

    # Print or process the error as needed
    print("Path following errors:")
    print(path_following_error)
