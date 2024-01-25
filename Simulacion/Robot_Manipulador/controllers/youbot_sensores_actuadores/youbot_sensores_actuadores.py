"""youbot_sensores_actuadores controller."""
# You may need to import some classes of the controller module
# For instance From controller import Robot, Motor and more
from controller import Robot, PositionSensor
import sys
import time
import numpy as np
def main(robot):
    # A function that executes the algorithm to obtain sensor data and actuate motors
    # An object instante that contains all possible information about the robot
    # Get time step of the current world
    time_step = int(robot.getBasicTimeStep())

    # Time definition
    t_final = 20

    # Sample time
    t_s = 0.03

    # Definition of the time vector
    t = np.arange(0, t_final + t_s, t_s, dtype=np.double)

    # Definition of the names of rotational motors
    names_m = ['arm1', 'arm2', 'arm3', 'arm4', 'arm5']

    # Set and activate motors
    motors = get_motor(robot, names_m)

    # Definition of the names of rotational sensors
    names_s = ['arm1sensor', 'arm2sensor', 'arm3sensor', 'arm4sensor', 'arm5sensor']

    # Set and activate rotational sensors
    sensors = get_sensor_pos(names_s, time_step)

    # Definition of the desired angles for each joint in the manipulator
    q_c = np.zeros((5, t.shape[0]), dtype = np.double)

    # Definition of the desired angular velocities for each joint in the manipulator
    qp_c = np.zeros((5, t.shape[0]), dtype = np.double)

    # Definition of the desired angles for each joint in the manipulator
    q = np.zeros((5, t.shape[0] + 1), dtype = np.double)

    

    # Init System
    init_system(robot, motors, [0.0, -0.3, -0.8, -1.5, 0.0], time_step, t_s)
    # get initial conditions of the system
    q[:, 0] = get_angular_position(robot, sensors, time_step)

    # Simulation loop
    for k in range(0, t.shape[0]):
         if robot.step(time_step) != -1:
            tic = time.time()
            print("Angular displacement")
            print(q[:, k])
            # Compute desired values
            qp_c[:, k] = [0.0, 0.0, 0.0, 0.0, 0.0]

            # actuate the rotational motors of the system
            set_motor_vel(motors, qp_c[:, k])

            # Get current states of the system
            q[:, k + 1] = get_angular_position(robot, sensors, time_step)
            # Sample time saturation
            while (time.time() - tic <= t_s):
                None
            toc = time.time() - tic 
            print("Sample Time")
            print(toc)
    return None

def get_motor(robot, name):
    # Motor Configuration
    # INPUTS
    # robot                                             - robot class
    # name                                              - device name  
    # OUTPUT
    # motors                                             - motor devices        
    motor = []
    for k in name:
        motor.append(robot.getDevice(k))
    return motor

def set_motor_pos(motor, pos):
    # Set motor with position control
    # INPUT
    # motor                                             - device motor
    # pos                                               - desired position for each motor
    # OUTPUT
    # None
    size = len(motor)
    for k in range(0, size):
        motor[k].setPosition(pos[k])
    return None

def set_motor_vel(motor, vel):
    # Set motor with velocity control
    # INPUT
    # motor                                             - device motor
    # vel                                               - desired velocity for each motor
    # OUTPUT
    # None
    size = len(motor)
    for k in range(0, size):
        motor[k].setPosition(float('inf'))
        motor[k].setVelocity(vel[k])
    return None

def get_sensor_pos(names, time_step):
    # Configuration of the rotational motor sensors
    # INPUT 
    # names                                              - list of the names for each sensor
    # time_step                                          - sample time simulation
    # OUTPUTS 
    # sensors                                            - A list with different objects for positional sensing
    sensors = []
    for k in names:
        instance = PositionSensor(k)
        instance.enable(time_step)
        sensors.append(instance)
    return sensors

def get_angular_position(robot, sensors, time_step):
    # A Function that enables the acquisition of the angular displacement from each rotational sensor
    # INPUT 
    # robot                                                                 - object instance 
    # sensors                                                               - A list with sensor objects            
    # OUTPUT                                                                
    # q                                                                     - A vector array with the respective information        
    q = np.zeros((5, ), dtype=np.double)
    size = len(sensors)
    if robot.step(time_step) != -1:
        for k in range(0, size):
            data = sensors[k].getValue()
            q[k] = data
    return q
def init_system(robot, motors, q_c, time_step, t_s):
    # Function that moves the robot to an initial configuration
    # INPUT 
    # robot                                        - A robot object that contains all the required information
    # motors                                       - A list with the required motor objects
    # q_c                                          - A vector of desired initial angular values
    # time_step                                    - sample time of the simulation webots
    # ts                                           - sample time of the simulation
    # OUTPUT
    # None
    for k in range(0, 100):
         if robot.step(time_step) != -1:
            tic = time.time()
            print("Init System")
            set_motor_pos(motors, q_c)
            # Sample time saturation
            while (time.time() - tic <= t_s):
                None
            toc = time.time() - tic 
    return None


if __name__ == '__main__':
    try:
        robot = Robot()
        main(robot)
        pass
    except(KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass