"""robot_movil controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, GPS, InertialUnit
import sys
import time
import numpy as np

def get_motor(robot, name):
    # Motor Configuration
    # INPUTS
    # robot                                             - robot class
    # name                                              - device name  
    # OUTPUT
    # motor                                             - device motor         
    motor = robot.getDevice(name)
    return motor

def set_motor(motor):
    # Set motor with velocity control
    # INPUT
    # motor                                             - device motor
    # OUTPUT
    # motor                                             - device motor after set up
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)
    return motor

def set_motors_velocity(motor_r, motor_l, w):
    # Set velocity for motors
    # INPUT
    # motor_r                                           - motor device
    # motor_l                                           - motor device
    # w                                                 - vector of angular velocities
    # OUTPUT
    # None
    w_r = w[0]
    w_l = w[1]
    motor_r.setVelocity(w_r)
    motor_l.setVelocity(w_l)
    return None

def set_gps(name, time_step):
    # Motor configuration
    # INPUTS
    # name                                  - defined name of the sensor
    # time_step                             - sample time defined in the simulation
    # OUTPUT                                    
    # gps                                   - object with all the necessary attributes to access the information of the sensor
    gps = GPS(name)
    gps.enable(time_step)
    return gps

def set_imu(name, time_step):
    # IMU configuration
    # INPUTS
    # name                                  - defined name of the sensor
    # time_step                             - sample time defined in the simulation
    # OUTPUT                                    
    # imu                                   - object with all the necessary attributes to access the information of the sensor
    imu = InertialUnit(name)
    imu.enable(time_step)
    return imu

def get_states(robot, gps, imu, time_step, L):
    # split internal values
    r = L[0] 
    l = L[1]
    a = L[2]
    if robot.step(time_step) != -1:
        position = gps.getValues()
        orientation = imu.getRollPitchYaw()
        data = [position[0] + a*np.cos(orientation[2]), position[1] + a*np.sin(orientation[2]), orientation[2]]
    x = np.array(data)
    return x

def main(robot):
    # Get the time step of the current world.
    time_step = int(robot.getBasicTimeStep())
    # Time Definition
    t_final = 20
    # Sample time
    t_s = 0.03
    # Time simulation
    t = np.arange(0, t_final + t_s, t_s, dtype=np.double)

    # System states
    x = np.zeros((3, t.shape[0]+1), dtype = np.double)

    # Signals designed to move the motors
    u = np.zeros((2, t.shape[0]), dtype = np.double)

    # Robot Acuators
    motor_left = get_motor(robot, "motor_izquierdo")
    motor_right = get_motor(robot, "motor_derecho")

    # Set Robot actuators
    motor_left = set_motor(motor_left)
    motor_right = set_motor(motor_right)

    # Robot Sensors
    # Get system sensors
    gps = set_gps("gps", time_step)
    imu = set_imu("inertial unit", time_step)

    # Parameters of the robot
    r = 0.190/2
    l = 0.381
    a = 0.01
    L = [r, l ,a]


    # SImulation loop
    for k in range(0, t.shape[0]):
        if robot.step(time_step) != -1:
            tic = time.time()
            # Control values generation
            u[:, k] = np.array([0.5, 0.5])

            # Send control values to the robot
            set_motors_velocity(motor_right, motor_left, u[:, k])
            print('Moving Robot')
            print("States of the robot")
            print(x[:, k])
            x[: , k+1] = get_states(robot, gps, imu, time_step, L)

            # Sample time saturation
            while (time.time() - tic <= t_s):
                None
            toc = time.time() - tic 
            print(toc)

    # Set zero values to the robot 
    set_motors_velocity(motor_right, motor_left, np.array([[0], [0]]))
    return None
# Enter here exit cleanup code.
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