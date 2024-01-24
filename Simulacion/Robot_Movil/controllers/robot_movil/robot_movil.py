"""robot_movil controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
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

def main(robot):
    # Get the time step of the current world.
    time_step = int(robot.getBasicTimeStep())
    # Time Definition
    t_final = 20
    # Sample time
    t_s = 0.03
    # Time simulation
    t = np.arange(0, t_final + t_s, t_s, dtype=np.double)

    # Signals designed to move the motors
    u = np.zeros((2, t.shape[0]), dtype = np.double)

    # Robot Acuators
    motor_left = get_motor(robot, "motor_izquierdo")
    motor_right = get_motor(robot, "motor_derecho")

    # Set Robot actuators
    motor_left = set_motor(motor_left)
    motor_right = set_motor(motor_right)

    for k in range(0, t.shape[0]):
        if robot.step(time_step) != -1:
            tic = time.time()
            # Control values generation
            u[:, k] = np.array([0.5, 0.5])

            # Send control values to the robot
            set_motors_velocity(motor_right, motor_left, u[:, k])
            print('Moving Robot')

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