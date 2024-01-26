"""controlador controller."""
"""robot_movil controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, GPS, InertialUnit, PositionSensor
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

def plot_results(x, xd, t):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

    # Plot x_estimate with label 'x_estimate'
    ax1.set_xticklabels([])
    state_1, = ax1.plot(t, x[0, 0:t.shape[0]], color='#C04747', lw=1.5, ls="-")
    state_2, = ax2.plot(t, x[1, 0:t.shape[0]], color='#47C05B', lw=1.5, ls="-")
    state_3, = ax1.plot(t, xd[0, 0:t.shape[0]], color='#C04747', lw=1.5, ls="--")
    state_4, = ax2.plot(t, xd[1, 0:t.shape[0]], color='#47C05B', lw=1.5, ls="--")
    # Add a legend
    ax1.legend([state_1, state_3],
                [r'x', r'x_d'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
             
    ax2.legend([state_2, state_4],
                [r'y', r'y_d'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
    
    
    
    ax1.grid(color='#949494', linestyle='-.', linewidth=0.8)
    ax2.grid(color='#949494', linestyle='-.', linewidth=0.8)
    
    ax2.set_xlabel(r"$Time}[s]$", labelpad=5)
    # Show the plot
    plt.show()

def plot_results_control(x, t):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

    # Plot x_estimate with label 'x_estimate'
    ax1.set_xticklabels([])
    state_1, = ax1.plot(t, x[0, 0:t.shape[0]], color='#C04747', lw=1.5, ls="-")
    state_2, = ax2.plot(t, x[1, 0:t.shape[0]], color='#47C05B', lw=1.5, ls="-")
    # Add a legend
    ax1.legend([state_1],
                [r'u'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
             
    ax2.legend([state_2],
                [r'w'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
    
    
    
    ax1.grid(color='#949494', linestyle='-.', linewidth=0.8)
    ax2.grid(color='#949494', linestyle='-.', linewidth=0.8)
    
    ax2.set_xlabel(r"$Time}[s]$", labelpad=5)
    # Show the plot
    plt.show()


def get_sensor_pos(name, time_step):
    # Motor Configuration
    # INPUTS
    # name                                              - device name  
    # OUTPUT
    # pose                                             - device motor         
    pose = PositionSensor(name)
    pose.enable(time_step)
    return pose

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
def get_angular_pos(pos, time_step):
    # Function to obtain the angular displacement of the motors
    # INPUTS
    # pos                                               - This is the object that containts the angular displacement information
    # time_step                                         - sample time of the simulation
    # OUTPUT
    # data                                              - angular displacement
    data = pos.getValue()
    return data

def get_angular_vel(motor):
    # Function that obtains the angular displacement of the motors
    # INPUTS
    # motor                                             - Object that contains the information of the sensor
    # OUTPUT
    # data                                              - angular displacement
    data = motor.getVelocity()
    return data

def get_states(robot, gps, imu, time_step, L):
    # Function that obtains the internatl states of the system
    # INPUTS
    # robot                                                                            - An object which includes all the necessary information of the robot
    # gps                                                                              - An object that contains the gps sensor information
    # imu                                                                              - An object that contains the information of imu sensor
    # time_step                                                                        - sample_time of the simulation
    # L                                                                                - List object that constains system parameters such as radius, lenght between wheels and distance to the point of interest.
    # OUTPUT
    # x                                                                                - A vector with the internal states of the system   
    r = L[0] 
    l = L[1]
    a = L[2]
    if robot.step(time_step) != -1:
        position = gps.getValues()
        orientation = imu.getRollPitchYaw()
        data = [position[0] + a*np.cos(orientation[2]), position[1] + a*np.sin(orientation[2]), orientation[2]]
    x = np.array(data)
    return x

def transformation(u, L):
    # Function that maps the linear and angular velocites to the angular velocities of each wheel
    # INPUT 
    # u                                                                                 - A vector with linear and angular velocity
    # L                                                                                 - A list that contains the system parameters such as radius, length and distance to control point
    # OUTPUT
    # w_r                                                                                - Desired angular velocity for the right wheel
    # w_l                                                                                - Desired angular velocity for the left wheel
    # Split system parameters
    r = L[0] 
    l = L[1]
    a = L[2]
    # Resize generalized vector
    u = u.reshape((2, 1))
    # Auxiliar linear mapping
    A = np.array([[r/2, r/2], [r/l, -r/l]], dtype=np.double)

    # Linear mapping
    W = np.linalg.inv(A)@u

    W = W.reshape((2, ))
    return W

def law_control(xd, xdp, x, k1, k2, L):
    # Function that computes the control values to move the system to a desired states
    # INPUT
    # xd                                          - Desired state in the plane x y
    # xdp                                         - Time derivative of the desired state
    # x                                           - Current state of the system
    # k1                                          - Controller gain value
    # k2                                          - Controller gain value
    # L                                           - A list that contains system parameters
    # OUTPUT
    # u                                           - control action

    # Split system parameters
    r = L[0] 
    l = L[1]
    a = L[2]

    # Reshape desired states of the system
    xd = xd.reshape((2, ))
    xdp = xdp.reshape((2, ))

    # Obtaint only the displacement in X Y and the angular displacement
    theta = x[2]
    x = x[0:2]
    x = x.reshape((2, ))

    # Compute control error
    xe = xd - x

    # Control gains
    K1 = k1*np.eye(2, 2)
    K2 = k2*np.eye(2, 2)

    # Compute Jacobian Matrix
    J = np.array([[np.cos(theta), -a*np.sin(theta)], [np.sin(theta), a*np.cos(theta)]])
    J_1 = np.linalg.inv(J)
    K2_1 = np.linalg.inv(K2)

    u = J_1@(xdp + K2@np.tanh(K2_1@K1@xe))

    # Reshape control value
    u = u.reshape((2, ))
    return u


def main(robot):
    # Get the time step of the current world.
    time_step = int(robot.getBasicTimeStep())
    # Time Definition
    t_final = 10
    # Sample time
    t_s = 0.03
    # Time simulation
    t = np.arange(0, t_final + t_s, t_s, dtype=np.double)

    # System states
    x = np.zeros((3, t.shape[0]+1), dtype = np.double)

    # Desired system states
    xd = np.zeros((2, t.shape[0]), dtype = np.double)
    xd[0, :] = 1*np.ones((1, t.shape[0]))
    xd[1, :] = 0*np.ones((1, t.shape[0]))

    xdp = np.zeros((2, t.shape[0]), dtype = np.double)
    xdp[0, :] = 0.0*np.ones((1, t.shape[0]))
    xdp[1, :] = 0.0*np.ones((1, t.shape[0]))

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

    motor_left_sensor_pos = get_sensor_pos("sensor_izquierdo", time_step)
    motor_right_sensor_pos = get_sensor_pos("sensor_derecho", time_step)

    # Parameters of the robot
    r = 0.02
    l = 0.11
    a = 0.055
    L = [r, l ,a]

    # Control gains
    k1 = 1
    k2 = 0.5

    # Get initial position
    x[: , 0] = get_states(robot, gps, imu, time_step, L)

    # SImulation loop
    for k in range(0, t.shape[0]):
        if robot.step(time_step) != -1:
            tic = time.time()
            # Control law
            # Control
            u[:, k] = law_control(xd[:, k], xdp[:, k], x[:, k], k1, k2, L)
            w = transformation(u[:, k], L)

            # Send control values to the robot
            set_motors_velocity(motor_right, motor_left, w)

            # Get new information of the system
            x[: , k+1] = get_states(robot, gps, imu, time_step, L)

            # Sample time saturation
            while (time.time() - tic <= t_s):
                None
            toc = time.time() - tic 

    # Set zero values to the robot 
    set_motors_velocity(motor_right, motor_left, np.array([[0], [0]]))
    plot_results(x, xd, t)
    plot_results_control(u, t)
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