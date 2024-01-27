"""youbot_controller controller."""
# You may need to import some classes of the controller module
# For instance From controller import Robot, Motor and more
from controller import Robot, PositionSensor, Supervisor
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
def main(robot):
    # A function that executes the algorithm to obtain sensor data and actuate motors
    # An object instante that contains all possible information about the robot
    # Get time step of the current world
    time_step = int(robot.getBasicTimeStep())

    # Time definition
    t_final = 10

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

    # Definition of the position vector
    x = np.zeros((3, t.shape[0] + 1), dtype = np.double)

    # Desired Point
    xd = np.zeros((3, t.shape[0] + 1), dtype = np.double)
    xd[0, :] = 0.48
    xd[1, :] = 0.0
    xd[2, :] = 0.21

    xdp = np.zeros((3, t.shape[0] + 1), dtype = np.double)
    xdp[0, :] = 0.0
    xdp[1, :] = 0.0
    xdp[2, :] = 0.0

    # Control Gains
    k1 = 1.0
    k2 = 0.5

    # Kuka Youbot Parameters
    a0 = 0.156
    a1 = 0.033
    d1 = 0.147
    a2 = 0.155
    a3 = 0.135
    a4 = 0.218
    L = [a0, a1, d1, a2, a3, a4]

    # DEsired states object
    supervisor_node = robot.getFromDef('ball')
    translation_field = supervisor_node.getField('translation')

    # Init System
    init_system(robot, motors, [0.0, -0.1, -0.1, -0.2, -0.0], time_step, t_s)
    # get initial conditions of the system
    q[:, 0] = get_angular_position(robot, sensors, time_step)
    x[:, 0] = forward_kinematics(q[:, 0], L)
    xd[:, 0] =  translation_field.getSFVec3f()

    # Simulation loop
    for k in range(0, t.shape[0]):
         if robot.step(time_step) != -1:
            tic = time.time()
            print(xd[:, k])
            # Compute desired values
            qp_c[:, k ] = control_law(xd[:, k], xdp[:, k], q[:, k], L, k1, k2)

            # actuate the rotational motors of the system
            set_motor_vel(motors, qp_c[:, k])

            # Get current states of the system
            q[:, k + 1] = get_angular_position(robot, sensors, time_step)
            x[:, k + 1] = forward_kinematics(q[:, k+1], L)
            xd[:, k +1] =  translation_field.getSFVec3f()
            # Sample time saturation
            while (time.time() - tic <= t_s):
                None
            toc = time.time() - tic 
    set_motor_vel(motors, [0.0, 0.0, 0.0, 0.0, 0.0])
    plot_results(x, xd, t)
    plot_results_control(q, qp_c, t)
    return None
def forward_kinematics(q, L):
    # Function that computes the forwrd kinematics
    # INPUT
    # q                                          - internal states of the system
    # L                                          - system parameters
    # OUTPUT
    # P                                          - A vector with the position of the end efector
    a_0 = L[0]
    a_1 = L[1]
    d_1 = L[2]
    a_2 = L[3]
    a_3 = L[4]
    a_4 = L[5]

    theta_1 = q[0]
    theta_2 = q[1]
    theta_3 = q[2]
    theta_4 = q[3]
    theta_5 = q[4]
    # Split elements
    px = a_0 + a_1*np.cos(theta_1) -a_2*np.sin(theta_2)*np.cos(theta_1) -a_3*np.sin(theta_2 + theta_3)*np.cos(theta_1) -a_4*np.sin(theta_2 + theta_3 +theta_4)*np.cos(theta_1)
    py = (a_1 - a_2*np.sin(theta_2) - a_3*np.sin(theta_2 + theta_3) - a_4*np.sin(theta_2 + theta_3 +theta_4))*np.sin(theta_1)
    pz = a_2*np.cos(theta_2) + a_3*np.cos(theta_2 + theta_3) + a_4*np.cos(theta_2 + theta_3 +theta_4) + d_1
    P = np.array([px, py, pz], dtype=np.double)
    return P

def Jacobian(q, L):
    # Function that computes the forwrd kinematics
    # INPUT
    # q                                          - internal states of the system
    # L                                          - system parameters
    # OUTPUT
    # J                                          - Jacobian matrix
    a_0 = L[0]
    a_1 = L[1]
    d_1 = L[2]
    a_2 = L[3]
    a_3 = L[4]
    a_4 = L[5]

    theta_1 = q[0]
    theta_2 = q[1]
    theta_3 = q[2]
    theta_4 = q[3]
    theta_5 = q[4]
    
    # Formulate elements of the jacobian matrix
    J11 = -a_1*np.sin(theta_1) + a_2*np.sin(theta_1)*np.sin(theta_2) + a_3*np.sin(theta_1)*np.sin(theta_2 + theta_3) + a_4*np.sin(theta_1)*np.sin(theta_2 + theta_3 + theta_4)
    J12 = -a_2*np.cos(theta_1)*np.cos(theta_2) - a_3*np.cos(theta_1)*np.cos(theta_2 + theta_3) - a_4*np.cos(theta_1)*np.cos(theta_2 + theta_3 +theta_4)
    J13 = -a_3*np.cos(theta_1)*np.cos(theta_2 + theta_3) - a_4*np.cos(theta_1)*np.cos(theta_2 + theta_3 +theta_4)
    J14 = -a_4*np.cos(theta_1)*np.cos(theta_2 + theta_3 + theta_4)

    J21 = (a_1 - a_2*np.sin(theta_2) - a_3*np.sin(theta_2 + theta_3) - a_4*np.sin(theta_2 + theta_3 +theta_4))*np.cos(theta_1)
    J22 = (-a_2*np.cos(theta_2) -a_3*np.cos(theta_2 + theta_3) - a_4*np.cos(theta_2 + theta_3 +theta_4))*np.sin(theta_1)
    J23 = (-a_3*np.cos(theta_2 + theta_3) - a_4*np.cos(theta_2 + theta_3 + theta_4 ))*np.sin(theta_1)
    J24 = -a_4*np.sin(theta_1)*np.cos(theta_2 + theta_3 +theta_4)

    J31 = 0.0
    J32 = -a_2*np.sin(theta_2) - a_3*np.sin(theta_2 + theta_3) - a_4*np.sin(theta_2 + theta_3 +theta_4)
    J33 = -a_3*np.sin(theta_2 + theta_3) - a_4*np.sin(theta_2 + theta_3 + theta_4)
    J34 = -a_4*np.sin(theta_2 + theta_3 + theta_4)

    J = np.array([[J11, J12, J13, J14], [J21, J22, J23, J24], [J31, J32, J33, J34]], dtype=np.double)
    return J

def control_law(xd, xdp, q, L, k1, k2):
    x = forward_kinematics(q, L)
    x = x.reshape((3, 1))

    xd = xd.reshape((3, 1))

    xdp = xdp.reshape((3, 1))

    xe = xd - x

    J = Jacobian(q, L)

    K1 = k1*np.eye(3, 3)
    K2 = k2*np.eye(3, 3)
    K2_1 = np.linalg.inv(K2)

    J_1 = np.linalg.pinv(J)
    u = J_1@(xdp + K2@np.tanh(K2_1@K1@xe))

    u_c = np.zeros((5, ))
    u_c[0:4] = u[:, 0]

    return u_c

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

def plot_results(x, xd, t):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))

    # Plot x_estimate with label 'x_estimate'
    ax1.set_xticklabels([])
    state_1, = ax1.plot(t, x[0, 0:t.shape[0]], color='#C04747', lw=1.5, ls="-")
    state_2, = ax2.plot(t, x[1, 0:t.shape[0]], color='#47C05B', lw=1.5, ls="-")
    state_3, = ax3.plot(t, x[2, 0:t.shape[0]], color='#478DC0', lw=1.5, ls="-")
    state_1_1, = ax1.plot(t, xd[0, 0:t.shape[0]], color='#C04747', lw=1.5, ls="--")
    state_2_1, = ax2.plot(t, xd[1, 0:t.shape[0]], color='#47C05B', lw=1.5, ls="--")
    state_3_1, = ax3.plot(t, xd[2, 0:t.shape[0]], color='#478DC0', lw=1.5, ls="--")
    # Add a legend
    ax1.legend([state_1, state_1_1],
                [r'x', r'x_d'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
             
    ax2.legend([state_2, state_2_1],
                [r'y', r'y_d'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)

    ax3.legend([state_3, state_3_1],
                [r'z', r'z_d'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
    
    ax1.grid(color='#949494', linestyle='-.', linewidth=0.8)
    ax2.grid(color='#949494', linestyle='-.', linewidth=0.8)
    ax3.grid(color='#949494', linestyle='-.', linewidth=0.8)
    
    ax3.set_xlabel(r"$Time}[s]$", labelpad=5)
    # Show the plot
    plt.show()

def plot_results_control(x, xp, t):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

    # Plot x_estimate with label 'x_estimate'
    ax1.set_xticklabels([])
    states_q = []
    states_qp = []
    size = x.shape[0]
    for k in range(0, size):
        aux_plot_1, = ax1.plot(t, x[k, 0:t.shape[0]],  lw=1.5, ls="-")
        aux_plot_2, = ax2.plot(t, xp[k, 0:t.shape[0]], lw=1.5, ls="-")
        states_q.append(aux_plot_1)
        states_qp.append(aux_plot_2)
    # Add a legend
    ax1.legend([states_q[0], states_q[1], states_q[2], states_q[3], states_q[4]],
                [r'q_1', r'q_2', r'q_3', r'q_4', r'q_5'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)

    ax2.legend([states_qp[0], states_qp[1], states_qp[2], states_qp[3], states_qp[4]],
                [r'qp_1', r'qp_2', r'qp_3', r'qp_4', r'qp_5'],
                loc="best",
                frameon=True, fancybox=True, shadow=False, ncol=2,
                borderpad=0.5, labelspacing=0.5, handlelength=3, handletextpad=0.1,
                borderaxespad=0.3, columnspacing=2)
    
    
    ax1.grid(color='#949494', linestyle='-.', linewidth=0.8)
    ax2.grid(color='#949494', linestyle='-.', linewidth=0.8)
    
    ax2.set_xlabel(r"$Time}[s]$", labelpad=5)
    # Show the plot
    plt.show()
if __name__ == '__main__':
    try:
        robot = Supervisor()
        main(robot)
        pass
    except(KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass
