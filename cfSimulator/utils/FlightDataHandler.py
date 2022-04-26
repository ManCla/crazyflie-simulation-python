import time
import pickle as pk
import matplotlib.pyplot as plt
import numpy as np

class FlightDataHandler:
    data_directory = "flightdata"
    date_format    = '%Y%m%d_%H%M%S'

    # class variables for plotting aesthetics
    chosen_size = (20, 7)
    chosen_grid_linewidth = 0.3
    chosen_grid_linestyle = '--'
    chosen_grid_color = 'gray'

    def __init__(self):
        pass

    # method called by the Simulation object to store the data
    # in the FlightDataHandler object before saving
    def store(self, trajectoryType, t, x, u, eta, acc, px_count, set_pt,\
              z_range, kal_err_fd, x_est):
        self.test    = trajectoryType
        self.time    = t
        # extract states
        self.pos     = x[0:3,:]
        self.vel     = x[3:6,:]
        self.gyro    = x[10:13,:]
        self.eta     = eta
        # control action, measurements, and other cf data
        self.u          = u
        self.acc        = acc
        self.px_count   = px_count
        self.set_pt     = set_pt
        self.z_range    = z_range
        self.kal_err_fd = kal_err_fd
        self.est_pos    = x_est[0:3,:]
        self.est_vel    = x_est[3:6,:]
        self.est_eta    = x_est[6:9,:]
        # no need to retrieve trace length each time it is needed
        self.trace_length = len(self.time)

    def save(self, name="no-name"):
        if name=="no-name" :
            # saves itself to file named as current date and time
            filename = time.strftime(self.date_format, time.localtime())
        else :
            # if provided, save with custom filename
            filename = name 
        with open(self.data_directory+"/"+filename, "wb") as f:
            pk.dump(self, f, protocol=pk.HIGHEST_PROTOCOL)

    def open(self, data_location, silent=False):
        self.data_location = data_location

        with open(self.data_location, 'rb') as f:
            data = pk.load(f)
        if not(silent):
            print('Read data from file: \033[4m' + self.data_location + '\033[0m')
        self.test         = data.test
        self.time         = data.time
        self.pos          = data.pos
        self.vel          = data.vel
        self.gyro         = data.gyro
        self.eta          = data.eta
        self.u            = data.u
        self.acc          = data.acc
        self.px_count     = data.px_count
        self.set_pt       = data.set_pt
        self.z_range      = data.z_range
        self.kal_err_fd   = data.kal_err_fd
        self.est_pos      = data.est_pos
        self.est_vel      = data.est_vel
        self.est_eta      = data.est_eta
        self.trace_length = data.trace_length

    ###############################
    ### CSV GENERATION FUNCTION ###
    ###############################

    def save_csv(self, csv_location):

        with open(csv_location, 'w') as f:

            # header for file
            f.write('i, time, ' +
                            'position_x, position_y, position_z, ' +
                            'estimated_position_x, estimated_position_y, estimated_position_z, ' +
                            'setpoint_position_x, setpoint_position_y, setpoint_position_z, ' +
                            'velocity_x, velocity_y, velocity_z, ' +
                            'estimated_velocity_x, estimated_velocity_y, estimated_velocity_z, ' +
                            'acceleration_x, acceleration_y, acceleration_z, ' +
                            'attitude_x, attitude_y, attitude_z, ' +
                            'gyro_x, gyro_y, gyro_z, ' +
                            'pixel_count_x, kalman_error_x, ' +
                            'pixel_count_y, kalman_error_y, ' +
                            'range_z, kalman_error_z, ' +
                            'control_motor_1, control_motor_2, control_motor_3, control_motor_4 \n')

            for i in range(self.trace_length):
                ith_string = str(i) + ", " + \
                                         str(self.time[i]) + ", " + \
                                         str(self.pos[0, i]) + ", " + \
                                         str(self.pos[1, i]) + ", " + \
                                         str(self.pos[2, i]) + ", " + \
                                         str(self.est_pos[0, i]) + ", " + \
                                         str(self.est_pos[1, i]) + ", " + \
                                         str(self.est_pos[2, i]) + ", " + \
                                         str(self.set_pt[0, i]) + ", " + \
                                         str(self.set_pt[1, i]) + ", " + \
                                         str(self.set_pt[2, i]) + ", " + \
                                         str(self.vel[0, i]) + ", " + \
                                         str(self.vel[1, i]) + ", " + \
                                         str(self.vel[2, i]) + ", " + \
                                         str(self.est_vel[0, i]) + ", " + \
                                         str(self.est_vel[1, i]) + ", " + \
                                         str(self.est_vel[2, i]) + ", " + \
                                         str(self.acc[0, i]) + ", " + \
                                         str(self.acc[1, i]) + ", " + \
                                         str(self.acc[2, i]) + ", " + \
                                         str(self.eta[0, i]) + ", " + \
                                         str(self.eta[1, i]) + ", " + \
                                         str(self.eta[2, i]) + ", " + \
                                         str(self.gyro[0, i]) + ", " + \
                                         str(self.gyro[1, i]) + ", " + \
                                         str(self.gyro[2, i]) + ", " + \
                                         str(self.px_count[0, i]) + ", " + \
                                         str(self.kal_err_fd[0, i]) + ", " + \
                                         str(self.px_count[1, i]) + ", " + \
                                         str(self.kal_err_fd[1,i]) + ", " + \
                                         str(self.z_range[i]) + ", " + \
                                         str(self.kal_err_fd[2, i]) + ", " + \
                                         str(self.u[0, i]) + ", " + \
                                         str(self.u[1, i]) + ", " + \
                                         str(self.u[2, i]) + ", " + \
                                         str(self.u[3, i]) + "\n"
                f.write(ith_string)

    ##########################
    ### PLOTTING FUNCTIONS ###
    ##########################

    def trajectoryPlot(self):
        plt.figure('3D trajectory')
        ax = plt.axes(projection="3d", label="uniquelabel")
        ax.plot(self.pos[0,:], self.pos[1,:], self.pos[2,:], 'r', label="position")
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        print('* figure 1:\033[33m 3d position\033[0m')

    def positionSpeedPlot(self):

        fig, axs = plt.subplots(3, 2, figsize=self.chosen_size)
        plt.subplots_adjust(wspace=0.2, hspace=1)

        axs[0, 0].title.set_text('Position (x)')
        axs[0, 0].plot(self.time, self.set_pt[0,:], 'k')
        axs[0, 0].plot(self.time, self.pos[0,:], 'b')
        axs[0, 0].plot(self.time, self.est_pos[0,:], 'b:')
        axs[0, 0].legend(['setpoint', 'position', 'estimated position'])
        axs[0, 0].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[1, 0].title.set_text('Position (y)')
        axs[1, 0].plot(self.time, self.set_pt[1,:], 'k')
        axs[1, 0].plot(self.time, self.pos[1,:], 'g')
        axs[1, 0].plot(self.time, self.est_pos[1,:], 'g:')
        axs[1, 0].legend(['setpoint', 'position', 'estimated position'])
        axs[1, 0].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[2, 0].title.set_text('Position (z)')
        axs[2, 0].plot(self.time, self.set_pt[2,:], 'k')
        axs[2, 0].plot(self.time, self.pos[2,:], 'r')
        axs[2, 0].plot(self.time, self.est_pos[2,:], 'r:')
        axs[2, 0].legend(['setpoint', 'position', 'estimated position'])
        axs[2, 0].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[0, 1].title.set_text('Velocity (x)')
        axs[0, 1].plot(self.time, self.vel[0,:], 'b')
        axs[0, 1].plot(self.time, self.est_vel[0,:], 'b:')
        axs[0, 1].legend(['velocity', 'estimated velocity'])
        axs[0, 1].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[1, 1].title.set_text('Velocity (y)')
        axs[1, 1].plot(self.time, self.vel[1,:], 'g')
        axs[1, 1].plot(self.time, self.est_vel[1,:], 'g:')
        axs[1, 1].legend(['velocity', 'estimated velocity'])
        axs[1, 1].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[2, 1].title.set_text('Velocity (z)')
        axs[2, 1].plot(self.time, self.vel[2,:], 'r')
        axs[2, 1].plot(self.time, self.est_vel[2,:], 'r:')
        axs[2, 1].legend(['velocity', 'estimated velocity'])
        axs[2, 1].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)
        print('* figure 2:\033[33m position and velocity (x,y,z)\033[0m')

    def sensorReadingsPlot(self):

        fig, axs = plt.subplots(3, 2, figsize=self.chosen_size)
        plt.subplots_adjust(wspace=0.2, hspace=1)

        axs[0, 0].title.set_text('Acceleration (x,y,z)')
        axs[0, 0].plot(self.time, self.acc[0,:], 'b')
        axs[0, 0].plot(self.time, self.acc[1,:], 'g')
        axs[0, 0].plot(self.time, self.acc[2,:], 'r')
        axs[0, 0].legend(['x', 'y', 'z'])
        axs[0, 0].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[0, 1].title.set_text('Attitude (x,y,z)')
        axs[0, 1].plot(self.time, self.eta[0,:], 'b')
        axs[0, 1].plot(self.time, self.eta[1,:], 'g')
        axs[0, 1].plot(self.time, self.eta[2,:], 'r')
        axs[0, 1].legend(['x', 'y', 'z'])
        axs[0, 1].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[1, 0].title.set_text('Gyro (x,y,z)')
        axs[1, 0].plot(self.time, self.gyro[0,:], 'b')
        axs[1, 0].plot(self.time, self.gyro[1,:], 'g')
        axs[1, 0].plot(self.time, self.gyro[2,:], 'r')
        axs[1, 0].legend(['x', 'y', 'z'])
        axs[1, 0].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[1, 1].title.set_text('z-range')
        axs[1, 1].plot(self.time, self.z_range, 'r')
        axs[1, 1].legend(['z'])
        axs[1, 1].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[2, 0].title.set_text('Pixel count (x,y)')
        axs[2, 0].plot(self.time, self.px_count[0,:], 'b')
        axs[2, 0].plot(self.time, self.px_count[1,:], 'g')
        axs[2, 0].legend(['x', 'y'])
        axs[2, 0].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[2, 1].title.set_text('Kalman errors (x,y,z)')
        axs[2, 1].plot(self.time, self.kal_err_fd[0,:], 'b')
        axs[2, 1].plot(self.time, self.kal_err_fd[1,:], 'g')
        axs[2, 1].plot(self.time, self.kal_err_fd[2,:], 'r')
        axs[2, 1].legend(['x', 'y', 'z'])
        axs[2, 1].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)
        print('* figure 3:\033[33m sensor data and Kalman errors\033[0m')

    def controlActionPlot(self):

        fig, axs = plt.subplots(2, 2, figsize=self.chosen_size)
        plt.subplots_adjust(wspace=0.2, hspace=1)

        axs[0, 0].title.set_text('Motor control signals (u1)')
        axs[0, 0].plot(self.time, self.u[0,:], 'k')
        axs[0, 0].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[0, 1].title.set_text('Motor control signals (u2)')
        axs[0, 1].plot(self.time, self.u[1,:], 'dimgray')
        axs[0, 1].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[1, 0].title.set_text('Motor control signals (u3)')
        axs[1, 0].plot(self.time, self.u[2,:], 'darkgray')
        axs[1, 0].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)

        axs[1, 1].title.set_text('Motor control signals (u4)')
        axs[1, 1].plot(self.time, self.u[3,:], 'lightgray')
        axs[1, 1].grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)
        print('* figure 4:\033[33m motor control signals (u1,u2,u3,u4)\033[0m')

    ### function to generate and show all plots available
    def show_all(self):
        self.trajectoryPlot()
        self.positionSpeedPlot()
        self.sensorReadingsPlot()
        self.controlActionPlot()
        plt.show()

    ### function to show only the already generated selected plots
    # note that caller should first generate the desired plots
    # with the dedicated functions
    def show(self):
        plt.show()
