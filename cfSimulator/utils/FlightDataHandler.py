import os
import time
import pickle as pk
import matplotlib.pyplot as plt
import numpy as np
import scipy.fft as fft
import scipy.signal as signal

# global variable: configuration
# should I remove intermediate files or not
# set to False to generate paper figures
remove_intermediate = True

# macros for plotting
chosen_size = (20, 7)
chosen_grid_linewidth = 0.3
chosen_grid_linestyle = '--'
chosen_grid_color = 'gray'

# encoding of behaviours -- TODO: define as enum (perhaps in separate file to import also in plot script)
bh_undefined       = 0
bh_good_tracking   = 1
bh_filtering       = 2
bh_good_tracking_extra = 3 
bh_sat_no_tracking = 4
bh_something_wrong = 5

class FlightDataHandler:
    data_directory = "flightdata"
    date_format    = '%Y%m%d_%H%M%S'

    def __init__(self):
        pass

    def save(self, name="no-name"):
        if name=="no-name" :
            # saves itself to file named as current date and time
            filename = time.strftime(self.date_format, time.localtime())
        else :
            # if provided, save with custom filename
            filename = name 
        with open(self.data_directory+"/"+filename, "wb") as f:
            pk.dump(self, f, protocol=pk.HIGHEST_PROTOCOL)

    def open(self, data_location, experiment_name, silent=False):
        self.data_location = data_location
        self.experiment_name = str(experiment_name)

        with open(self.data_location, 'rb') as f:
            data = pk.load(f)
        self.type = data.type
        self.unwrap(data)
        if not(silent):
            print('Reading data from file: \033[4m' + self.data_location + '\033[0m')
            print('Type of test is: ' + self.type)

    def unwrap(self, data):
        # splitting data into specific values
        self.time = data.t
        self.test = data.test
        self.trace_length = len(self.time)

        if not(self.type=='pitl'):
            self.position_x = data.pos[0, :]
            self.position_y = data.pos[1, :]
            self.position_z = data.pos[2, :]
        else:
            self.position_x = np.zeros(self.trace_length)
            self.position_y = np.zeros(self.trace_length)
            self.position_z = np.zeros(self.trace_length)
        self.estimated_position_x = data.est_pos[0, :]
        self.estimated_position_y = data.est_pos[1, :]
        self.estimated_position_z = data.est_pos[2, :]
        self.setpoint_position_x = data.set_pt[0, :]
        self.setpoint_position_y = data.set_pt[1, :]
        self.setpoint_position_z = data.set_pt[2, :]

        if not(self.type=='pitl'):
            self.velocity_x = data.vel[0, :]
            self.velocity_y = data.vel[1, :]
            self.velocity_z = data.vel[2, :]
        else:
            self.velocity_x = np.zeros(self.trace_length)
            self.velocity_y = np.zeros(self.trace_length)
            self.velocity_z = np.zeros(self.trace_length)
        self.estimated_velocity_x = data.est_vel[0, :]
        self.estimated_velocity_y = data.est_vel[1, :]
        self.estimated_velocity_z = data.est_vel[2, :]

        self.acceleration_x = data.acc[0, :]
        self.acceleration_y = data.acc[1, :]
        self.acceleration_z = data.acc[2, :]

        if not(self.type=='pitl'):
            self.attitude_x = data.eta[0, :]
            self.attitude_y = data.eta[1, :]
            self.attitude_z = data.eta[2, :]
        else:
            self.attitude_x = np.zeros(self.trace_length)
            self.attitude_y = np.zeros(self.trace_length)
            self.attitude_z = np.zeros(self.trace_length)
        self.gyro_x = data.gyro[0, :]
        self.gyro_y = data.gyro[1, :]
        self.gyro_z = data.gyro[2, :]

        self.pixel_count_x = data.pxCount[0, :]
        self.pixel_count_y = data.pxCount[1, :]
        self.range_z = data.zrange
        if not(self.type=='pitl'):
            self.kalman_error_x = data.err_fd[1, :]
            self.kalman_error_y = data.err_fd[2, :]
            self.kalman_error_z = data.err_fd[0, :]
        else:
            self.kalman_error_x = np.zeros(self.trace_length)
            self.kalman_error_y = np.zeros(self.trace_length)
            self.kalman_error_z = np.zeros(self.trace_length)

        self.control_motor_1 = data.u[0, :]
        self.control_motor_2 = data.u[1, :]
        self.control_motor_3 = data.u[2, :]
        self.control_motor_4 = data.u[3, :]

    ################################
    ### PDF GENERATION FUNCTIONS ###
    ################################

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
                                         str(self.position_x[i]) + ", " + \
                                         str(self.position_y[i]) + ", " + \
                                         str(self.position_z[i]) + ", " + \
                                         str(self.estimated_position_x[i]) + ", " + \
                                         str(self.estimated_position_y[i]) + ", " + \
                                         str(self.estimated_position_z[i]) + ", " + \
                                         str(self.setpoint_position_x[i]) + ", " + \
                                         str(self.setpoint_position_y[i]) + ", " + \
                                         str(self.setpoint_position_z[i]) + ", " + \
                                         str(self.velocity_x[i]) + ", " + \
                                         str(self.velocity_y[i]) + ", " + \
                                         str(self.velocity_z[i]) + ", " + \
                                         str(self.estimated_velocity_x[i]) + ", " + \
                                         str(self.estimated_velocity_y[i]) + ", " + \
                                         str(self.estimated_velocity_z[i]) + ", " + \
                                         str(self.acceleration_x[i]) + ", " + \
                                         str(self.acceleration_y[i]) + ", " + \
                                         str(self.acceleration_z[i]) + ", " + \
                                         str(self.attitude_x[i]) + ", " + \
                                         str(self.attitude_y[i]) + ", " + \
                                         str(self.attitude_z[i]) + ", " + \
                                         str(self.gyro_x[i]) + ", " + \
                                         str(self.gyro_y[i]) + ", " + \
                                         str(self.gyro_z[i]) + ", " + \
                                         str(self.pixel_count_x[i]) + ", " + \
                                         str(self.kalman_error_x[i]) + ", " + \
                                         str(self.pixel_count_y[i]) + ", " + \
                                         str(self.kalman_error_y[i]) + ", " + \
                                         str(self.range_z[i]) + ", " + \
                                         str(self.kalman_error_z[i]) + ", " + \
                                         str(self.control_motor_1[i]) + ", " + \
                                         str(self.control_motor_2[i]) + ", " + \
                                         str(self.control_motor_3[i]) + ", " + \
                                         str(self.control_motor_4[i]) + "\n"
                f.write(ith_string)

    def save_latex(self, latex_location, csv_location):
        with open(latex_location, 'w') as f:
            f.write(r'\documentclass{article}' + '\n')
            f.write(r'\usepackage[margin=2cm]{geometry}' + '\n')
            f.write(r'\usepackage{tikz, pgfplots}' + '\n')
            f.write(r'\usepgfplotslibrary{groupplots}' + '\n')
            f.write(r'\pgfplotsset{compat=1.17}' + '\n')
            f.write(r'\pagestyle{empty}' + '\n')
            f.write(r'\begin{document}' + '\n')
            f.write(r'\centering' + '\n')
            f.write(r'{\Large\verb|' + self.experiment_name + r'|}' + '\n')
            f.write(r'\resizebox{0.9\textwidth}{!}{%' + '\n')
            f.write(r'\begin{tikzpicture}[font=\small]' + '\n')
            f.write(r'\begin{groupplot}[%' + '\n')
            f.write(r'group style={group size = 2 by 8, vertical sep = 1cm, horizontal sep = 1cm},' + '\n')
            f.write(r'width = 10cm, height = 4cm,' + '\n')
            f.write(r'enlarge x limits = false,' + '\n')
            f.write(r'legend pos = south east,' + '\n')
            f.write(r'title style={yshift=-1.5ex},' + '\n')
            f.write(r'grid style={densely dotted, black!50},' + '\n')
            f.write(r'grid = major]' + '\n')
            f.write(r'\nextgroupplot[title = {Position $x$}, legend pos = north east]' + '\n')
            f.write(r'\addplot[ultra thick, black] table[col sep=comma, x index=1, y index=8] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{Setpoint $x$}' + '\n')
            if not(self.type=='pitl'):
                f.write(r'\addplot[ultra thick, blue] table[col sep=comma, x index=1, y index=2] {' + csv_location + r'};' + '\n')
                f.write(r'\addlegendentry{Position $x$}' + '\n')
            f.write(r'\addplot[ultra thick, blue, densely dotted] table[col sep=comma, x index=1, y index=5] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{Estimated $x$}' + '\n')
            f.write(r'\nextgroupplot[title = {Velocity $x$}]' + '\n')
            if not(self.type=='pitl'):
                f.write(r'\addplot[ultra thick, blue] table[col sep=comma, x index=1, y index=11] {' + csv_location + r'};' + '\n')
                f.write(r'\addlegendentry{Velocity $x$}' + '\n')
            f.write(r'\addplot[ultra thick, blue, densely dotted] table[col sep=comma, x index=1, y index=14] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{Estimated velocity $x$}' + '\n')
            f.write(r'\nextgroupplot[title = {Position $y$}]' + '\n')
            f.write(r'\addplot[ultra thick, black] table[col sep=comma, x index=1, y index=9] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{Setpoint $y$}' + '\n')
            if not(self.type=='pitl'):
                f.write(r'\addplot[ultra thick, green!60!black] table[col sep=comma, x index=1, y index=3] {' + csv_location + r'};' + '\n')
                f.write(r'\addlegendentry{Position $y$}' + '\n')
            f.write(r'\addplot[ultra thick, green!60!black, densely dotted] table[col sep=comma, x index=1, y index=6] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{Estimated $y$}' + '\n')
            f.write(r'\nextgroupplot[title = {Velocity $y$}, legend pos = north east]' + '\n')
            if not(self.type=='pitl'):
                f.write(r'\addplot[ultra thick, green!60!black] table[col sep=comma, x index=1, y index=12] {' + csv_location + r'};' + '\n')
                f.write(r'\addlegendentry{Velocity $y$}' + '\n')
            f.write(r'\addplot[ultra thick, green!60!black, densely dotted] table[col sep=comma, x index=1, y index=15] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{Estimated velocity $y$}' + '\n')
            f.write(r'\nextgroupplot[title = {Position $z$}]' + '\n')
            f.write(r'\addplot[ultra thick, black] table[col sep=comma, x index=1, y index=10] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{Setpoint $z$}' + '\n')
            if not(self.type=='pitl'):
                f.write(r'\addplot[ultra thick, red!80!black] table[col sep=comma, x index=1, y index=4] {' + csv_location + r'};' + '\n')
                f.write(r'\addlegendentry{Position $z$}' + '\n')
            f.write(r'\addplot[ultra thick, red!80!black, densely dotted] table[col sep=comma, x index=1, y index=7] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{Estimated $z$}' + '\n')
            f.write(r'\nextgroupplot[title = {Velocity $z$}, legend pos = north east]' + '\n')
            f.write(r'\addplot[ultra thick, red!80!black] table[col sep=comma, x index=1, y index=13] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{Velocity $z$}' + '\n')
            f.write(r'\addplot[ultra thick, red!80!black, densely dotted] table[col sep=comma, x index=1, y index=16] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{Estimated velocity $z$}' + '\n')
            f.write(r'\nextgroupplot[title = {Acceleration $x, y, z$}, legend columns = 3, legend pos = north east]' + '\n')
            f.write(r'\addplot[ultra thick, blue] table[col sep=comma, x index=1, y index=17] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$x$}' + '\n')
            f.write(r'\addplot[ultra thick, green!60!black] table[col sep=comma, x index=1, y index=18] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$y$}' + '\n')
            f.write(r'\addplot[ultra thick, red!80!black] table[col sep=comma, x index=1, y index=19] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$z$}' + '\n')
            f.write(r'\nextgroupplot[title = {Attitude $x, y, z$}, legend columns = 3, legend pos = north east]' + '\n')
            if not(self.type=='pitl'):
                f.write(r'\addplot[ultra thick, blue] table[col sep=comma, x index=1, y index=20] {' + csv_location + r'};' + '\n')
                f.write(r'\addlegendentry{$x$}' + '\n')
                f.write(r'\addplot[ultra thick, green!60!black] table[col sep=comma, x index=1, y index=21] {' + csv_location + r'};' + '\n')
                f.write(r'\addlegendentry{$y$}' + '\n')
                f.write(r'\addplot[ultra thick, red!80!black] table[col sep=comma, x index=1, y index=22] {' + csv_location + r'};' + '\n')
                f.write(r'\addlegendentry{$z$}' + '\n')
            f.write(r'\nextgroupplot[title = {Gyro $x, y, z$}, legend columns = 3, legend pos = north east]' + '\n')
            f.write(r'\addplot[ultra thick, blue] table[col sep=comma, x index=1, y index=23] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$x$}' + '\n')
            f.write(r'\addplot[ultra thick, green!60!black] table[col sep=comma, x index=1, y index=24] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$y$}' + '\n')
            f.write(r'\addplot[ultra thick, red!80!black] table[col sep=comma, x index=1, y index=25] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$z$}' + '\n')
            f.write(r'\nextgroupplot[title = {$z$-range}]' + '\n')
            f.write(r'\addplot[ultra thick, red!80!black] table[col sep=comma, x index=1, y index=30] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$z$-range}' + '\n')
            f.write(r'\nextgroupplot[title = {Pixel count $x,y$}, legend pos = north east, legend columns = 2]' + '\n')
            f.write(r'\addplot[ultra thick, blue] table[col sep=comma, x index=1, y index=26] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$x$}' + '\n')
            f.write(r'\addplot[ultra thick, green!60!black] table[col sep=comma, x index=1, y index=28] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$y$}' + '\n')
            f.write(r'\nextgroupplot[title = {Kalman errors $x,y,z$}, legend columns = 3, legend pos = north east]' + '\n')
            if not(self.type=='pitl'):
                f.write(r'\addplot[ultra thick, blue, densely dotted] table[col sep=comma, x index=1, y index=27] {' + csv_location + r'};' + '\n')
                f.write(r'\addlegendentry{$x$}' + '\n')
                f.write(r'\addplot[ultra thick, green!60!black, densely dotted] table[col sep=comma, x index=1, y index=29] {' + csv_location + r'};' + '\n')
                f.write(r'\addlegendentry{$y$}' + '\n')
                f.write(r'\addplot[ultra thick, red!80!black] table[col sep=comma, x index=1, y index=31] {' + csv_location + r'};' + '\n')
                f.write(r'\addlegendentry{$z$}' + '\n')
            f.write(r'\nextgroupplot[title = {Motors control signals $u_1$}]' + '\n')
            f.write(r'\addplot[ultra thick, black] table[col sep=comma, x index=1, y index=32] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$u_1$}' + '\n')
            f.write(r'\nextgroupplot[title = {Motors control signals $u_2$}]' + '\n')
            f.write(r'\addplot[ultra thick, black!80] table[col sep=comma, x index=1, y index=33] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$u_2$}' + '\n')
            f.write(r'\nextgroupplot[title = {Motors control signals $u_3$}]' + '\n')
            f.write(r'\addplot[ultra thick, black!60] table[col sep=comma, x index=1, y index=34] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$u_3$}' + '\n')
            f.write(r'\nextgroupplot[title = {Motors control signals $u_4$}]' + '\n')
            f.write(r'\addplot[ultra thick, black!40] table[col sep=comma, x index=1, y index=35] {' + csv_location + r'};' + '\n')
            f.write(r'\addlegendentry{$u_4$}' + '\n')
            f.write(r'\end{groupplot}' + '\n')
            f.write(r'\end{tikzpicture}' + '\n')
            f.write(r'}' + '\n')
            f.write(r'\end{document}' + '\n')

    def pdf(self, experiment_name):
        # first save the data to an intermediate csv file
        intermediate_data_file = "data.csv"
        intermediate_latex_file = experiment_name + ".tex"
        self.save_csv(intermediate_data_file)
        print('* saved data to intermediate file: \033[33m' + str(intermediate_data_file) + '\033[0m')

        # now do the plotting
        self.save_latex(intermediate_latex_file, intermediate_data_file)
        print('* saved tex to intermediate file: \033[33m' + str(intermediate_latex_file) + '\033[0m')

        # you want to use lualatex because there could be a lot of data
        os.system("lualatex " + intermediate_latex_file + " &>/dev/null")
        print('* compiled latex plot: \033[33m' + str(experiment_name) + '.pdf\033[0m')

        # then remove intermediate files
        if remove_intermediate:
            try:
                os.remove(intermediate_data_file)
                print('* removed intermediate file: \033[33m' + str(intermediate_data_file) + '\033[0m')
                os.remove(intermediate_latex_file)
                os.remove(experiment_name + ".aux")
                os.remove(experiment_name + ".log")
                print('* removed intermediate file: \033[33m' + str(intermediate_latex_file) + '\033[0m + .aux .log')
            except OSError:
                pass

    ##########################
    ### PLOTTING FUNCTIONS ###
    ##########################

    def trajectoryPlot(self):
        if not(self.type=='pitl'):# plot trajectory in space
            plt.figure('3D trajectory')
            ax = plt.axes(projection="3d", label="uniquelabel")
            ax.plot(self.position_x, self.position_y, self.position_z, 'r', label="position")
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            print('* figure 1:\033[33m 3d position\033[0m')
        else :
            # plot trajectory in space
            plt.figure('estimated 3D trajectory')
            ax = plt.axes(projection="3d", label="uniquelabel")
            ax.plot(self.estimated_position_x, self.estimated_position_y, self.estimated_position_z, 'r', label="position")
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            print('* figure 1:\033[33m 3d position\033[0m')

    def positionSpeedPlot(self):

        fig, axs = plt.subplots(3, 2, figsize=chosen_size)
        plt.subplots_adjust(wspace=0.2, hspace=1)

        axs[0, 0].title.set_text('Position (x)')
        axs[0, 0].plot(self.time, self.setpoint_position_x, 'k')
        if not(self.type=='pitl'):
            axs[0, 0].plot(self.time, self.position_x, 'b')
        axs[0, 0].plot(self.time, self.estimated_position_x, 'b:')
        axs[0, 0].legend(['setpoint', 'position', 'estimated position'])
        axs[0, 0].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        axs[1, 0].title.set_text('Position (y)')
        axs[1, 0].plot(self.time, self.setpoint_position_y, 'k')
        if not(self.type=='pitl'):
            axs[1, 0].plot(self.time, self.position_y, 'g')
        axs[1, 0].plot(self.time, self.estimated_position_y, 'g:')
        axs[1, 0].legend(['setpoint', 'position', 'estimated position'])
        axs[1, 0].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        axs[2, 0].title.set_text('Position (z)')
        axs[2, 0].plot(self.time, self.setpoint_position_z, 'k')
        if not(self.type=='pitl'):
            axs[2, 0].plot(self.time, self.position_z, 'r')
        axs[2, 0].plot(self.time, self.estimated_position_z, 'r:')
        axs[2, 0].legend(['setpoint', 'position', 'estimated position'])
        axs[2, 0].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        axs[0, 1].title.set_text('Velocity (x)')
        if not(self.type=='pitl'):
            axs[0, 1].plot(self.time, self.velocity_x, 'b')
        axs[0, 1].plot(self.time, self.estimated_velocity_x, 'b:')
        axs[0, 1].legend(['velocity', 'estimated velocity'])
        axs[0, 1].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        axs[1, 1].title.set_text('Velocity (y)')
        if not(self.type=='pitl'):
            axs[1, 1].plot(self.time, self.velocity_y, 'g')
        axs[1, 1].plot(self.time, self.estimated_velocity_y, 'g:')
        axs[1, 1].legend(['velocity', 'estimated velocity'])
        axs[1, 1].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        axs[2, 1].title.set_text('Velocity (z)')
        if not(self.type=='pitl'):
            axs[2, 1].plot(self.time, self.velocity_z, 'r')
        axs[2, 1].plot(self.time, self.estimated_velocity_z, 'r:')
        axs[2, 1].legend(['velocity', 'estimated velocity'])
        axs[2, 1].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)
        print('* figure 2:\033[33m position and velocity (x,y,z)\033[0m')

    def sensorReadingsPlot(self):

        fig, axs = plt.subplots(3, 2, figsize=chosen_size)
        plt.subplots_adjust(wspace=0.2, hspace=1)

        axs[0, 0].title.set_text('Acceleration (x,y,z)')
        axs[0, 0].plot(self.time, self.acceleration_x, 'b')
        axs[0, 0].plot(self.time, self.acceleration_y, 'g')
        axs[0, 0].plot(self.time, self.acceleration_z, 'r')
        axs[0, 0].legend(['x', 'y', 'z'])
        axs[0, 0].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        if not(self.type=='pitl'):
            axs[0, 1].title.set_text('Attitude (x,y,z)')
            axs[0, 1].plot(self.time, self.attitude_x, 'b')
            axs[0, 1].plot(self.time, self.attitude_y, 'g')
            axs[0, 1].plot(self.time, self.attitude_z, 'r')
            axs[0, 1].legend(['x', 'y', 'z'])
            axs[0, 1].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        axs[1, 0].title.set_text('Gyro (x,y,z)')
        axs[1, 0].plot(self.time, self.gyro_x, 'b')
        axs[1, 0].plot(self.time, self.gyro_y, 'g')
        axs[1, 0].plot(self.time, self.gyro_z, 'r')
        axs[1, 0].legend(['x', 'y', 'z'])
        axs[1, 0].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        axs[1, 1].title.set_text('z-range')
        axs[1, 1].plot(self.time, self.range_z, 'r')
        axs[1, 1].legend(['z'])
        axs[1, 1].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        axs[2, 0].title.set_text('Pixel count (x,y)')
        axs[2, 0].plot(self.time, self.pixel_count_x, 'b')
        axs[2, 0].plot(self.time, self.pixel_count_y, 'g')
        axs[2, 0].legend(['x', 'y'])
        axs[2, 0].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        if not(self.type=='pitl'):
            axs[2, 1].title.set_text('Kalman errors (x,y,z)')
            axs[2, 1].plot(self.time, self.kalman_error_x, 'b')
            axs[2, 1].plot(self.time, self.kalman_error_y, 'g')
            axs[2, 1].plot(self.time, self.kalman_error_z, 'r')
            axs[2, 1].legend(['x', 'y', 'z'])
            axs[2, 1].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)
        print('* figure 3:\033[33m sensor data and Kalman errors\033[0m')

    def controlActionPlot(self):

        fig, axs = plt.subplots(2, 2, figsize=chosen_size)
        plt.subplots_adjust(wspace=0.2, hspace=1)

        axs[0, 0].title.set_text('Motor control signals (u1)')
        axs[0, 0].plot(self.time, self.control_motor_1, 'k')
        axs[0, 0].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        axs[0, 1].title.set_text('Motor control signals (u2)')
        axs[0, 1].plot(self.time, self.control_motor_2, 'dimgray')
        axs[0, 1].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        axs[1, 0].title.set_text('Motor control signals (u3)')
        axs[1, 0].plot(self.time, self.control_motor_3, 'darkgray')
        axs[1, 0].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

        axs[1, 1].title.set_text('Motor control signals (u4)')
        axs[1, 1].plot(self.time, self.control_motor_4, 'lightgray')
        axs[1, 1].grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)
        print('* figure 4:\033[33m motor control signals (u1,u2,u3,u4)\033[0m')

    def z_loop_frequency_plot(self):
        self.analyse_z() # needed to get the frequency spectrum
        fig, axs = plt.subplots(1, 1, figsize=chosen_size)
        plt.subplots_adjust(wspace=0.2, hspace=1)
        min_freq_plot = 0
        max_freq_plot = 6
        axs.title.set_text('Frequency spectrum of z reference and z position')
        axs.plot(self.z_fft_freq, self.z_ref_fft, 'k')
        axs.plot(self.z_fft_freq, self.z_pos_fft, 'red')
        axs.set_xlim([min_freq_plot, max_freq_plot])
        axs.grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

    def show(self):
        self.trajectoryPlot()
        self.positionSpeedPlot()
        self.sensorReadingsPlot()
        self.controlActionPlot()
        self.z_loop_frequency_plot()
        plt.show()

    ##########################
    ### ANALYSIS FUNCTIONS ###
    ##########################

    def analyse_z(self):
        err_rel     = 0  # error normalized over setpoint
        err_abs_cum = 0 
        mot_sat_tot = 0  # total time motors are saturated [# of time steps]
        hit_ground  = 0  # total time spent "hitting the ground" [# of time steps]
        max_error   = 0

        ### PARAMETERS -- TODO should be defined elsewhere
        dt = 0.001         # sampling time in seconds
        thrust_min = 20000 # saturation limits of motors
        thrust_max = 65535 # 
        settle     = int(10/dt) # 
        # time in percentage of test trace length above which the drone is considered to have hit the saturations
        motors_saturated_threshold = 0.001 
        # deviation from expected gain of 1 that is accepted for good reference tracking
        filtering_threshold = 0.2
        avg_error_rel_threshold = 0.5
        ### END PARAMETERS

        if (self.test=="sinus"):
            # if test was a sinus remove the warm up in the first 5 seconds
            time_range = range(settle,self.trace_length)
        else :
            # otherwise include the whole test
            time_range = range(1,self.trace_length)
        
        # TIME DOM. ANALYSIS: iterate over time and perform actual analysis
        for i in time_range:
            abs_error = np.abs(self.setpoint_position_z[i] - self.position_z[i])
            if abs_error>max_error :
                max_error = abs_error
            err_rel = err_rel + abs_error/self.setpoint_position_z[i]
            err_abs_cum = err_abs_cum + abs_error
            mot_sat_tot = mot_sat_tot + \
                          (self.control_motor_1[i]<thrust_min+1 or\
                           self.control_motor_1[i]>thrust_max-1)
            hit_ground  = hit_ground + (self.position_z[i]<0.01)
        
        # FREQ. DOM. ANALYSIS
        # detrend signals (otherwise 0-freq component hides everything)
        z_ref_detrended = signal.detrend(self.setpoint_position_z[settle:self.trace_length],type='constant')
        z_pos_detrended = signal.detrend(self.position_z[settle:self.trace_length],type='constant')
        # fft computation
        z_ref_fft  = list(map(abs, fft.fft(z_ref_detrended)))
        z_pos_fft  = list(map(abs, fft.fft(z_pos_detrended)))
        z_fft_freq = fft.fftfreq((self.trace_length-settle), d=dt)
        # spectrum is symmetric
        self.z_ref_fft = z_ref_fft[:len(z_ref_fft)//2]
        self.z_pos_fft = z_pos_fft[:len(z_pos_fft)//2]
        self.z_fft_freq = z_fft_freq[:len(z_fft_freq)//2]
        
        # finalize analysis
        self.z_avg_error_abs        = err_abs_cum/(self.trace_length-settle)
        self.z_avg_error_rel        = err_rel/(self.trace_length-settle)
        self.z_max_error            = max_error
        # compute saturation and ground time as percentage of total test time
        self.motors_saturated_time  = mot_sat_tot/(self.trace_length-settle)
        self.hit_ground_time        = hit_ground/(self.trace_length-settle)
        if (self.test=="sinus"):
            # check for filtering of reference's frequency with largest component
            index_input_freq = np.where(self.z_ref_fft== np.amax(self.z_ref_fft))
            self.z_filtering =  self.z_pos_fft[index_input_freq[0][0]]\
                                /self.z_ref_fft[index_input_freq[0][0]]

        # define behavioural region on the base of: 
        #  - hitting saturations
        #  - good reference tracking
        #  - filtering reference
        self.behaviour = bh_undefined
        if (self.test == "sinus"):
            if self.motors_saturated_time<motors_saturated_threshold : 
                # we have not saturated
                if abs(self.z_filtering-1)>filtering_threshold :
                    self.behaviour = bh_filtering
                else :
                    if self.z_avg_error_rel>avg_error_rel_threshold :
                        self.behaviour = bh_something_wrong
                    else :
                        self.behaviour = bh_good_tracking
            else :
                # we have saturated
                if self.z_avg_error_rel<avg_error_rel_threshold :
                    self.behaviour = bh_good_tracking_extra
                else :
                    self.behaviour = bh_sat_no_tracking

    def compute_z_avg_error_abs(self):
        if not(hasattr(self, "z_avg_error_abs")):
            self.analyse_z()
        return self.z_avg_error_abs

    def compute_z_avg_error_rel(self):
        if not(hasattr(self, "z_avg_error_rel")):
            self.analyse_z()
        return self.z_avg_error_rel

    def compute_z_max_error(self):
        if not(hasattr(self, "z_max_error")):
            self.analyse_z()
        return self.z_max_error

    def motors_saturated(self):
        # return the number of time steps in which the controller was saturated
        if not(hasattr(self, "motors_saturated_time")):
            self.analyse_z()
        return self.motors_saturated_time

    def hit_ground(self):
        if not(hasattr(self, "hit_ground_time")):
            self.analyse_z()
        return self.hit_ground_time

    def compute_z_filtering(self):
        if self.test!="sinus":
            return 0
        if not(hasattr(self, "z_filtering")):
            self.analyse_z()
        return self.z_filtering

    def compute_behaviour(self):
        if self.test!="sinus":
            return bh_undefined
        if not(hasattr(self, "behaviour")):
            self.analyse_z()
        return self.behaviour