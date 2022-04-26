import os
import time
import pickle as pk
import matplotlib.pyplot as plt
import numpy as np
import scipy.fft as fft
import scipy.signal as signal

class FlightDataHandler:
    data_directory = "flightdata"
    date_format    = '%Y%m%d_%H%M%S'

    # class variables for plotting aesthetics
    chosen_size = (20, 7)
    chosen_grid_linewidth = 0.3
    chosen_grid_linestyle = '--'
    chosen_grid_color = 'gray'

    # encoding of behaviours -- TODO: define as enum
    bh_undefined       = 0
    bh_good_tracking   = 1
    bh_filtering       = 2
    bh_good_tracking_extra = 3 
    bh_sat_no_tracking = 4
    bh_something_wrong = 5

    # colour palette for behaviour coding
    bh_palette = np.array([[255,255,255],  # black: bh_undefined
                           [  0,255,  0],  # green: bh_good_tracking
                           [  0,  0,255],  # blue : bh_filtering
                           [  0,255,255],  # azure: bh_good_tracking_extra
                           [255,  0,  0],  # red  : bh_sat_no_tracking
                           [  0,  0,  0]]) # dark : bh_something_wrong

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
        axs[1, 1].plot(self.time, self.vel[2,:], 'g')
        axs[1, 1].plot(self.time, self.est_vel[2,:], 'g:')
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

    def z_loop_frequency_plot(self):
        self.analyse_z() # needed to get the frequency spectrum
        fig, axs = plt.subplots(1, 1, figsize=self.chosen_size)
        plt.subplots_adjust(wspace=0.2, hspace=1)
        min_freq_plot = 0
        max_freq_plot = 6
        axs.title.set_text('Frequency spectrum of z reference and z position')
        axs.plot(self.z_fft_freq, self.z_ref_fft, 'k')
        axs.plot(self.z_fft_freq, self.z_pos_fft, 'red')
        axs.set_xlim([min_freq_plot, max_freq_plot])
        axs.grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)
        print('* figure 5:\033[33m frequency spectrum of z reference and position\033[0m')

    ### function to generate and show all plots available
    def show_all(self):
        self.trajectoryPlot()
        self.positionSpeedPlot()
        self.sensorReadingsPlot()
        self.controlActionPlot()
        self.z_loop_frequency_plot()
        plt.show()

    ### function to show only the already generated selected plots
    # note that caller should first generate the desired plots
    # with the dedicated functions
    def show(self):
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
            abs_error = np.abs(self.set_pt[2,i] - self.pos[2,:][i])
            if abs_error>max_error :
                max_error = abs_error
            err_rel = err_rel + abs_error/self.set_pt[2,i]
            err_abs_cum = err_abs_cum + abs_error
            mot_sat_tot = mot_sat_tot + \
                          (self.u[0,i]<thrust_min+1 or\
                           self.u[0,i]>thrust_max-1)
            hit_ground  = hit_ground + (self.pos[2,:][i]<0.01)
        
        # FREQ. DOM. ANALYSIS
        # detrend signals (otherwise 0-freq component hides everything)
        if (self.test=="sinus"):
            # if test is a sinus test remove warm up
            z_ref_detrended = signal.detrend(self.set_pt[2,settle:self.trace_length],type='constant')
            z_pos_detrended = signal.detrend(self.pos[2,:][settle:self.trace_length],type='constant')
            z_fft_freq = fft.fftfreq((self.trace_length-settle), d=dt)
        else :
            # otherwise use the whole trace
            z_ref_detrended = signal.detrend(self.set_pt[2,:],type='constant')
            z_pos_detrended = signal.detrend(self.pos[2,:],type='constant')
            z_fft_freq = fft.fftfreq((self.trace_length), d=dt)
        # fft computation
        z_ref_fft  = list(map(abs, fft.fft(z_ref_detrended)))
        z_pos_fft  = list(map(abs, fft.fft(z_pos_detrended)))
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
        self.behaviour = self.bh_undefined
        if (self.test == "sinus"):
            if self.motors_saturated_time<motors_saturated_threshold : 
                # we have not saturated
                if abs(self.z_filtering-1)>filtering_threshold :
                    self.behaviour = self.bh_filtering
                else :
                    if self.z_avg_error_rel>avg_error_rel_threshold :
                        self.behaviour = self.bh_something_wrong
                    else :
                        self.behaviour = self.bh_good_tracking
            else :
                # we have saturated
                if self.z_avg_error_rel<avg_error_rel_threshold :
                    self.behaviour = self.bh_good_tracking_extra
                else :
                    self.behaviour = self.bh_sat_no_tracking

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
            return self.bh_undefined
        if not(hasattr(self, "behaviour")):
            self.analyse_z()
        return self.behaviour