from . import FlightDataHandler

import numpy as np
import scipy.signal as signal
import scipy.fft as fft


class ZAnalysis(FlightDataHandler):

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
    ### Override parent method to plot also z loop frequency spectrum
    def show_all(self):
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
        settle     = int(5/dt) #
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
        if not(hasattr(self, "behaviour")):
            self.analyse_z()
        return self.behaviour