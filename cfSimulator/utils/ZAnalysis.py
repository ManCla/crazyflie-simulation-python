from . import FlightDataHandler

import numpy as np
import scipy.signal as signal
import scipy.fft as fft
import matplotlib.pyplot as plt

class ZAnalysis(FlightDataHandler):

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

    def z_loop_frequency_plot(self):
        if not(hasattr(self, "z_fft_freq")):
            self.analyse_z() # needed to get the frequency spectrum
        fig, axs = plt.subplots(1, 1, figsize=self.chosen_size)
        plt.subplots_adjust(wspace=0.2, hspace=1)
        min_freq_plot = 0
        max_freq_plot = 3
        axs.title.set_text('Frequency spectrum of z reference and z position')
        axs.plot(self.z_fft_freq, self.z_ref_fft, 'k')
        axs.plot(self.z_fft_freq, self.z_pos_fft, 'red')
        axs.plot(self.z_fft_freq, self.z_err_fft, 'green')
        axs.legend(['reference','position','error'])
        axs.set_xlim([min_freq_plot, max_freq_plot])
        axs.grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)
        print('* figure 5:\033[33m frequency spectrum of z reference and position\033[0m')

    ### function to generate and show all plots available
    ### Override parent method to plot also z loop frequency spectrum
    def show_all(self):
        self.positionSpeedPlot()
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

        # remove the warm up (defined by settle variable)
        time_range = range(settle,self.trace_length)
        
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
        z_err_detrended = signal.detrend(self.set_pt[2,settle:self.trace_length]\
                                        -self.pos[2,:][settle:self.trace_length],type='constant')
        z_ref_detrended = signal.detrend(self.set_pt[2,settle:self.trace_length],type='constant')
        z_pos_detrended = signal.detrend(self.pos[2,:][settle:self.trace_length],type='constant')
        z_fft_freq = fft.fftfreq((self.trace_length-settle), d=dt)

        # fft computation
        z_err_fft  = list(map(abs, fft.fft(z_err_detrended)))
        z_ref_fft  = list(map(abs, fft.fft(z_ref_detrended)))
        z_pos_fft  = list(map(abs, fft.fft(z_pos_detrended)))
        # spectrum is symmetric
        self.z_err_fft = z_err_fft[:len(z_err_fft)//2]
        self.z_ref_fft = z_ref_fft[:len(z_ref_fft)//2]
        self.z_pos_fft = z_pos_fft[:len(z_pos_fft)//2]
        self.z_fft_freq = z_fft_freq[:len(z_fft_freq)//2]

        # find peaks in spectrum
        # TODO --- think of different possible definitions of peaks in spectrum
        # TODO --- could consider also filtering peaks that are too high in freq
        peaks, _ = signal.find_peaks(self.z_ref_fft, height=0.1*max(self.z_ref_fft))
        self.z_fft_freq_peaks = np.array(self.z_fft_freq)[peaks]
        self.z_ref_fft_peaks = np.array(self.z_ref_fft)[peaks]
        # TODO fin frequency peaks on output?
        
        # finalize analysis
        self.z_avg_error_abs        = err_abs_cum/(self.trace_length-settle)
        self.z_avg_error_rel        = err_rel/(self.trace_length-settle)
        self.z_max_error            = max_error
        # compute saturation and ground time as percentage of total test time
        self.motors_saturated_time  = mot_sat_tot/(self.trace_length-settle)
        self.hit_ground_time        = hit_ground/(self.trace_length-settle)
        # check for filtering of reference's frequency with largest component
        # TODO --- define filtering on more than the highest peak
        index_input_freq = np.where(self.z_ref_fft== np.amax(self.z_ref_fft))
        self.z_filtering =  self.z_pos_fft[index_input_freq[0][0]]\
                            /self.z_ref_fft[index_input_freq[0][0]]

        # define behavioural region on the base of: 
        #  - hitting saturations
        #  - good reference tracking
        #  - filtering reference
        self.behaviour = self.bh_undefined
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

    def get_z_avg_error_abs(self):
        if not(hasattr(self, "z_avg_error_abs")):
            self.analyse_z()
        return self.z_avg_error_abs

    def get_z_avg_error_rel(self):
        if not(hasattr(self, "z_avg_error_rel")):
            self.analyse_z()
        return self.z_avg_error_rel

    def get_z_max_error(self):
        if not(hasattr(self, "z_max_error")):
            self.analyse_z()
        return self.z_max_error

    def get_motors_saturated(self):
        if not(hasattr(self, "motors_saturated_time")):
            self.analyse_z()
        return self.motors_saturated_time

    def get_hit_ground(self):
        if not(hasattr(self, "hit_ground_time")):
            self.analyse_z()
        return self.hit_ground_time

    def get_z_filtering(self):
        if not(hasattr(self, "z_filtering")):
            self.analyse_z()
        return self.z_filtering

    def get_behaviour(self):
        if not(hasattr(self, "behaviour")):
            self.analyse_z()
        return self.behaviour

    def get_z_fft_freq(self):
        if not(hasattr(self, "z_fft_freq")):
            self.analyse_z()
        return self.z_fft_freq

    def get_z_pos_fft(self):
        if not(hasattr(self, "z_pos_fft")):
            self.analyse_z()
        return self.z_pos_fft

    def get_z_ref_fft(self):
        if not(hasattr(self, "z_ref_fft")):
            self.analyse_z()
        return self.z_ref_fft

    def get_z_fft_freq_peaks(self):
        if not(hasattr(self, "z_fft_freq_peaks")):
            self.analyse_z()
        return self.z_fft_freq_peaks

    def get_z_ref_fft_peaks(self):
        if not(hasattr(self, "z_ref_fft_peaks")):
            self.analyse_z()
        return self.z_ref_fft_peaks
