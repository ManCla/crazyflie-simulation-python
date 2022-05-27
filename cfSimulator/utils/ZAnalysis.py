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
    bh_filtering_extra = 3
    bh_sat_no_tracking = 4
    bh_something_wrong = 5
    bh_no_linear       = 4

    # colour palette for behaviour coding
    bh_palette = np.array([[155,155,155],  # grey : bh_undefined
                           [  0,255,  0],  # green: bh_good_tracking
                           [  0,  0,255],  # blue : bh_filtering
                           [  0,255,255],  # azure: bh_good_tracking_extra
                           [255,  0,  0],  # red  : bh_sat_no_tracking
                           [  0,  0,  0]]) # dark : bh_something_wrong

    def z_loop_frequency_plot(self):
        if not(hasattr(self, "z_fft_freq")):
            self.freq_behaviour_z() # needed to get the frequency spectrum
        fig, axs = plt.subplots(1, 1, figsize=self.chosen_size)
        plt.subplots_adjust(wspace=0.2, hspace=1)
        min_freq_plot = 0
        max_freq_plot = 10
        axs.title.set_text('Frequency spectrum of z reference and z position')
        axs.scatter(self.z_fft_freq, self.z_ref_fft, color='k', s=5)
        axs.scatter(self.z_fft_freq, self.z_pos_fft, color='red', s=5)
        axs.scatter(self.z_fft_freq, self.z_err_fft, color='green', s=5)
        axs.plot(self.z_ref_freq_peaks, self.z_ref_amp_peaks,"x", color='k')
        axs.plot(self.z_pos_freq_peaks, self.z_pos_amp_peaks,"x", color='red')
        axs.plot(self.z_err_freq_peaks, self.z_err_amp_peaks,"x", color='green')
        axs.legend(['reference','position','error','reference peaks','position peaks','error peaks'])
        axs.set_xlim([min_freq_plot, max_freq_plot])
        axs.grid(color=self.chosen_grid_color, linestyle=self.chosen_grid_linestyle, linewidth=self.chosen_grid_linewidth)
        print('* figure 5:\033[33m frequency spectrum of z reference and position\033[0m')

    ### function to generate and show all plots available
    ### Override parent method to plot also z loop frequency spectrum
    def show_all(self):
        print("Motor saturation percentage: {:.2f}".format(self.get_motors_saturated()))
        print("Hit ground percentage: {:.2f}".format(self.get_hit_ground()))
        print("Detected behaviour is: {}".format(self.get_behaviour()))
        print("Degree of non-linearity is: {}".format(self.get_z_non_linear_degree()))
        print("Degree of filtering is: {}".format(self.get_z_filter_degree()))
        self.positionSpeedPlot()
        self.z_loop_frequency_plot()
        plt.show()


    ##########################
    ### ANALYSIS FUNCTIONS ###
    ##########################

    '''
    Function that computes the frequency-peaks-based behaviour definition.
    ALGO: TODO --- write documentation sketch of algo
    USES: 
    OUTPUT: list of triplets [(frequency, amplitude, behaviour)] TODO --- fix me
    '''
    def freq_behaviour_z(self):
        # FREQ. DOM. ANALYSIS

        ### PARAMETERS -- TODO should be defined elsewhere
        peak_threshold = 0.05 # percentage of maximum peak above which we look for more peaks
        dt     = 0.001     # sampling time in seconds
        settle = int(5/dt) # test  warm up time not used in analysis
        freq_diff_tolerance = 1 # maximum accepted difference in indexes over freq vector of peaks
        base_hover = 1 # removed from the reference and position to study only effect of repeated shape
        ### END PARAMETERS

        # apply fft to a given number of periods
        # this can be used to test the effect of considering test durations defined by the 
        # number of repetitions rather than the absolute time
        # TODO: since the number of periods will be the metric to define test duration
        #       it will come included in the test duration. Move it to test definition.
        time_coef = float(self.data_location.split('-')[-1]) # get duration of a single period
        num_periods_spectrum = 5 # number of periods of the input to be used for dft (max = (60-5)*time_coef/10)
        if num_periods_spectrum > 0 :
            end_analysis = settle + num_periods_spectrum*int((10/time_coef)/dt)
            if end_analysis>self.trace_length :
                print("Trying to analyse more periods than we have, {} I will use what I have".format(self.data_location))
                end_analysis = self.trace_length
        else :
            end_analysis = self.trace_length
        z_fft_freq = fft.fftfreq((end_analysis-settle), d=dt)

        # fft computation and extract module
        z_err_fft  = list(map(abs, fft.fft(self.set_pt[2,settle:end_analysis]\
                                           -self.pos[2,:][settle:end_analysis],\
                                           norm="forward", workers=-1)))
        z_ref_fft  = list(map(abs, fft.fft(self.set_pt[2,settle:end_analysis]-base_hover,\
                                           norm="forward", workers=-1)))
        z_pos_fft  = list(map(abs, fft.fft(self.pos[2,:][settle:end_analysis]-base_hover,\
                                           norm="forward", workers=-1)))
        # spectrum is symmetric
        self.z_err_fft = z_err_fft[:len(z_err_fft)//2]
        self.z_ref_fft = z_ref_fft[:len(z_ref_fft)//2]
        self.z_pos_fft = z_pos_fft[:len(z_pos_fft)//2]
        self.z_fft_freq = z_fft_freq[:len(z_fft_freq)//2]

        # TODO --- define peak threshold only on the base of ref amplitude?
        #      --- in general should investigate better how to do this:
        #      --- see the problem for low freq tests and triangular wave and threshold at 0.1

        # find peaks in reference spectrum 
        # peak at zero frequency is included manually because it is always important
        # but also always excluded by find_peaks.
        ref_peaks_indexes, _  = signal.find_peaks(self.z_ref_fft,\
                                                  height= peak_threshold*max(self.z_ref_fft[1:]))
        ref_peaks_indexes = np.hstack(([0],ref_peaks_indexes))
        self.z_ref_freq_peaks = np.array(self.z_fft_freq)[ref_peaks_indexes]
        self.z_ref_amp_peaks  = np.array(self.z_ref_fft)[ref_peaks_indexes]

        # find peaks in output spectrum
        pos_peaks_indexes, _  = signal.find_peaks(self.z_pos_fft,\
                                                  height= peak_threshold*max(self.z_ref_fft[1:]))
        pos_peaks_indexes = np.hstack(([0],pos_peaks_indexes))
        self.z_pos_freq_peaks = np.array(self.z_fft_freq)[pos_peaks_indexes]
        self.z_pos_amp_peaks  = np.array(self.z_pos_fft)[pos_peaks_indexes]

        # find peaks in error spectrum
        err_peaks_indexes, _  = signal.find_peaks(self.z_err_fft,\
                                                  height= peak_threshold*max(self.z_ref_fft[1:]))
        err_peaks_indexes = np.hstack(([0],err_peaks_indexes))
        self.z_err_freq_peaks = np.array(self.z_fft_freq)[err_peaks_indexes]
        self.z_err_amp_peaks  = np.array(self.z_err_fft)[err_peaks_indexes]

        ### BEHAVIOUR DETECTION ###
        # initialize behaviour variables
        self.freq_analysis_bin_behaviour = [self.bh_undefined] * len(ref_peaks_indexes)
        self.z_non_linear_degree = 0
        self.z_filter_degree     = [0] * len(ref_peaks_indexes)

        # iterate over reference peaks and measure filtering degree
        # note that we are also doing it for the tests that show nonlinear behaviour
        # it is up to the later steps to exclude those tests from the filtering analysis
        for idx in range(len(ref_peaks_indexes)) :
            rp_idx = ref_peaks_indexes[idx]
            self.z_filter_degree[idx] = abs((self.z_pos_fft[rp_idx]/self.z_ref_fft[rp_idx])-1)

        # iterate over frequency of peaks of position spectrum
        for pp_idx in pos_peaks_indexes :
            # look for same peak in reference spectrum peaks
            find_ref_peak = [abs(x-pp_idx)<=freq_diff_tolerance for x in ref_peaks_indexes]
            if any(find_ref_peak) :
                # peak was found, define type {ref_tracking, filtering}
                idx = find_ref_peak.index(True)
                if self.z_pos_fft[ref_peaks_indexes[idx]]>self.z_err_fft[ref_peaks_indexes[idx]]:
                    self.freq_analysis_bin_behaviour[idx] = self.bh_good_tracking
                else :
                    self.freq_analysis_bin_behaviour[idx] = self.bh_filtering
            else :
                # peak was not found: non-linear behaviour detected, mark all input frequencies
                self.freq_analysis_bin_behaviour = [self.bh_no_linear] * len(ref_peaks_indexes) # binary detection
                self.z_non_linear_degree = min(1,self.z_non_linear_degree + self.z_pos_fft[pp_idx]/max(self.z_ref_fft[1:])) # gradual detection
        # if behaviour is OK iterate over remaining reference peaks 
        # and mark as filtering
        if self.freq_analysis_bin_behaviour[0] != self.bh_no_linear:
            for rp_idx in ref_peaks_indexes :
                find_pos_peak = [abs(x-rp_idx)<=freq_diff_tolerance for x in pos_peaks_indexes]
                if not(any(find_pos_peak)) :
                    idx = np.where(ref_peaks_indexes==rp_idx)[0][0]
                    self.freq_analysis_bin_behaviour[idx] = self.bh_filtering

    def analyse_z_sat_and_ground(self):
        ### PARAMETERS -- TODO should be defined elsewhere
        dt = 0.001         # sampling time in seconds
        thrust_min = 20000 # saturation limits of motors
        thrust_max = 65535 # 
        settle     = int(5/dt) #
        ### END PARAMETERS

        self.motors_saturated_percentage = sum([(x==thrust_min or x==thrust_max) for x in self.u[0,settle:self.trace_length]])\
                                           /(self.trace_length-settle)
        self.hit_ground_percentage = sum( x<0.01 for x in self.pos[2,settle:self.trace_length] )\
                                     /(self.trace_length-settle)

    #####################
    ### GET FUNCTIONS ###
    #####################

    def get_motors_saturated(self):
        if not(hasattr(self, "motors_saturated_percentage")):
            self.analyse_z_sat_and_ground()
        return self.motors_saturated_percentage

    def get_hit_ground(self):
        if not(hasattr(self, "hit_ground_percentage")):
            self.analyse_z_sat_and_ground()
        return self.hit_ground_percentage

    def get_behaviour(self):
        if not(hasattr(self, "freq_analysis_bin_behaviour")):
            self.freq_behaviour_z()
        return self.freq_analysis_bin_behaviour

    def get_z_fft_freq(self):
        if not(hasattr(self, "z_fft_freq")):
            self.freq_behaviour_z()
        return self.z_fft_freq

    def get_z_pos_fft(self):
        if not(hasattr(self, "z_pos_fft")):
            self.freq_behaviour_z()
        return self.z_pos_fft

    def get_z_ref_fft(self):
        if not(hasattr(self, "z_ref_fft")):
            self.freq_behaviour_z()
        return self.z_ref_fft

    def get_z_fft_freq_peaks(self):
        if not(hasattr(self, "z_ref_freq_peaks")):
            self.freq_behaviour_z()
        return self.z_ref_freq_peaks

    def get_z_ref_fft_peaks(self):
        if not(hasattr(self, "z_ref_amp_peaks")):
            self.freq_behaviour_z()
        return self.z_ref_amp_peaks

    def get_z_non_linear_degree(self):
        if not(hasattr(self, "z_non_linear_degree")):
            self.freq_behaviour_z()
        return self.z_non_linear_degree

    def get_z_filter_degree(self):
        if not(hasattr(self, "z_filter_degree")):
            self.freq_behaviour_z()
        return self.z_filter_degree
