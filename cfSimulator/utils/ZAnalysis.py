from . import FlightDataHandler

import numpy as np
import scipy.signal as signal
import scipy.fft as fft
import matplotlib.pyplot as plt
import copy as cp

# number of input periods to be used for fft (negative value uses full trace)
num_periods_spectrum = 5
freq_diff_tolerance = 1 # maximum accepted difference in indexes over freq vector of peaks
peak_threshold_percentage = 0.05 # percentage of max ref peak above which we look for more peaks in spectra

### DRONE AND TEST PARAMETERS 
# TODO: should be retrieved from test case and model
base_period_input = 1 # base period over which input shapes are defined. Used to compute number of periods
dt     = 0.001     # sampling time in seconds
settle = int(5/dt) # test  warm up time not used in analysis
base_hover = 1     # removed from the reference and position to study only effect of repeated shape
thrust_min = 20000 # saturation limits of motors
thrust_max = 65535 # 

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
        min_freq_plot = 0.005 # cant be zero with logarithmic scale
        max_freq_plot = 5
        axs.title.set_text('Frequency spectrum of z reference and z position')
        axs.scatter(self.z_fft_freq, self.z_ref_fft, color='k', s=5)
        axs.scatter(self.z_fft_freq, self.z_pos_fft, color='red', s=5)
        axs.set_xscale('log')
        axs.set_yscale('log')
        self.z_ref_freq_peaks[0] = min_freq_plot # needed to show zero frequency on logarithmic scale
        self.z_pos_freq_peaks[0] = min_freq_plot # needed to show zero frequency on logarithmic scale
        axs.plot(self.z_ref_freq_peaks, self.z_ref_amp_peaks,"x", color='k')
        axs.plot(self.z_pos_freq_peaks, self.z_pos_amp_peaks,"x", color='red')
        axs.legend(['reference','position','reference peaks','position peaks'])
        axs.set_xlim([min_freq_plot, max_freq_plot])
        axs.grid(color=self.chosen_grid_color,\
                 linestyle=self.chosen_grid_linestyle,\
                 linewidth=self.chosen_grid_linewidth)
        print('* figure 5:\033[33m frequency spectrum of z reference and position\033[0m')

    ### function to generate and show all plots available
    ### Override parent method to plot also z loop frequency spectrum
    def show_all(self):
        print("Motor saturation percentage: {:.2f}".format(self.get_motors_saturated()))
        print("Hit ground percentage: {:.2f}".format(self.get_hit_ground()))
        print("Degree of non-linearity is: {}".format(self.get_z_non_linear_degree()))
        print("Degree of filtering is: {}".format(self.get_z_filter_degree()))
        self.positionSpeedPlot()
        self.z_loop_frequency_plot()
        plt.show()


    ##########################
    ### ANALYSIS FUNCTIONS ###
    ##########################

    '''
    Utility function that computes the number of samples of trace
    to analyse according to the number of periods of interest.
    This can be used to test the effect of considering different
    numbers of periods in the fft.
    '''
    def compute_end_analysis(self):
        time_coef = float(self.data_location.split('-')[-1]) # time dilation coefficient of test
        if num_periods_spectrum > 0 :
            self.end_analysis = settle + num_periods_spectrum*int((base_period_input/time_coef)/dt)
            if self.end_analysis>(self.trace_length+1) : # we accept a rounding error
                print("Trying to fft too many periods: I will use what I have--in TEST: {}".format(self.data_location))
                self.end_analysis = self.trace_length
        else :
            self.end_analysis = self.trace_length

    '''
    Function that computes the frequency-peaks-based behaviour definition.
    ALGO: TODO --- write documentation sketch of algo
    Computes spectra of input and output with relative peaks.
    Evaluates the behaviour of peaks as:
     - z_non_linear_degree (one for all test)
     - z_filter_degree (one for each peak in reference spectrum)
    '''
    def freq_behaviour_z(self, silent=True):
        if not(hasattr(self, "end_analysis")):
            self.compute_end_analysis()
        ### AMPLITUDE OF FREQUENCY SPECTRUM COMPUTATION ###
        z_fft_freq = fft.fftfreq(self.end_analysis-settle, d=dt)
        cropped_ref = self.set_pt[2,settle:self.end_analysis]-base_hover
        cropped_pos = self.pos[2,:][settle:self.end_analysis]-base_hover
        z_ref_fft  = [abs(x) for x in fft.fft(cropped_ref, norm="forward", workers=-1, overwrite_x=True)]
        z_pos_fft  = [abs(x) for x in fft.fft(cropped_pos, norm="forward", workers=-1, overwrite_x=True)]
        # spectrum is symmetric
        self.z_fft_freq = z_fft_freq[:len(z_fft_freq)//2]
        self.z_ref_fft  = z_ref_fft[:len(z_ref_fft)//2]
        self.z_pos_fft  = z_pos_fft[:len(z_pos_fft)//2]

        # find peaks in reference spectrum
        peak_threshold = peak_threshold_percentage * max(self.z_ref_fft[1:]) # min value of peaks relative to ref
        # zero frequency is always included because it is always important but also always excluded by find_peaks
        ref_peaks_indexes, _  = signal.find_peaks(self.z_ref_fft, height= peak_threshold)
        ref_peaks_indexes     = np.hstack(([0],ref_peaks_indexes))
        self.z_ref_freq_peaks = np.array(self.z_fft_freq)[ref_peaks_indexes]
        self.z_ref_amp_peaks  = np.array(self.z_ref_fft)[ref_peaks_indexes]
        # find peaks in output spectrum
        pos_peaks_indexes, _  = signal.find_peaks(self.z_pos_fft, height= peak_threshold)
        pos_peaks_indexes     = np.hstack(([0],pos_peaks_indexes))
        self.z_pos_freq_peaks = np.array(self.z_fft_freq)[pos_peaks_indexes]
        self.z_pos_amp_peaks  = np.array(self.z_pos_fft)[pos_peaks_indexes]

        ### BEHAVIOUR DETECTION ###
        # initialize behaviour variables
        self.z_non_linear_degree = 0
        self.z_filter_degree     = [0] * len(ref_peaks_indexes)

        # iterate over reference peaks and measure filtering degree
        # note that we are also doing it for the tests that show nonlinear behaviour
        # it is up to the later steps to exclude those tests from the filtering analysis
        z_pos_fft_non_lin_part = cp.copy(self.z_pos_fft)
        z_pos_fft_non_lin_part[0] = 0 # always remove zero freq

        for idx in range(len(ref_peaks_indexes)) :
            rp_idx = ref_peaks_indexes[idx]
            z_pos_fft_non_lin_part[rp_idx] = 0 # remove input frequencies from output spectrum
            self.z_filter_degree[idx] = abs((self.z_pos_fft[rp_idx]/self.z_ref_fft[rp_idx])-1)

        nl_deg = max(z_pos_fft_non_lin_part)/max(self.z_ref_fft[1:])
        if nl_deg > 1 and not(silent) :
            print("---nonlinear degree >1!!!-----in TEST: {}".format(self.data_location))
        self.z_non_linear_degree = nl_deg

    '''
    get given number of maximum components of reference fft
    '''
    def maxima_ref_fa_components(self, num_maxima) :
        if not(hasattr(self, "z_ref_freq_peaks")):
            self.freq_behaviour_z()
        #get indexes of maxima elements - exclude zero frequency
        ordered_amp_indexes = np.array(self.z_ref_fft[1:]).argsort()[-num_maxima:]+1
        self.z_ref_freq_maxima = np.array(self.z_fft_freq)[ordered_amp_indexes]
        self.z_ref_amp_maxima  = np.array(self.z_ref_fft)[ordered_amp_indexes]
        return self.z_ref_freq_maxima, self.z_ref_amp_maxima

    def analyse_z_sat_and_ground(self):
        if not(hasattr(self, "end_analysis")):
            self.compute_end_analysis()
        mot_sat_samples = sum([(x==thrust_min or x==thrust_max) for x in self.u[0,settle:self.end_analysis]])
        self.motors_saturated_percentage = mot_sat_samples/(self.end_analysis-settle)
        hit_ground_samples = sum(x<0.01 for x in self.pos[2,settle:self.end_analysis])
        self.hit_ground_percentage = hit_ground_samples/(self.end_analysis-settle)

    #####################
    ### GET FUNCTIONS ###
    #####################

    def get_end_analysis(self):
        if not(hasattr(self, "end_analysis")):
            self.compute_end_analysis()
        return self.end_analysis

    def get_motors_saturated(self):
        if not(hasattr(self, "motors_saturated_percentage")):
            self.analyse_z_sat_and_ground()
        return self.motors_saturated_percentage

    def get_hit_ground(self):
        if not(hasattr(self, "hit_ground_percentage")):
            self.analyse_z_sat_and_ground()
        return self.hit_ground_percentage

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
