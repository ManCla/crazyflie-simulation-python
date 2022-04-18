import os
import matplotlib.pyplot as plt
import numpy as np
from cfSimulator.utils.FlightDataHandler import FlightDataHandler as fdh

path = 'flightdata/z-aggregated/'
z_test_directory    = 'z-test'

exclude_step_responses = True

chosen_grid_linewidth = 0.3
chosen_grid_linestyle = '--'
chosen_grid_color = 'gray'

if __name__ == "__main__":

    #######################
    ### read input data ###
    #######################

    frequencies = np.genfromtxt(path+'frequencies.csv', delimiter=',')
    amplitudes  = np.genfromtxt(path+'amplitudes.csv', delimiter=',')
    behaviour   = np.genfromtxt(path+'behaviour.csv', delimiter=',',dtype=np.int16)

    # if we are not interested in the step responses, remove them
    # the ones with freq=0 (first element of frequencies vector)
    if exclude_step_responses:
        num_freqs = len(frequencies)
        frequencies = frequencies[1:num_freqs]

    ################
    ### plotting ###
    ################

    # colour palette for behaviour coding
    bh_palette = np.array([[255,255,255],  # black: bh_undefined
                           [  0,255,  0],  # green: bh_good_tracking
                           [  0,  0,255],  # blue : bh_filtering
                           [  0,255,255],  # azure: bh_good_tracking_extra
                           [255,  0,  0],  # red  : bh_sat_no_tracking
                           [  0,  0,  0]]) # dark : bh_something_wrong
    behaviour_plot = bh_palette[behaviour]

    # titles
    

    # ticks and ticks labels
    x_label           = "Frequency [Hz]"
    x_ticks           = range(len(frequencies))
    x_ticks_labels    = list(map(lambda x: '.%d'%((x/(2*np.pi))*100),frequencies))
    if not(exclude_step_responses): x_ticks_labels[0] = 'step'
    y_label           = "Amplitude [m]"
    y_ticks           = range(len(amplitudes))
    y_ticks_labels    = list(map(str,amplitudes))

    min_freq_plot = 0
    max_freq_plot = 1.5
    min_amp_plot  = 0
    max_amp_plot  = 150000

    # generate plotting objects
    fig1, ax = plt.subplots(2, 1)
    plt.subplots_adjust(wspace=0.2, hspace=1)

    ### Plot ###
    direcrtory = fdh.data_directory+'/'+z_test_directory
    dir_content = os.listdir(direcrtory)
    for file in dir_content:
        file_path = fdh.data_directory+'/'+\
                    z_test_directory+'/'+\
                    file
        data_storage = fdh()
        data_storage.open(file_path, silent=True)
        data_storage.analyse_z() # needed to get the frequency spectrum

        bh_undefined       = 0
        bh_good_tracking   = 1
        bh_filtering       = 2
        bh_good_tracking_extra = 3 
        bh_sat_no_tracking = 4
        bh_something_wrong = 5
        if data_storage.behaviour==bh_good_tracking:
            ax[0].plot(data_storage.z_fft_freq, data_storage.z_ref_fft, 'green')
            ax[0].set_xlim([min_freq_plot, max_freq_plot])
            ax[0].set_ylim([min_amp_plot, max_amp_plot])
        if data_storage.behaviour==bh_filtering:
            ax[0].plot(data_storage.z_fft_freq, data_storage.z_ref_fft, 'blue')
            ax[0].set_xlim([min_freq_plot, max_freq_plot])
            ax[0].set_ylim([min_amp_plot, max_amp_plot])
        if data_storage.behaviour==bh_good_tracking_extra:
            ax[0].plot(data_storage.z_fft_freq, data_storage.z_ref_fft, 'cyan')
            ax[0].set_xlim([min_freq_plot, max_freq_plot])
            ax[0].set_ylim([min_amp_plot, max_amp_plot])
        if data_storage.behaviour==bh_sat_no_tracking and max(data_storage.z_ref_fft)<85000:
            ax[1].plot(data_storage.z_fft_freq, data_storage.z_ref_fft, 'red')
            ax[1].set_xlim([min_freq_plot, max_freq_plot])
            ax[1].set_ylim([min_amp_plot, max_amp_plot])

    plt.show()
