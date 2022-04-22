import os
import matplotlib.pyplot as plt
import numpy as np
from cfSimulator.utils.FlightDataHandler import FlightDataHandler as fdh
import scipy.signal as ss

z_test_directory    = 'z-test'

exclude_step_responses = True
plot_only_peaks = True

chosen_grid_linewidth = 0.3
chosen_grid_linestyle = '--'
chosen_grid_color = 'gray'

# colour palette for behaviour coding
bh_palette = np.array([[255,255,255],  # black: bh_undefined
                       [  0,255,  0],  # green: bh_good_tracking
                       [  0,  0,255],  # blue : bh_filtering
                       [  0,255,255],  # azure: bh_good_tracking_extra
                       [255,  0,  0],  # red  : bh_sat_no_tracking
                       [  0,  0,  0]]) # dark : bh_something_wrong

if __name__ == "__main__":

    ######################
    ### Plotting Setup ###
    ######################

    # ticks and ticks labels
    x_label           = "Frequency [Hz]"
    y_label           = "Amplitude [m]"
    
    # generate plotting objects
    fig1, ax = plt.subplots(1, 1)
    plt.subplots_adjust(wspace=0.2, hspace=1)
    plt.setp(ax,xlabel=x_label,ylabel=y_label)
    ax.grid(color=chosen_grid_color, linestyle=chosen_grid_linestyle, linewidth=chosen_grid_linewidth)

    ##########################################
    ### Iterate over Test Results and Plot ###
    ##########################################

    directory = fdh.data_directory+'/'+z_test_directory
    dir_content = os.listdir(directory)
    # filter out directories and look only at files
    dir_content = list(filter(lambda x:os.path.isfile(directory+x), dir_content))
    
    for file in dir_content:
        # file opening and data extraction
        file_path = directory+'/'+file
        data_storage = fdh()
        data_storage.open(file_path, silent=True)
        data_storage.analyse_z() # needed to get the frequency spectrum

        # actual plotting for each test
        bh_color=bh_palette[data_storage.behaviour]/255
        if plot_only_peaks :
            peaks, _ = ss.find_peaks(data_storage.z_ref_fft)
            ax.plot(np.array(data_storage.z_fft_freq)[peaks],\
                    np.array(data_storage.z_ref_fft)[peaks],\
                    "x",color=bh_color)
        else : 
            ax.plot(data_storage.z_fft_freq, data_storage.z_ref_fft, color=bh_color, linewidth=1)
            ax.set_xlim([0, 1]) # spectrum above 1Hz seems to be basically zero

    plt.show()
