import os
import matplotlib.pyplot as plt
import numpy as np
from cfSimulator import ZAnalysis as fdh

# directory inside the FlightDataHandler.data_directory directory
# where to take the data for the analysis
z_test_directory    = 'z-test-set'

# script parameters
plot_only_peaks = True

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
    ax.grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)

    ##########################################
    ### Iterate over Test Results and Plot ###
    ##########################################

    directory = fdh.data_directory+'/'+z_test_directory
    dir_content = os.listdir(directory)
    # filter out directories and look only at files
    dir_content = list(filter(lambda x:os.path.isfile(directory+'/'+x), dir_content))

    for file in dir_content:
        # file opening and data extraction
        file_path = directory+'/'+file
        data_storage = fdh()
        data_storage.open(file_path, silent=True)

        # actual plotting for each test
        bh_color=fdh.bh_palette[data_storage.get_behaviour()]/255
        if plot_only_peaks :
            ax.plot(data_storage.get_z_fft_freq_peaks(),\
                    data_storage.get_z_ref_fft_peaks(),\
                    "x",color=bh_color)
            ax.plot(data_storage.get_z_fft_freq(),\
                    data_storage.get_z_ref_fft(),\
                    color=bh_color)
        else :
            ax.plot(data_storage.get_z_fft_freq(),\
                    data_storage.get_z_ref_fft(),\
                    color=bh_color, linewidth=1)
        ax.set_xlim([0, 5]) # spectrum above 1Hz seems to be basically zero

    plt.show()
