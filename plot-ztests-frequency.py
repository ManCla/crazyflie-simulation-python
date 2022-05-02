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
    
    ### generate plotting objects
    # figure with separated different types of tests
    fig1, ax  = plt.subplots(4, 1)
    plt.subplots_adjust(wspace=0.2, hspace=1)
    plt.setp(ax,xlabel=x_label,ylabel=y_label)
    # TODO --- do not hardcode this but rather take this from the z-test-set file
    ax[0].grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)
    ax[1].grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)
    ax[2].grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)
    ax[3].grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)

    # figure with all tests together
    fig2, ax2 = plt.subplots(1, 1)
    plt.setp(ax2,xlabel=x_label,ylabel=y_label)
    ax2.grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)

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

        if data_storage.test=="steps" :
            plot_index = 0
        elif data_storage.test=="triangular" :
            plot_index = 1
        elif data_storage.test=="trapezoidal" :
            plot_index = 2
        elif data_storage.test=="sinus" :
            plot_index = 3

        # actual plotting for each test
        bh_color=fdh.bh_palette[data_storage.get_behaviour()]/255
        if plot_only_peaks :
            ax[plot_index].scatter(data_storage.get_z_fft_freq_peaks(),\
                                   data_storage.get_z_ref_fft_peaks(),\
                                   marker='x',c=bh_color)
            ax2.scatter(data_storage.get_z_fft_freq_peaks(),\
                        data_storage.get_z_ref_fft_peaks(),\
                        marker='x',c=bh_color)
        else :
            ax[plot_index].plot(data_storage.get_z_fft_freq(),\
                                   data_storage.get_z_ref_fft(),\
                                   c=bh_color, linewidth=1)
            ax2.plot(data_storage.get_z_fft_freq(),\
                        data_storage.get_z_ref_fft(),\
                        c=bh_color, linewidth=1)
    x_max = 3
    y_max = 120000
    ax[0].set_xlim([0, x_max]) # spectrum above 1Hz seems to be basically zero
    ax[1].set_xlim([0, x_max]) # spectrum above 1Hz seems to be basically zero
    ax[2].set_xlim([0, x_max]) # spectrum above 1Hz seems to be basically zero
    ax[3].set_xlim([0, x_max]) # spectrum above 1Hz seems to be basically zero
    ax[0].set_ylim([0, y_max])
    ax[1].set_ylim([0, y_max])
    ax[2].set_ylim([0, y_max])
    ax[3].set_ylim([0, y_max])
    ax[0].title.set_text("steps")
    ax[1].title.set_text("triangular")
    ax[2].title.set_text("trapezoidal")
    ax[3].title.set_text("sinus")
    ax2.set_xlim([0, x_max]) # spectrum above 1Hz seems to be basically zero
    ax2.set_ylim([0, y_max])
    ax2.title.set_text("All tests together")
    plt.show()
