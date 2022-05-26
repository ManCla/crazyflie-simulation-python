import os
import matplotlib.pyplot as plt
import numpy as np
from cfSimulator import ZAnalysis as fdh

# directory inside the FlightDataHandler.data_directory directory
# where to take the data for the analysis
z_test_directory    = 'z-test-set'

# script parameters
use_nl_bh_binary = False

if __name__ == "__main__":

    ######################
    ### Plotting Setup ###
    ######################

    # ticks and ticks labels
    x_label           = "Frequency [Hz]"
    y_label           = "Amplitude [m]"
    x_min = 0.01
    x_max = 2.5 # spectrum above 1Hz seems to be basically zero
    y_max = 5
    
    ### generate plotting objects
    # figure with separated different types of tests
    fig1, ax  = plt.subplots(5, 1)
    plt.subplots_adjust(wspace=0.2, hspace=1)
    plt.setp(ax,xlabel=x_label,ylabel=y_label)
    # TODO --- do not hardcode this but rather take this from the z-test-set file
    ax[0].grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)
    ax[1].grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)
    ax[2].grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)
    ax[3].grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)
    ax[4].grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)

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
        elif data_storage.test=="ud1" :
            plot_index = 4
        else :
            print("ERROR: Shape not recognized {}".format(data_storage.test))
            exit()

        ## get coordinates to plot
        freq_coordinates = data_storage.get_z_fft_freq_peaks()
        freq_coordinates[0] = x_min # show zero frequency to the left of the plot
        ampl_coordinates = data_storage.get_z_ref_fft_peaks()
        ## get behaviour to plot
        if use_nl_bh_binary :
            # detect non-linear behaviour as a binary variable when new peaks appear
            bh_color = fdh.bh_palette[data_storage.get_behaviour()]/255
        else :
            # detect non-linear behaviour as ration between new peaks and peaks in reference
            nld = data_storage.get_z_non_linear_degree()
            bh_color = [[nld,1-nld,0]] * len(freq_coordinates)
        ## actual plotting
        ax[plot_index].scatter(freq_coordinates, ampl_coordinates, marker='o',s=6,c=bh_color)
        ax2.scatter(freq_coordinates, ampl_coordinates, marker='o',s=6,c=bh_color)

    ax[0].set_xlim([x_min, x_max])
    ax[1].set_xlim([x_min, x_max])
    ax[2].set_xlim([x_min, x_max])
    ax[3].set_xlim([x_min, x_max])
    ax[4].set_xlim([x_min, x_max])
    ax[0].set_ylim([0, y_max])
    ax[1].set_ylim([0, y_max])
    ax[2].set_ylim([0, y_max])
    ax[3].set_ylim([0, y_max])
    ax[4].set_ylim([0, y_max])
    ax[0].title.set_text("steps")
    ax[1].title.set_text("triangular")
    ax[2].title.set_text("trapezoidal")
    ax[3].title.set_text("sinus")
    ax[4].title.set_text("user-defined")
    ax2.set_xlim([x_min, x_max]) # spectrum above 1Hz seems to be basically zero
    ax2.set_ylim([0, y_max])
    ax2.title.set_text("All tests together: gradient")
    # use log scale along logaritmoc axis
    ax[0].set_xscale('log')
    ax[1].set_xscale('log')
    ax[2].set_xscale('log')
    ax[3].set_xscale('log')
    ax[4].set_xscale('log')
    ax2.set_xscale('log')
    plt.show()
