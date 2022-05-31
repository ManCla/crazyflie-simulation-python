import os
import matplotlib.pyplot as plt
import numpy as np
from cfSimulator import ZAnalysis as fdh

# directory inside the FlightDataHandler.data_directory directory
# where to take the data for the analysis
z_test_directory    = 'z-test-set'

# script parameters
non_lin_threshold = 0.3
stain_non_linear_tests_in_filtering = False

# plotting parameters
x_label = "Frequency [Hz]"
y_label = "Amplitude [m]"
x_min   = 0.005 # used as x coordinate for zero frequency component of tests in log scale

if __name__ == "__main__":

    #################################
    ### generate plotting objects ###
    ### NON LINEAR BEHAVIOUR      ###
    #################################

    # figure with separated different types of tests
    non_lin_fig_shapes , non_lin_ax_shapes  = plt.subplots(5, 1, sharex=True, sharey=True)
    non_lin_fig_shapes.tight_layout()
    non_lin_fig_shapes.supxlabel(x_label)
    non_lin_fig_shapes.supylabel(y_label)
    for ax in non_lin_ax_shapes : ax.grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)
    non_lin_ax_shapes[0].set_xscale('log')
    non_lin_ax_shapes[0].set_yscale('log')
    non_lin_ax_shapes[0].title.set_text("Non Linear Degree Steps")
    non_lin_ax_shapes[1].title.set_text("Non Linear Degree Triangular")
    non_lin_ax_shapes[2].title.set_text("Non Linear Degree Trapezoidal")
    non_lin_ax_shapes[3].title.set_text("Non Linear Degree Sinus")
    non_lin_ax_shapes[4].title.set_text("Non Linear Degree User-Defined")

    # figure with all tests together
    non_lin_fig , non_lin_ax = plt.subplots(1, 1)
    non_lin_fig.tight_layout()
    non_lin_ax.grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)
    plt.setp(non_lin_ax, xlabel=x_label, ylabel=y_label)
    non_lin_ax.set_xscale('log')
    non_lin_ax.set_yscale('log')
    non_lin_ax.title.set_text("Non Linear Degree All Shapes Together")

    #################################
    ### generate plotting objects ###
    ### FILTERING ACTION          ###
    #################################
    
    # figure with separated different types of tests
    filter_fig_shapes , filter_ax_shapes  = plt.subplots(5, 1, sharex=True, sharey=True)
    filter_fig_shapes.tight_layout()
    filter_fig_shapes.supxlabel(x_label)
    filter_fig_shapes.supylabel(y_label)
    for ax in filter_ax_shapes : ax.grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)
    filter_ax_shapes[0].set_xscale('log')
    filter_ax_shapes[0].set_yscale('log')
    filter_ax_shapes[0].title.set_text("Filtering Degree Steps")
    filter_ax_shapes[1].title.set_text("Filtering Degree Triangular")
    filter_ax_shapes[2].title.set_text("Filtering Degree Trapezoidal")
    filter_ax_shapes[3].title.set_text("Filtering Degree Sinus")
    filter_ax_shapes[4].title.set_text("Filtering Degree User-Defined")

    # figure with all tests together
    filter_fig , filter_ax = plt.subplots(1, 1)
    filter_fig.tight_layout()
    filter_ax.grid(color=fdh.chosen_grid_color, linestyle=fdh.chosen_grid_linestyle, linewidth=fdh.chosen_grid_linewidth)
    plt.setp(filter_ax, xlabel=x_label, ylabel=y_label)
    filter_ax.set_xscale('log')
    filter_ax.set_yscale('log')
    filter_ax.title.set_text("Filtering Degree All Shapes Together")

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

        ### NON LINEAR DEGREE
        # note: this is one measure for the whole test
        nld = min(1,data_storage.get_z_non_linear_degree())     # get behaviour
        if nld<0.5 :
            # gradient from blue to yellow
            non_lin_color = [[2*nld,2*nld,2*(0.5-nld)]] * len(freq_coordinates) # transform into rgb colour
        else :
            # gradient from yellow to red
            non_lin_color = [[1,2*(0.5-(nld-0.5)),0]] * len(freq_coordinates) # transform into rgb colour
        non_lin_ax_shapes[plot_index].scatter(freq_coordinates, ampl_coordinates, marker='o',s=2,c=non_lin_color)
        non_lin_ax.scatter(freq_coordinates, ampl_coordinates, marker='o',s=2,c=non_lin_color)

        ### FILTERING DEGREE
        # note: this is a measure for each of the peaks of the input
        if nld>non_lin_threshold and stain_non_linear_tests_in_filtering :
            # non linear behaviour is above threshold, stain it
            filter_color = [ [1,0,0] for x in data_storage.get_z_filter_degree()]
        else : 
            # colour for degree of filtering
            filter_color = [ [0,min(x,1),1-min(x,1)] for x in data_storage.get_z_filter_degree()]
        filter_ax_shapes[plot_index].scatter(freq_coordinates, ampl_coordinates, marker='o',s=2,c=filter_color)
        filter_ax.scatter(freq_coordinates, ampl_coordinates, marker='o',s=2,c=filter_color)

    plt.show()
