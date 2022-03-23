import matplotlib.pyplot as plt
import numpy as np

path = 'flightdata/z-aggregated/'

exclude_step_responses = True

if __name__ == "__main__":

    #######################
    ### read input data ###
    #######################

    frequencies = np.genfromtxt(path+'frequencies.csv', delimiter=',')
    amplitudes  = np.genfromtxt(path+'amplitudes.csv', delimiter=',') 
    z_avg_error_abs  = np.genfromtxt(path+'z_avg_error_abs.csv', delimiter=',')
    z_avg_error_rel  = np.genfromtxt(path+'z_avg_error_rel.csv', delimiter=',')
    z_max_error      = np.genfromtxt(path+'z_max_error.csv', delimiter=',')
    motors_saturated = np.genfromtxt(path+'motors_saturated.csv', delimiter=',')
    hit_ground       = np.genfromtxt(path+'hit_ground.csv', delimiter=',')
    z_filtering      = np.genfromtxt(path+'z_filtering.csv', delimiter=',')
    behaviour        = np.genfromtxt(path+'behaviour.csv', delimiter=',',dtype=np.int16)

    # if we are not interested in the step responses, remove them
    # the ones with freq=0 (first element of frequencies vector)
    if exclude_step_responses:
        num_freqs = len(frequencies)
        frequencies = frequencies[1:num_freqs]
        z_avg_error_abs = z_avg_error_abs[:,1:num_freqs]
        z_avg_error_rel = z_avg_error_rel[:,1:num_freqs]
        z_max_error = z_max_error[:,1:num_freqs]
        motors_saturated = motors_saturated[:,1:num_freqs]
        hit_ground = hit_ground[:,1:num_freqs]
        z_filtering = z_filtering[:,1:num_freqs]
        behaviour = behaviour[:,1:num_freqs]

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
    z_avg_error_rel_title  = 'Average Normalized Absolute Error'
    motors_saturated_title = 'Percentage of time that motors are saturated'
    z_filtering_title      = 'Gain at highest freq component of input'
    behaviour_plot_title   = 'Gain at highest freq component of input'

    # ticks and ticks labels
    x_label           = "Frequency [Hz]"
    x_ticks           = range(len(frequencies))
    x_ticks_labels    = list(map(lambda x: '%.2f'%(x/(2*np.pi)),frequencies))
    if not(exclude_step_responses): x_ticks_labels[0] = 'step'
    y_label           = "Amplitude [m]"
    y_ticks           = range(len(amplitudes))
    y_ticks_labels    = list(map(str,amplitudes))

    # generate plotting objects
    fig1, ax1 = plt.subplots(2, 2)
    aspect='0.5' # width height ratio of cells

    ### Actual plotting ###
    ax1[0,0].imshow(z_avg_error_rel,  cmap='jet', interpolation='nearest',origin='lower',aspect=aspect)
    ax1[0,1].imshow(motors_saturated, cmap='jet', interpolation='nearest',origin='lower',aspect=aspect)
    ax1[1,0].imshow(z_filtering,      cmap='jet', interpolation='nearest',origin='lower',aspect=aspect)
    ax1[1,1].imshow(behaviour_plot,   origin='lower', aspect=aspect)
    # add values inside cells
    for i in range(len(frequencies)):
        for j in range(len(amplitudes)):
            ax1[0,0].text(i,j,'%.1f'%(z_avg_error_rel[j,i]),ha='center',va='center')
            ax1[0,1].text(i,j,'%.2f'%(motors_saturated[j,i]),ha='center',va='center')
            ax1[1,0].text(i,j,'%.2f'%(z_filtering[j,i]),ha='center',va='center')

    # set titles
    ax1[0,0].set_title(z_avg_error_rel_title)
    ax1[0,1].set_title(motors_saturated_title)
    ax1[1,0].set_title(z_filtering_title)
    ax1[1,1].set_title(behaviour_plot_title)

    # set axis ticks
    plt.setp(ax1, xticks=x_ticks, xticklabels=x_ticks_labels,xlabel=x_label,\
                  yticks=y_ticks, yticklabels=y_ticks_labels,ylabel=y_label)

    ### Scatter Plot ###
    fig2 = plt.figure()
    for f in range(len(frequencies)):
        for a in range(len(amplitudes)):
            color = [behaviour_plot[a,f]/255]
            plt.scatter(frequencies[f],amplitudes[a],c=color,s=25)
    plt.setp(fig2.axes, xticks=frequencies, xticklabels=x_ticks_labels,xlabel=x_label,\
                        yticks=amplitudes,  yticklabels=y_ticks_labels,ylabel=y_label)

    plt.show()
