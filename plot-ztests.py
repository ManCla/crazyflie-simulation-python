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
    z_avg_error            = np.genfromtxt(path+'z_avg_error.csv', delimiter=',')
    z_avg_error_rel        = np.genfromtxt(path+'z_avg_error_rel.csv', delimiter=',')
    z_max_error            = np.genfromtxt(path+'z_max_error.csv', delimiter=',')
    motors_saturated       = np.genfromtxt(path+'motors_saturated.csv', delimiter=',')
    hit_ground             = np.genfromtxt(path+'hit_ground.csv', delimiter=',')
    z_filtering            = np.genfromtxt(path+'z_filtering.csv', delimiter=',')

    if exclude_step_responses:
        num_freqs = len(frequencies)
        frequencies = frequencies[1:num_freqs]
        z_avg_error = z_avg_error[:,1:num_freqs]
        z_avg_error_rel = z_avg_error_rel[:,1:num_freqs]
        z_max_error = z_max_error[:,1:num_freqs]
        motors_saturated = motors_saturated[:,1:num_freqs]
        hit_ground = hit_ground[:,1:num_freqs]
        z_filtering = z_filtering[:,1:num_freqs]

    ################
    ### plotting ###
    ################

    # generate plotting objects
    fig1, ax1 = plt.subplots(2, 2)
    
    # ticks and ticks labels
    x_label           = "Frequency"
    x_ticks           = range(len(frequencies))
    x_ticks_labels    = list(map(str,frequencies))
    if not(exclude_step_responses):
        x_ticks_labels[0] = 'step'
    y_label           = "Amplitude"
    y_ticks           = range(len(amplitudes))
    y_ticks_labels    = list(map(str,amplitudes))

    ### Error plots ###
    aspect='0.5'
    ax1[0,0].imshow(z_avg_error, cmap='jet', interpolation='nearest',origin='lower',aspect=aspect)
    ax1[0,0].set_title('average error')
    ax1[0,1].imshow(motors_saturated, cmap='jet', interpolation='nearest',origin='lower',aspect=aspect)
    ax1[0,1].set_title('percentage of time that motors are saturated')
    ax1[1,0].imshow(z_filtering, cmap='jet', interpolation='nearest',origin='lower',aspect=aspect)
    ax1[1,0].set_title('Gain at highest freq component of input')
    # set axis ticks
    plt.setp(ax1, xticks=x_ticks, xticklabels=x_ticks_labels,xlabel=x_label,\
                  yticks=y_ticks, yticklabels=y_ticks_labels,ylabel=y_label)

    ### Saturation and ground contact ###

    # add values inside cells
    for i in range(len(frequencies)):
        for j in range(len(amplitudes)):
            ax1[0,0].text(i,j,'%.1f'%(z_avg_error_rel[j,i]),ha='center',va='center')
            ax1[0,1].text(i,j,'%.2f'%(motors_saturated[j,i]),ha='center',va='center')
            ax1[1,0].text(i,j,'%.2f'%(z_filtering[j,i]),ha='center',va='center')
    plt.show()
