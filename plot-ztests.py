import matplotlib.pyplot as plt
import numpy as np

path = 'flightdata/z-aggregated/'

exclude_step_responses = True

if __name__ == "__main__":

    frequencies = np.genfromtxt(path+'frequencies.csv', delimiter=',')
    amplitudes  = np.genfromtxt(path+'amplitudes.csv', delimiter=',') 
    z_avg_error            = np.genfromtxt(path+'z_avg_error.csv', delimiter=',')
    z_avg_error_rel        = np.genfromtxt(path+'z_avg_error_rel.csv', delimiter=',')
    z_max_error            = np.genfromtxt(path+'z_max_error.csv', delimiter=',')
    motors_saturated       = np.genfromtxt(path+'motors_saturated.csv', delimiter=',')
    hit_ground             = np.genfromtxt(path+'hit_ground.csv', delimiter=',')

    if exclude_step_responses:
        num_freqs = len(frequencies)
        frequencies = frequencies[1:num_freqs]
        z_avg_error_rel = z_avg_error_rel[:,1:num_freqs]
        z_max_error = z_max_error[:,1:num_freqs]
        motors_saturated = motors_saturated[:,1:num_freqs]
        hit_ground = hit_ground[:,1:num_freqs]

    # generate plotting objects
    fig1, ax1 = plt.subplots(1, 2)
    fig2, ax2 = plt.subplots(1, 2)

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
    ax1[0].imshow(z_avg_error, cmap='jet', interpolation='nearest',origin='lower')
    ax1[0].set_title('average error')
    ax1[1].imshow(z_max_error, cmap='jet', interpolation='nearest',origin='lower')
    ax1[1].set_title('max error')
    # set axis ticks
    plt.setp(ax1, xticks=x_ticks, xticklabels=x_ticks_labels,xlabel=x_label,\
                  yticks=y_ticks, yticklabels=y_ticks_labels,ylabel=y_label)

    ### Saturation and ground contact ###
    ax2[0].imshow(motors_saturated, cmap='jet', interpolation='nearest',origin='lower')
    ax2[0].set_title('percentage of time that motors are saturated')
    ax2[1].imshow(hit_ground, cmap='jet', interpolation='nearest',origin='lower')
    ax2[1].set_title('percentage of time the drone is on the ground')
    # set axis ticks
    plt.setp(ax2, xticks=x_ticks, xticklabels=x_ticks_labels,xlabel=x_label,\
                  yticks=y_ticks, yticklabels=y_ticks_labels,ylabel=y_label)

    # add values inside cells
    for i in range(len(frequencies)):
        for j in range(len(amplitudes)):
            ax1[0].text(i,j,'%.1f'%(z_avg_error_rel[j,i]),ha='center',va='center')
            ax1[1].text(i,j,'%.1f'%(z_max_error[j,i]),ha='center',va='center')
            ax2[0].text(i,j,'%.2f'%(motors_saturated[j,i]),ha='center',va='center')
            ax2[1].text(i,j,'%.2f'%(hit_ground[j,i]),ha='center',va='center')

    plt.show()
