import matplotlib.pyplot as plt
import numpy as np

path = 'flightdata/z-aggregated/'

exclude_step_responses = True

if __name__ == "__main__":

    frequencies = np.genfromtxt(path+'frequencies.csv', delimiter=',')
    amplitudes  = np.genfromtxt(path+'amplitudes.csv', delimiter=',') 
    z_error_rel_cumulative = np.genfromtxt(path+'z_error_rel_cumulative.csv', delimiter=',')
    z_max_error            = np.genfromtxt(path+'z_max_error.csv', delimiter=',')
    motors_saturated       = np.genfromtxt(path+'motors_saturated.csv', delimiter=',')
    hit_ground             = np.genfromtxt(path+'hit_ground.csv', delimiter=',')

    if exclude_step_responses:
        num_freqs = len(frequencies)
        frequencies = frequencies[1:num_freqs]
        z_error_rel_cumulative = z_error_rel_cumulative[:,1:num_freqs]
        z_max_error = z_max_error[:,1:num_freqs]
        motors_saturated = motors_saturated[:,1:num_freqs]
        hit_ground = hit_ground[:,1:num_freqs]

    # generate plotting objects
    fig1, ax1 = plt.subplots(1, 2)
    fig2, ax2 = plt.subplots(1, 2)

    # ticks and ticks labels
    x_ticks           = range(len(frequencies))
    x_ticks_labels    = list(map(str,frequencies))
    if not(exclude_step_responses):
        x_ticks_labels[0] = 'step'
    y_ticks           = range(len(amplitudes))
    y_ticks_labels    = list(map(str,amplitudes))

    ### Error plots ###
    ax1[0].imshow(z_error_rel_cumulative, cmap='jet', interpolation='nearest',origin='lower')
    ax1[0].set_title('cumulative error (relative to sp)')
    ax1[1].imshow(z_max_error, cmap='jet', interpolation='nearest',origin='lower')
    ax1[1].set_title('max error (relative to sp)')
    # set axis ticks
    plt.setp(ax1, xticks=x_ticks, xticklabels=x_ticks_labels,\
                  yticks=y_ticks, yticklabels=y_ticks_labels)

    ### Saturation and ground contact ###
    ax2[0].imshow(motors_saturated, cmap='jet', interpolation='nearest',origin='lower')
    ax2[0].set_title('percentage of time that motors are saturated')
    ax2[1].imshow(hit_ground, cmap='jet', interpolation='nearest',origin='lower')
    ax2[1].set_title('percentage of time the drone is on the ground')
    # set axis ticks
    plt.setp(ax2, xticks=x_ticks, xticklabels=x_ticks_labels,\
                  yticks=y_ticks, yticklabels=y_ticks_labels)

    # add values inside cells
    for i in range(len(frequencies)):
        for j in range(len(amplitudes)):
            ax1[0].text(i,j,'%d'%(z_error_rel_cumulative[j,i]/z_error_rel_cumulative[0,0]),ha='center',va='center')
            ax1[1].text(i,j,'%d'%(z_max_error[j,i]),ha='center',va='center')
            ax2[0].text(i,j,'%.2f'%(motors_saturated[j,i]),ha='center',va='center')
            ax2[1].text(i,j,'%.2f'%(hit_ground[j,i]),ha='center',va='center')

    plt.show()
