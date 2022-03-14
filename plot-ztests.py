import matplotlib.pyplot as plt
import numpy as np

path = 'flightdata/z-aggregated/'

if __name__ == "__main__":

    frequencies = np.genfromtxt(path+'frequencies.csv', delimiter=',')
    amplitudes  = np.genfromtxt(path+'amplitudes.csv', delimiter=',') 
    z_error_cumulative     = np.genfromtxt(path+'z_error_cumulative.csv', delimiter=',')
    z_error_rel_cumulative = np.genfromtxt(path+'z_error_rel_cumulative.csv', delimiter=',')
    motors_saturated       = np.genfromtxt(path+'motors_saturated.csv', delimiter=',')
    hit_ground             = np.genfromtxt(path+'hit_ground.csv', delimiter=',')

    # generate plotting objects
    fig, ax = plt.subplots(1, 3)

    # ticks and ticks labels
    x_ticks           = range(len(frequencies))
    x_ticks_labels    = list(map(str,frequencies))
    x_ticks_labels[0] = 'step'
    y_ticks           = range(len(amplitudes))
    y_ticks_labels    = list(map(str,amplitudes))
    plt.setp(ax, xticks=x_ticks, xticklabels=x_ticks_labels,\
                 yticks=y_ticks, yticklabels=y_ticks_labels)

    ax[0].imshow(z_error_cumulative, cmap='jet', interpolation='nearest',origin='lower')
    ax[0].set_title('cumulative error')
    ax[1].imshow(z_error_rel_cumulative, cmap='jet', interpolation='nearest',origin='lower')
    ax[1].set_title('cumulative error relative (to sp)')
    ax[2].imshow(motors_saturated, cmap='jet', interpolation='nearest',origin='lower')
    ax[2].set_title('total saturation time')

    # add values inside cells
    for i in range(len(frequencies)):
        for j in range(len(amplitudes)):
            ax[0].text(i,j,'%d'%(z_error_cumulative[j,i]),    ha='center',va='center')
            ax[1].text(i,j,'%d'%(z_error_rel_cumulative[j,i]),ha='center',va='center')
            ax[2].text(i,j,'%d'%(motors_saturated[j,i]),      ha='center',va='center')

    plt.show()
    # code.interact(local=locals())