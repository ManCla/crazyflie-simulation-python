import sys
import os
from utils.FlightDataHandler import FlightDataHandler

if __name__ == "__main__":

    # parsing command line parameters
    if sys.argv[1] not in ("pdf", "show"):
        print('\033[91mError:\033[0m ' + 'python plot.py <1> <2>')
        print(' <1>: either "pdf" or "show"')
        print(' <2>: OPTIONAL - absolute or relative path of the experiment file')
        exit()

    command = sys.argv[1]
    # if len(sys.argv) == 3:
    file_location = sys.argv[2]
    # else :
    #     print 
    #     file_location = "flightdata/"+(os.listdir("flightdata").sort()[-1])
    experiment_name = file_location.split('/')[-1]
    data_storage = FlightDataHandler()
    data_storage.open(file_location, experiment_name)
    print('* read data with total length: \033[33m' + str(data_storage.trace_length) + '\033[0m')

    if command == "pdf":
        data_storage.pdf(experiment_name)

    if command == "show":
        data_storage.show()