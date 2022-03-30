import sys
import os
from datetime import datetime
from cfSimulator.utils.FlightDataHandler import FlightDataHandler as fdh

if __name__ == "__main__":

    if len(sys.argv) == 2: # if data file is given
        file_location   = sys.argv[1]
        experiment_name = file_location.split('/')[-1]
    else : # otherwise take latest test flight
        dir_content = os.listdir("flightdata")
        files_only = list(filter(lambda x:os.path.isfile('flightdata/'+x), dir_content))
        files_list = list(map(lambda x:datetime.strptime(x,fdh.date_format), files_only))
        files_list.sort()
        experiment_name = files_list[-1].strftime(fdh.date_format)
        file_location = fdh.data_directory+"/"+ experiment_name

    data_storage = fdh()
    data_storage.open(file_location, experiment_name)
    print('* read data with length: \033[33m' + str(data_storage.trace_length) + '\033[0m')

    data_storage.show()
