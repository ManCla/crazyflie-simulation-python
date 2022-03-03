import sys
from utils.FlightDataHandler import FlightDataHandler

if __name__ == "__main__":

  # parsing command line parameters
  if len(sys.argv) != 3 or sys.argv[1] not in ("pdf", "show"):
    print('\033[91mError:\033[0m ' + 'python plot.py <1> <2>')
    print('  <1>: either "pdf" or "show"')
    print('  <2>: absolute or relative path of the experiment file')
    exit()

  command = sys.argv[1]
  file_location = sys.argv[2]
  experiment_name = file_location.split('/')[-1]
  data_storage = FlightDataHandler()
  data_storage.open(file_location, experiment_name)
  print('* read data with total length: \033[33m' + str(data_storage.trace_length) + '\033[0m')

  if command == "pdf":
    data_storage.pdf(experiment_name)

  if command == "show":
    data_storage.show()