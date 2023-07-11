# cp cfSimulator/Physics.py        cfSimulator/Physics.pyx
# cp cfSimulator/Controller.py     cfSimulator/Controller.pyx
# cp cfSimulator/StateEstimator.py cfSimulator/StateEstimator.pyx
# cp cfSimulator/Simulation.py     cfSimulator/Simulation.pyx

python3 setup.py build_ext --inplace
