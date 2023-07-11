from setuptools import setup
from Cython.Build import cythonize
from cfSimulator import cfSimulation

ext_options = {"compiler_directives": {"profile": True}, "annotate": True, "language_level" : "3"}

ext_modules = ["cfSimulator/Physics.py",\
               "cfSimulator/Controller.py",\
               "cfSimulator/StateEstimator.py",\
               "cfSimulator/Simulation.py",\
               "cfSimulator/utils/PID.py",\
               # "cfSimulator/utils/lp2Filter.py",\
              ]

ext_modules = cythonize(ext_modules,**ext_options,)

setup(
    name='Crazyflie Simulator app',
    ext_modules = ext_modules,
    # zip_safe=False,
)