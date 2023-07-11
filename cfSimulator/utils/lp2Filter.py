# import numpy as np
# import cython

# '''
#     Second order low pass filter used to filter derivative action
# '''
# @cython.cclass
# class lp2Filter():
#     b0: cython.double
#     b1: cython.double
#     b2: cython.double
#     a1: cython.double
#     a2: cython.double
#     x1: cython.double
#     x2: cython.double

#     def __init__(self, sample_freq: float, cutoff_freq: float):
#         fr: cython.double
#         fr   = sample_freq/cutoff_freq
#         ohm: cython.double
#         ohm  = np.tan(np.pi/fr)
#         ohm2: cython.double
#         ohm2 = ohm**2
#         c: cython.double
#         c    = 1+2*np.cos(np.pi/4)*ohm+ohm2
#         # coefficients
#         self.b0 = ohm2/c
#         self.b1 = 2*self.b0
#         self.b2 = self.b0
#         self.a1 = 2*(ohm2-1)/c
#         self.a2 = (1-2*np.cos(np.pi/4)*ohm+ohm2)/c
#         # init states
#         self.x1 = 0
#         self.x2 = 0

#     @cython.ccall
#     def filter(self, sample: float) -> float:
#         x0: cython.double
#         x0  = sample     - self.x1*self.a1 - self.x2*self.a2
#         out: cython.double
#         out = x0*self.b0 + self.x1*self.b1 + self.x2*self.b2
#         self.x2 = self.x1
#         self.x1 = x0
#         return out
