import ctypes 
import os 
from typing import List, Tuple

class Controller(ctypes.Structure): 
    _fields_: List[Tuple] = [
        ('collide_distance_threshold', ctypes.c_double), 
        ('runaway_force_threshold', ctypes.c_double), 
        ('significant_force_threshold', ctypes.c_double), 
        ('avoid_supress_time', ctypes.c_double), 
        ('previous_wander_time', ctypes.c_double), 
        ('previous_avoid_heading', ctypes.POINTER(ctypes.c_double)), 
        ('previous_heading', ctypes.POINTER(ctypes.c_double)), 
        ('previous_wander', ctypes.POINTER(ctypes.c_double)), 
        ('previous_time', ctypes.c_double), 
        ('wander_period', ctypes.c_int), 
        ('num_sensors', ctypes.c_int), 
        ('sonar_radian_offsets', ctypes.POINTER(ctypes.c_int * 4)), 
        ('sonar_basis_vectors', ctypes.POINTER(ctypes.POINTER(ctypes.c_double)))
    ]

library_path: os.PathLike = "./controller.dylib"
shared_object = ctypes.CDLL(library_path)

controller: Controller = Controller()
initialize_controller_default: callable = shared_object.initialize_controller_default
initialize_controller_default(ctypes.byref(controller))

print([controller.sonar_radian_offsets.contents[i] for i in range(4)])
dummy: int = 5
