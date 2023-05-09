from abc import ABC, abstractmethod
import ctypes 
import os 
from typing import Optional, List, Tuple

import numpy as np 

from typedefs import ndarray
from utils import PROJECT_DIRECTORY

class HCS04Controller(ABC): 
    def register_headings(self, headings: ndarray) -> None: 
        self.headings: ndarray = headings

    @abstractmethod 
    def reset(self) -> None: 
        raise NotImplementedError

    @abstractmethod 
    def __call__(self, distances: ndarray) -> ndarray: 
        """Accepts as input an n-vector of distance measurements 
        corresponding to the `n` registered headings, which are given 
        in radians relative to some reference vector, nominally the front
        facing normal of the vehicle.

        Parameters 
        ----------
        distances: ndarray
            an n-vector of distance measurements corresponding to the `n` registered 
            headings, which are given in radians relative to some reference vector, nominally 
            the front facing normal of the vehicle.

        Returns 
        -------
        new_velocity: ndarray 
            a 2-vector representing the velocity control signal. 
        """
        raise NotImplementedError 


class AvoidingController(HCS04Controller): 
    def __init__(self, headings: Optional[ndarray]=None) -> None: 
        self.headings = headings 
        
        UP: ndarray = np.array([0., 1.]) * 1e-1
        self.previous_velocity = UP

    def reset(self) -> None: 
        self.previous_velocity: ndarray = None
        # TODO implement me to reset any state variables 
        pass

    def __call__(self, distances: ndarray) -> ndarray: 
        if ((self.headings is None) or (self.headings.shape[0] != distances.size)): 
            raise AttributeError(f"headings attribute must be set and have equal size to distance argument.") 

        collision_threshold: float = 1e-1

        if np.all(distances > collision_threshold): 
            return self.previous_velocity
        else: 
            new_velocity: ndarray = -self.previous_velocity 
            self.previous_velocity = new_velocity
            return new_velocity


class Creature(HCS04Controller):
    def __init__(self):
        self.prev_wander_time: float = -10.0
        self.wander_period: float = 6.0
        self.sonar_radian_offsets: np.ndarray = np.array([0, np.pi/2, np.pi, 3*np.pi/2])
        self.num_sensors: int = 2
        self.sonar_basis_vectors: np.ndarray = np.array([np.sin(self.sonar_radian_offsets), np.cos(self.sonar_radian_offsets)], dtype=int).T
        self.collide_distance_threshold: float = 0.1
        self.runaway_force_threshold: float = 0.1
        self.significant_force_threshold: float = 0.0
        self.avoid_supress_time: float = 0.5
        self.prev_avoid_heading: np.ndarray = np.zeros(2)
        self.prev_heading: np.ndarray = np.array([0, 1])
        self.prev_time: float = 0.0
        self.prev_wander: np.ndarray = np.zeros(2)

        self.avoid_history: List[np.ndarray] = []
        self.wander_history: List[np.ndarray] = []
        self.force_mag_history: List[np.ndarray] = []
        self.force_history: List[np.ndarray] = []

    def _feel_force(self, distances: np.ndarray) -> np.ndarray:
        force_per_sensor: np.ndarray = -0.001 / (distances.reshape((-1,1))+ 0.001)**5
        overall_force: np.ndarray = np.sum(self.sonar_basis_vectors * force_per_sensor, axis=0)
        return overall_force

    def _collide(self, distances: np.ndarray) -> bool:
        halt: bool = distances[np.where(self.sonar_radian_offsets == 0.0)] < self.collide_distance_threshold
        return halt

    def _runaway(self, force: np.ndarray):
        if np.linalg.norm(force) > self.runaway_force_threshold:
            return force
        else:
            return np.zeros(2)

    def _wander(self) -> np.ndarray:
        wander_force = np.array([np.random.uniform(-1,1), np.random.uniform(-1,1)])
        wander_force_normalized = wander_force / np.linalg.norm(wander_force)
        self.prev_wander = wander_force_normalized
        return wander_force_normalized

    def _avoid(self, avoid_force: np.ndarray, wander_force: np.ndarray):
        combined = avoid_force + wander_force
        combined_magnitude = np.linalg.norm(combined)
        if combined_magnitude > self.significant_force_threshold:
            return combined / combined_magnitude
        else:
            return np.zeros(2)

    def reset(self) -> None:
        self.prev_heading: ndarray = None
        # TODO implement me to reset any state variables
        pass

    def __call__(self, distances: ndarray, time: float):
        print(f"distances: {distances}\ttime: {time:0.4f}")
        #halt: bool = self._collide(distances)
        # runaway_heading = self._runaway(force)
        #if np.linalg.norm(self.prev_heading) > 0 and halt:
        #    velocity = np.zeros(2)

        # -- get raw repulsive force (sum over sensors)
        avoid_force: np.ndarray = self._feel_force(distances=distances)

        # -- record force and force magnitude
        self.force_history.append(avoid_force)
        self.force_mag_history.append(np.linalg.norm(avoid_force))

        print(f"avoid force experienced: {avoid_force}")

        # -- generate new wander force (normalized) every wander period
        if time - self.prev_wander_time >= self.wander_period:
            wander_force = self._wander()
            self.prev_wander_time = time
        else:
            # -- default wander is to go straight (i.e. prev wander heading is followed)
            wander_force = np.array([0, 1])

        self.wander_history.append(wander_force)

        # -- combine wander and avoid forces, round to zero if threshold magnitude is not exceeded
        # -- vector resulting from combining forces and normalizing is the final velocity
        velocity = self._avoid(avoid_force=avoid_force, wander_force=wander_force)

        print(f"wander force: {wander_force}")
        print(f"combined wander/avoid (velocity): {velocity}")
            #self.prev_avoid_heading = avoid_heading

            # if (time - self.prev_wander_time) > self.avoid_supress_time:
            #     velocity = runaway_heading
            #     print(time, "running away")
            # else:
        #velocity = heading#self.prev_avoid_heading
            #print(time, "running away + avoiding")



        self.prev_heading = velocity
        self.prev_time = time
        return velocity

class CreatureC(ctypes.Structure): 
    _fields_: List[Tuple] = [
        ('collide_distance_threshold', ctypes.c_double), 
        ('runaway_force_threshold', ctypes.c_double), 
        ('significant_force_threshold', ctypes.c_double), 
        ('avoid_supress_time', ctypes.c_double), 
        ('previous_wander_time', ctypes.c_double), 
        ('previous_avoid_heading', ctypes.POINTER(ctypes.c_double * 2)), 
        ('previous_heading', ctypes.POINTER(ctypes.c_double * 2)), 
        ('previous_wander', ctypes.POINTER(ctypes.c_double * 2)), 
        ('previous_time', ctypes.c_double), 
        ('wander_period', ctypes.c_int), 
        ('num_sensors', ctypes.c_int), 
        ('sonar_radian_offsets', ctypes.POINTER(ctypes.c_int * 4)), 
        ('sonar_basis_vectors', ctypes.POINTER(ctypes.POINTER(ctypes.c_double * 2) * 4))
    ]

class CreatureCInterface(HCS04Controller): 
    def __init__(self): 
        # initialize DLL 
        self._initialize_shared_object()
        self.c_controller: ctypes.Structure = CreatureC()
        self.shared_object.initialize_controller_default(ctypes.byref(self.c_controller))

        # state for rendering animations
        self.avoid_history: List[np.ndarray] = []
        self.wander_history: List[np.ndarray] = []
        self.force_mag_history: List[np.ndarray] = []
        self.force_history: List[np.ndarray] = []

    @property 
    def prev_heading(self) -> np.ndarray: 
        return self.c_to_ndarray(self.c_controller.previous_heading)

    def __getstate__(self) -> dict: 
        state: dict = self.__dict__.copy()
        del state["shared_object"]
        del state["c_controller"]
        return state

    def __setstate__(self, state: dict) -> None: 
        self.__dict__.update(state)

    def _initialize_shared_object(self) -> None: 
        library_path: os.PathLike = os.path.join(PROJECT_DIRECTORY, "control_c.cpython-39-darwin.so")
        self.shared_object = ctypes.CDLL(library_path)
        self.shared_object.feel_force.restype = ctypes.POINTER(ctypes.c_double * 2)
        self.shared_object.avoid.restype = ctypes.POINTER(ctypes.c_double * 2)
        self.shared_object.wander.restype = ctypes.POINTER(ctypes.c_double * 2)

    def c_to_ndarray(self, pointer: ctypes.POINTER) -> np.ndarray: 
        return np.array([value for value in pointer.contents])

    def ndarray_to_c(self, ndarray: np.ndarray, as_ptr: Optional[bool]=True) -> ctypes.POINTER: 
        ndarray_c = (ctypes.c_double * ndarray.size)()
        ndarray_c[:] = ndarray
        if as_ptr: 
            ndarray_c_pointer = ctypes.cast(ndarray_c, ctypes.POINTER(ctypes.c_double))
            return ndarray_c_pointer
        else: 
            return ndarray_c

    def __call__(self, distances: ndarray, time: float):
        print(f"distances: {distances}\ttime: {time:0.4f}")
        # -- get raw repulsive force (sum over sensors)
        distances_c = (ctypes.c_double * 4)()
        distances_c[:] = distances 
        distances_c_pointer = ctypes.cast(distances_c, ctypes.POINTER(ctypes.c_double))

        avoid_force: np.ndarray = self.c_to_ndarray(self.shared_object.feel_force(ctypes.byref(self.c_controller), self.ndarray_to_c(distances)))

        # -- record force and force magnitude
        self.force_history.append(avoid_force)
        self.force_mag_history.append(np.linalg.norm(avoid_force))

        print(f"avoid force experienced: {avoid_force}")

        # -- generate new wander force (normalized) every wander period
        if time - self.c_controller.previous_wander_time >= self.c_controller.wander_period:
            wander_force = self.c_to_ndarray(self.shared_object.wander(ctypes.byref(self.c_controller)))
            self.c_controller.previous_wander_time = time
        else:
            # -- default wander is to go straight (i.e. prev wander heading is followed)
            wander_force = np.array([0, 1])

        self.wander_history.append(wander_force)

        # -- combine wander and avoid forces, round to zero if threshold magnitude is not exceeded
        # -- vector resulting from combining forces and normalizing is the final velocity
        velocity = self.c_to_ndarray(self.shared_object.avoid(ctypes.byref(self.c_controller), self.ndarray_to_c(avoid_force), self.ndarray_to_c(wander_force)))

        print(f"wander force: {wander_force}")
        print(f"combined wander/avoid (velocity): {velocity}")

        self.c_controller.previous_heading = ctypes.pointer((ctypes.c_double * 2)(*velocity))
        self.c_controller.previous_time = time
        return velocity

    def reset(self) -> None:
        pass 
