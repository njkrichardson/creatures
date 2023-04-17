from abc import ABC, abstractmethod
from typing import Optional, List

import numpy as np 

from typedefs import ndarray

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
        self.sonar_radian_offsets: np.ndarray = np.array([0, 90, 180, 270])
        self.num_sensors: int = 2
        self.sonar_basis_vectors: np.ndarray = np.array([np.sin([0]), np.cos([0])]).T
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
        self.force_history: List[np.ndarray] = []

    def _feel_force(self, distances: np.ndarray) -> np.ndarray:

        force_per_sensor: np.ndarray = -1.0 / (distances.reshape((-1,1))+ 0.001)**2

        overall_force: np.ndarray = np.sum(self.sonar_basis_vectors * force_per_sensor, axis=0)
        print("overall",overall_force, distances)
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
        wander_heading = np.array([np.random.uniform(-1,1), np.random.uniform(-1,1)])
        wander_heading_normalized = wander_heading / np.linalg.norm(wander_heading)
        self.prev_wander = wander_heading_normalized
        return wander_heading_normalized

    def _avoid(self, force: np.ndarray, heading: np.ndarray):
        if np.linalg.norm(force + heading) > self.significant_force_threshold:
            return force + heading
        else:
            return np.zeros(2)

    def reset(self) -> None:
        self.prev_heading: ndarray = None
        # TODO implement me to reset any state variables
        pass

    def __call__(self, distances: ndarray):
        time = self.prev_time

        #halt: bool = self._collide(distances)
        # runaway_heading = self._runaway(force)
        #if np.linalg.norm(self.prev_heading) > 0 and halt:
        #    velocity = np.zeros(2)
        force: np.ndarray = self._feel_force(distances)
        print(time, "force experienced", force)
        if time - self.prev_wander_time >= 4.0:
            wander_heading = self._wander()
            self.prev_wander_time = time
        else:
            wander_heading = np.array([0, 1])

        print(time, "wander heading", wander_heading)
        self.wander_history.append(wander_heading)
        avoid_heading = self._avoid(force, wander_heading)
        print(time, "avoid heading", avoid_heading)
            #self.prev_avoid_heading = avoid_heading

            # if (time - self.prev_wander_time) > self.avoid_supress_time:
            #     velocity = runaway_heading
            #     print(time, "running away")
            # else:
        velocity = avoid_heading#self.prev_avoid_heading
            #print(time, "running away + avoiding")

        velocity = 0.1 * (velocity / (np.linalg.norm(velocity) if np.any(velocity != 0) else 1.0))
        self.prev_heading = velocity
        self.prev_time = time + 0.1
        print(velocity)
        return velocity
