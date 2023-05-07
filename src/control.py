from abc import ABC, abstractmethod
from typing import Optional, List

import numpy as np 

from src.typedefs import ndarray

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
        print("overall", overall_force)
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
        print(distances)
        #halt: bool = self._collide(distances)
        # runaway_heading = self._runaway(force)
        #if np.linalg.norm(self.prev_heading) > 0 and halt:
        #    velocity = np.zeros(2)

        # -- get raw repulsive force (sum over sensors)
        avoid_force: np.ndarray = self._feel_force(distances=distances)

        # -- record force and force magnitude
        self.force_history.append(avoid_force)
        self.force_mag_history.append(np.linalg.norm(avoid_force))

        print(time, "force experienced", avoid_force)

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

        print(time, "wander heading", wander_force)
        print(time, "avoid heading", velocity)
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
