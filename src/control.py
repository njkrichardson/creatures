from abc import ABC, abstractmethod
from typing import Optional

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