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

    def reset(self) -> None: 
        # TODO implement me to reset any state variables 
        pass

    def __call__(self, distances: ndarray) -> ndarray: 
        if ((self.headings is None) or (self.headings.shape[0] != distances.size)): 
            raise AttributeError(f"headings attribute must be set and have equal size to distance argument.") 

        # TODO: write a repulsive control policy 

        return np.zeros(2)