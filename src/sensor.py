from abc import ABC, abstractmethod
from typing import Optional

import numpy as np 
import numpy.random as npr

from typedefs import ndarray

class Sensor(ABC): 
    @abstractmethod 
    def reset(self) -> None: 
        raise NotImplementedError

    @abstractmethod 
    def read(self) -> ndarray: 
        raise NotImplementedError

    @abstractmethod 
    def write(self, value: ndarray) -> None: 
        raise NotImplementedError

class DistanceSensor(Sensor, ABC): 
    @property
    @abstractmethod
    def heading(self) -> ndarray: 
        raise NotImplementedError

    @heading.setter 
    @abstractmethod 
    def heading(self, new_heading: ndarray) -> None: 
        raise NotImplementedError

class HCS04(DistanceSensor): 
    minimum_range: float = 0.02 # [m]
    maximum_range: float = 4    # [m]
    noise_scale: float = 0.     # [m]

    def __init__(self): 
        self.reset()

    def __repr__(self) -> str: 
        return f"{self.__class__.__name__}(value={self._value}, noise_scale={self.noise_scale}, range=({self.minimum_range}, {self.maximum_range}))"

    def reset(self) -> None: 
        self._value: ndarray = np.zeros(1)
        self._heading: ndarray = np.zeros(2)

    @property 
    def heading(self) -> ndarray: 
        return self._heading 

    @heading.setter 
    def heading(self, new_heading: ndarray) -> None: 
        self._heading = new_heading

    def read(self) -> ndarray: 
        noise: ndarray = self.noise_scale * npr.randn()
        return self._value + noise

    def write(self, value: ndarray) -> None: 
        if (value < self.minimum_range): 
            self._value = np.zeros(1)
        elif (value > self.maximum_range): 
            self._value = np.ones(1)*100#self.maximum_range
        else: 
            self._value = value
