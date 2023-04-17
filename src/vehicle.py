from abc import ABC, abstractmethod
from typing import Sequence

import matplotlib.pyplot as plt 
import numpy as np 

from control import HCS04Controller, AvoidingController
from control_c import Controller 
from sensor import Sensor, HCS04
from typedefs import ndarray 

class Vehicle(ABC): 
    @property
    @abstractmethod 
    def position(self) -> ndarray: 
        raise NotImplementedError

    @property
    @abstractmethod 
    def velocity(self) -> ndarray: 
        raise NotImplementedError

    @position.setter
    @abstractmethod
    def position(self, new_position: ndarray) -> None: 
        raise NotImplementedError

    @velocity.setter
    @abstractmethod 
    def velocity(self, new_velocity: ndarray) -> ndarray: 
        raise NotImplementedError

    @abstractmethod 
    def draw(self, ax) -> None: 
        raise NotImplementedError

class SimpleCar(Vehicle): 
    def __init__(self) -> None: 
        # private 
        self._position: ndarray = np.zeros(2)
        self._velocity: ndarray = np.zeros(2)
        self._heading: ndarray = np.array([0., 1.])

        # publice: sensor/control suite 
        self.sensors: Sequence[Sensor] = [HCS04()] 
        self.controller: HCS04Controller = Controller()

        self.configure_sensors()
        self.configure_controller()

    def reset(self) -> None: 
        self._position: ndarray = np.zeros(2)
        self._velocity: ndarray = np.zeros(2)
        self._heading: ndarray = np.zeros(2)
        self.controller.reset()
        self.sensors.reset()

    @property 
    def position(self) -> ndarray: 
        return self._position

    @property 
    def velocity(self) -> ndarray: 
        return self._velocity 

    @position.setter
    def position(self, new_position: ndarray) -> None: 
        self._position = new_position 

    @velocity.setter 
    def velocity(self, new_velocity: ndarray) -> ndarray: 
        self._velocity = new_velocity 

        if np.linalg.norm(new_velocity): 
            self._heading = new_velocity / np.linalg.norm(new_velocity)
            # TODO generalize to multiple sensors to have distince (but relative fixed) headers
            for sensor in self.sensors: 
                sensor.heading = self._heading

    def configure_sensors(self) -> None: 
        # TODO generalize to multiple sensors to have distince (but relative fixed) headers
        for sensor in self.sensors: 
            sensor.heading = self._heading

    def configure_controller(self) -> None: 
        pass
        # self.controller.register_headings(np.array([sensor.heading for sensor in self.sensors]))

    def draw(self, ax) -> None: 
        ax.scatter(self.position[0], self.position[1], marker="o", s=100)
        ax.arrow(self.position[0], self.position[1], self._heading[0] / 5., self._heading[1] / 5., width=0.02, color='k')
        ax.arrow(self.position[0], self.position[1], self.velocity[0], self.velocity[1], width=0.02, color='tab:red')