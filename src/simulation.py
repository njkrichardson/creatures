import dataclasses
import os 
from typing import Optional, Sequence

import matplotlib   
import matplotlib.pyplot as plt 
import numpy as np

from environment import Environment
from typedefs import ndarray
from vehicle import Vehicle

matplotlib.use("Agg")

class Simulator: 
    step_duration: float = 1.0 # [s] 

    def __init__(self, environment: Optional[Environment]=None, vehicles: Optional[Sequence[Vehicle]]=None) -> None: 
        self.current_step: int = 0 
        self.environment = environment 

        if ((vehicles is not None) and (not isinstance(vehicles, list))): 
            self.vehicles = [vehicles]
        else: 
            self.vehicles = vehicles 

    def __repr__(self) -> str: 
        return f"{self.__class__.__name__}(environment={self.environment}, vehicles={self.vehicles})"

    @property
    def current_time(self) -> float: 
        return self.step_duration * self.current_step

    def render(self) -> None: 
        save_path: os.PathLike = f"step_{self.current_step}"

        figure, ax = plt.subplots(nrows=1, ncols=1)
        plt.title(f"Step {self.current_step}")

        self.environment.draw(ax)
        for vehicle in self.vehicles: 
            vehicle.draw(ax)

        plt.savefig(save_path)
        plt.close()

    def reset(self) -> None: 
        self.current_step: int = 0 
        for vehicle in self.vehicles: 
            vehicle.reset()

    def simulate(self, num_steps: int) -> None: 
        for _ in range(num_steps): 
            self.step()

    def step(self) -> None: 
        for vehicle in self.vehicles: 
            # move the vehicle based on its current velocity 
            try: 
                vehicle.position += vehicle.velocity * self.step_duration
                if (not self.environment.inside(vehicle.position)): 
                    raise ValueError
            except ValueError: 
                raise ValueError(f"Collision detected: tried to move vehicle to position: {vehicle.position}")

            # take a distance measurement from this position 
            distance_measurements = np.zeros(len(vehicle.sensors))

            for i, sensor in enumerate(vehicle.sensors): 
                # TODO time mux this w.r.t. sensor sampling rates, but there's some added complexity 
                sensor_position: ndarray = vehicle.position 
                sensor_heading: ndarray = sensor.heading
                sensor_reading: ndarray = self.environment.distance_to_boundary(sensor_position, sensor_heading)
                sensor.write(sensor_reading)
                distance_measurements[i] = sensor.read()

            control_signal: ndarray = vehicle.controller(distance_measurements)
            vehicle.velocity = control_signal 

        self.current_step += 1