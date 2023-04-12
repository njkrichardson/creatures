import copy 
import dataclasses
import os 
from typing import List, Optional, Sequence

import matplotlib   
from matplotlib import animation 
import matplotlib.pyplot as plt 
import numpy as np

from environment import Environment
from typedefs import ndarray
from vehicle import Vehicle

matplotlib.use("Agg")

class Simulator: 
    step_duration: float = 1.0 # [s] 

    def __init__(self, environment: Optional[Environment]=None, vehicles: Optional[Sequence[Vehicle]]=None, artifact_path: Optional[os.PathLike]=None) -> None: 
        self.current_step: int = 0 
        self.environment = environment 
        self.artifact_path = artifact_path

        if ((vehicles is not None) and (not isinstance(vehicles, list))): 
            self.vehicles = [vehicles]
        else: 
            self.vehicles = vehicles 

    def __repr__(self) -> str: 
        return f"{self.__class__.__name__}(environment={self.environment}, vehicles={self.vehicles})"

    @property
    def current_time(self) -> float: 
        return self.step_duration * self.current_step

    def save_render_artifacts(self) -> None: 
        if not hasattr(self, "render_artifacts"): 
            self.render_artifacts: List[Sequence[Vehicle]] = [] 

        self.render_artifacts.append(copy.deepcopy(self.vehicles))

    def render(self) -> None: 
        save_path: os.PathLike = os.path.join(self.artifact_path, f"step_{self.current_step}")

        figure, ax = plt.subplots(nrows=1, ncols=1)
        plt.title(f"Step {self.current_step}")

        self.environment.draw(ax)
        for vehicle in self.vehicles: 
            vehicle.draw(ax)

        plt.savefig(save_path)
        plt.close()

    def create_animation(self) -> None: 
        save_path: os.PathLike = os.path.join(self.artifact_path, "animation.mp4")

        fig, ax = plt.subplots(nrows=1, ncols=1)

        def init():
            self.environment.draw(ax)
            return fig,

        def animate(i):
            ax.clear()
            self.environment.draw(ax)
            vehicles: Sequence[Vehicle] = self.render_artifacts[i]
            for vehicle in vehicles: 
                vehicle.draw(ax)
            return fig,

        animated = animation.FuncAnimation(fig, animate, init_func=init, frames=len(self.render_artifacts), interval=1, blit=True)
        animated.save(save_path, fps=30, extra_args=['-vcodec', 'libx264'])

    def reset(self) -> None: 
        self.current_step: int = 0 
        self.render_artifacts = [] 
        for vehicle in self.vehicles: 
            vehicle.reset()

    def simulate(self, num_steps: int, **kwargs) -> None: 
        for _ in range(num_steps): 
            self.step(**kwargs)

    def step(self, **kwargs) -> None: 
        for vehicle in self.vehicles: 
            if kwargs.get("save_artifacts", False): 
                self.save_render_artifacts()

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
                sensor_position: ndarray = vehicle.position 
                sensor_heading: ndarray = sensor.heading
                sensor_reading: ndarray = self.environment.distance_to_boundary(sensor_position, sensor_heading)
                sensor.write(sensor_reading)
                distance_measurements[i] = sensor.read()

            control_signal: ndarray = vehicle.controller(distance_measurements)
            vehicle.velocity = control_signal 

        self.current_step += 1