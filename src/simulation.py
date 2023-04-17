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
from vehicle import Vehicle, SimpleCar

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
        
        self.prev_vehicle_velocities = [v.controller.prev_heading for v in self.vehicles]

    def __repr__(self) -> str: 
        return f"{self.__class__.__name__}(environment={self.environment}, vehicles={self.vehicles})"

    @property
    def current_time(self) -> float: 
        return self.step_duration * self.current_step

    def save_render_artifacts(self) -> None: 
        if not hasattr(self, "render_artifacts"): 
            self.render_artifacts: List[Sequence[Vehicle]] = [] 

        vehicle_copies: Sequence[Vehicle] = [] 
        for vehicle in self.vehicles: 
            try: 
                # TODO NJKR be smarter...
                vehicle_copy: Vehicle = SimpleCar() 
                vehicle_copy.position = vehicle.position 
                vehicle_copy.velocity = vehicle.velocity 
                vehicle_copy._heading = vehicle._heading
                vehicle_copies.append(vehicle_copy)
            except AttributeError as e: 
                raise e

        self.render_artifacts.append(vehicle_copies)

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

        interval: int = max(1, len(self.render_artifacts) / 20)
        frames: int = int(max(1, len(self.render_artifacts) / 20))

        animated = animation.FuncAnimation(fig, animate, init_func=init, frames=frames, interval=1, blit=True)
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
        rotation = np.array([[0, 1], [-1, 0]])
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
            if isinstance(control_signal, list): control_signal = np.array(control_signal)

            # in world basis
            control_signal_world_basis = np.column_stack((rotation.dot(self.prev_vehicle_velocities[i]), self.prev_vehicle_velocities[i])).dot(control_signal)
            control_signal_world_basis = 0.1 * control_signal_world_basis / (np.linalg.norm(control_signal_world_basis) if np.any(control_signal_world_basis != 0) else 1.0)
            vehicle.velocity = (vehicle.velocity + control_signal_world_basis) / 2

            self.prev_vehicle_velocities[i] = vehicle.velocity if np.any(vehicle.velocity != 0) else self.prev_vehicle_velocities[i]
 

        self.current_step += 1
