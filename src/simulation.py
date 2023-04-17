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
    step_duration: float = 0.100 # [s] 

    def __init__(self, environment: Optional[Environment]=None, vehicles: Optional[Sequence[Vehicle]]=None, artifact_path: Optional[os.PathLike]=None) -> None: 
        self.current_step: int = 0 
        self.environment = environment 
        self.artifact_path = artifact_path

        if ((vehicles is not None) and (not isinstance(vehicles, list))): 
            self.vehicles = [vehicles]
        else:
            self.vehicles = vehicles

        self.prev_vehicle_velocities = [v.controller.prev_heading for v in self.vehicles]

        self.prev_vehicle_wander_headings = []
        self.prev_vehicle_force_headings = []


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

        fig, axs = plt.subplots(nrows=1, ncols=3)
        ax_env = axs[0]
        ax_force = axs[1]
        ax_headings = axs[2]
        max_heading = np.max(np.abs(np.array([self.prev_vehicle_wander_headings, self.prev_vehicle_force_headings])))
        def init():

            self.environment.draw(ax_env)
            ax_force.set_xlim((0, len(self.render_artifacts)*self.step_duration))
            ax_force.set_ylim((0, np.max(np.array(self.render_artifacts[-1][0].controller.force_mag_history))))
            ax_headings.set_xlim((-max_heading,max_heading))
            ax_headings.set_ylim((-max_heading,max_heading))

            return fig,

        def animate(i):
            ax_env.clear()
            ax_force.clear()
            ax_headings.clear()
            ax_force.set_xlim((0, len(self.render_artifacts)*self.step_duration))
            ax_force.set_ylim((0, np.max(np.array(self.render_artifacts[-1][0].controller.force_mag_history))))
            ax_headings.set_xlim((-max_heading, max_heading))
            ax_headings.set_ylim((-max_heading, max_heading))

            self.environment.draw(ax_env)
            vehicles: Sequence[Vehicle] = self.render_artifacts[i]
            for vehicle in vehicles: 
                vehicle.draw(ax_env)
                ax_force.plot(np.arange(i) * self.step_duration, vehicle.controller.force_mag_history[:i+1])
                ax_headings.arrow(0, 0, self.prev_vehicle_wander_headings[i][0], self.prev_vehicle_wander_headings[i][1], width=0.02,
                         color='green')
                ax_headings.arrow(0, 0, self.prev_vehicle_force_headings[i][0],
                                  self.prev_vehicle_force_headings[i][1], width=0.02,
                                  color='red')

            distance: float = vehicles[0].sensors[0].read()
            if isinstance(distance, np.ndarray):
                distance = distance[0]

            ax_env.set_title(f"distance: {distance:0.3f} [m]")
            return fig,

        animated = animation.FuncAnimation(fig, animate, init_func=init, frames=len(self.render_artifacts), interval=1, blit=True)
        animated.save(save_path, fps=30, extra_args=['-vcodec', 'libx264'], writer='ffmpeg')


    def create_animation2(self) -> None:
        save_path: os.PathLike = os.path.join(self.artifact_path, "animation2.mp4")

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


            distance: float = vehicles[0].sensors[0].read()
            if isinstance(distance, np.ndarray):
                distance = distance[0]

            ax.set_title(f"distance: {distance:0.3f} [m]")
            return fig,

        animated = animation.FuncAnimation(fig, animate, init_func=init, frames=len(self.render_artifacts), interval=1, blit=True)
        animated.save(save_path, fps=30, extra_args=['-vcodec', 'libx264'], writer='ffmpeg')

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

        for i, vehicle in enumerate(self.vehicles):
            if kwargs.get("save_artifacts", False): 
                self.save_render_artifacts()

            # move the vehicle based on its current velocity 
            try: 
                vehicle.position += vehicle.velocity * self.step_duration
                print("pos", vehicle.position)
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

            # in Vehicle basis
            control_signal: ndarray = vehicle.controller(distance_measurements)
            if isinstance(control_signal, list): control_signal = np.array(control_signal)
            # in world basis
            control_signal_world_basis = np.column_stack((rotation.dot(self.prev_vehicle_velocities[i]), self.prev_vehicle_velocities[i]))
            control_signal = control_signal_world_basis.dot(control_signal)
            control_signal = 0.1 * control_signal / (np.linalg.norm(control_signal) if np.any(control_signal != 0) else 1.0)
            vehicle.velocity = (vehicle.velocity + control_signal) / 2

            self.prev_vehicle_force_headings.append(control_signal_world_basis.dot(vehicle.controller.force_history[-1]))
            self.prev_vehicle_wander_headings.append(control_signal_world_basis.dot(vehicle.controller.wander_history[-1]))

            self.prev_vehicle_velocities[i] = vehicle.velocity / np.linalg.norm(vehicle.velocity) if np.any(vehicle.velocity != 0) else self.prev_vehicle_velocities[i]

            # in world basis
            control_signal_world_basis = np.column_stack((rotation.dot(self.prev_vehicle_velocities[i]), self.prev_vehicle_velocities[i])).dot(control_signal)
            control_signal_world_basis = 0.1 * control_signal_world_basis / (np.linalg.norm(control_signal_world_basis) if np.any(control_signal_world_basis != 0) else 1.0)
            vehicle.velocity = (vehicle.velocity + control_signal_world_basis) / 2

            self.prev_vehicle_velocities[i] = vehicle.velocity if np.any(vehicle.velocity != 0) else self.prev_vehicle_velocities[i]
 

        self.current_step += 1
