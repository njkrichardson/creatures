import argparse 
import os 
from typing import Sequence

import numpy as np 

from custom_logging import setup_logger
from environment import Environment, BoxEnvironment, CompositeEnvironment
from simulation import Simulator
from vehicle import Vehicle, SimpleCar
from typedefs import namespace
from utils import setup_experiment_directory, get_now_str

parser = argparse.ArgumentParser()

# visuals
parser.add_argument("--save_animation", action="store_true")
parser.add_argument("--num_steps", type=int, default=500)

def main(args: namespace): 
    # logging 
    experiment_directory: os.PathLike = setup_experiment_directory("avoid")
    log = setup_logger(__name__, custom_handle=os.path.join(experiment_directory, "log.out"))

    # configure the environment geometry 
    wall_length: float = 2.0 # [m]
    obstacles: Sequence[BoxEnvironment] = [BoxEnvironment(0.5)]
    obstacle_locations: np.ndarray = np.array([
        [0.5, 0.0]
    ])
    room: Environment = CompositeEnvironment(obstacles, obstacle_locations, wall_length)
    # room: Environment = BoxEnvironment(wall_length)
    log.info(f"Environment: {room}")

    # configure the vehicle 
    vehicle: Vehicle = SimpleCar()
    log.info("configured vehicle")

    # set up the simulator 
    simulator: Simulator = Simulator(room, vehicle, experiment_directory)
    log.info("configured simulator")

    simulator.simulate(args.num_steps, save_artifacts=args.save_animation)

    if args.save_animation:
        log.info("animating simulation history")
        simulator.create_animation()
        log.info("finished animation")

if __name__=="__main__": 
    args = parser.parse_args()
    main(args)