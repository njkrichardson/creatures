import argparse 
import os 

from src.custom_logging import setup_logger
from src.environment import Environment, BoxEnvironment
from src.simulation import Simulator
from src.vehicle import Vehicle, SimpleCar
from src.typedefs import namespace
from src.utils import setup_experiment_directory, get_now_str

parser = argparse.ArgumentParser()

# visuals
parser.add_argument("--save_animation", action="store_true", default=True)

def main(args: namespace): 
    # logging 
    experiment_directory: os.PathLike = setup_experiment_directory("avoid")
    log = setup_logger(__name__, custom_handle=os.path.join(experiment_directory, "log.out"))

    # configure the environment geometry 
    wall_length: float = 5.0 # [m]
    room: Environment = BoxEnvironment(wall_length)
    log.info(f"Environment: {room}")

    # configure the vehicle 
    vehicle: Vehicle = SimpleCar()
    log.info("configured vehicle")

    # set up the simulator 
    simulator: Simulator = Simulator(room, vehicle, experiment_directory)
    log.info("configured simulator")

    simulator.simulate(100, save_artifacts=args.save_animation)

    if args.save_animation: 
        log.info("animating simulation history")
        simulator.create_animation()
        log.info("finished animation")

if __name__=="__main__": 
    args = parser.parse_args()
    main(args)