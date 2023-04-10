import argparse 

from environment import Environment, BoxEnvironment
from simulation import Simulator
from vehicle import Vehicle, SimpleCar
from typedefs import namespace

parser = argparse.ArgumentParser()

def main(args: namespace): 
    # configure the environment geometry 
    wall_length: float = 5.0 # [m]
    room: Environment = BoxEnvironment(wall_length)

    # configure the vehicle 
    vehicle: Vehicle = SimpleCar()

    # set up the simulator 
    simulator: Simulator = Simulator(room, vehicle)

    simulator.render()
    simulator.step()
    simulator.render()

if __name__=="__main__": 
    args = parser.parse_args()
    main(args)
