from robot import Robot
from planner import PathGenerator
from simulator import Simulator

def main():
    robot = Robot()
    pathgen = PathGenerator("../data/steps/cylinder.step")
    simulator = Simulator()

    segments = pathgen.generate_from_step(segment_file="segments.txt")
    simulator.simulate(segments, pause_time=0.005)

if __name__ == "__main__":
    main()
