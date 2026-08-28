from typing import Tuple
from algorithms import utils
from algorithms.problems import SystemRepairProblem
from math import sqrt


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def manhattanHeuristic(state, problem):
    """
    The Manhattan distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    pos = state[0]
    hasKit = state[1]
    pendingSystems = state[2]
    
    if not hasKit:
        return abs(pos[0] - problem.kitPosition[0]) + abs(pos[1] - problem.kitPosition[1])
    
    elif len(pendingSystems) != 0:
        closest_t = pendingSystems[0]
        closest_t_distance = abs(pos[0] - closest_t[0]) + abs(pos[1] - closest_t[1])
        
        for t in pendingSystems:
            current_t_distance = abs(pos[0] - t[0]) + abs(pos[1] - t[1])
            if closest_t_distance > current_t_distance:
                closest_t = t
                closest_t_distance = current_t_distance
        
        return closest_t_distance
    
    else:
        return abs(pos[0] - problem.controlPosition[0]) + abs(pos[1] - problem.controlPosition[1])
        


def euclideanHeuristic(state, problem):
    """
    The Euclidean distance heuristic.

    Baseline rule for this workshop: estimate the direct distance to the next
    mandatory target:
    - K if the robot does not have the kit yet.
    - the nearest pending T if the robot has the kit and systems remain.
    - C if all systems have been repaired.
    """
    pos = state[0]
    hasKit = state[1]
    pendingSystems = state[2]
    
    if not hasKit:
        return sqrt((pos[0] - problem.kitPosition[0])**2 + (pos[1] - problem.kitPosition[1])**2)
    
    elif len(pendingSystems) != 0:
        closest_t = pendingSystems[0]
        closest_t_distance = sqrt((pos[0] - closest_t[0])**2 + (pos[1] - closest_t[1])**2)
        
        for t in pendingSystems:
            current_t_distance = sqrt((pos[0] - t[0])**2 + (pos[1] - t[1])**2)
            if closest_t_distance > current_t_distance:
                closest_t = t
                closest_t_distance = current_t_distance
        
        return closest_t_distance
    
    else:
        return sqrt((pos[0] - problem.controlPosition[0])**2 + (pos[1] - problem.controlPosition[1])**2)


def systemRepairHeuristic(
    state: Tuple[Tuple, bool, Tuple], problem: SystemRepairProblem
):
    """
    Your heuristic for the SystemRepairProblem.

    state: (position, hasKit, pendingSystems)
    problem: SystemRepairProblem instance

    This must be admissible and preferably consistent.

    Hints:
    - Use problem.heuristicInfo to cache expensive computations
    - Go with some simple heuristics first, then build up to more complex ones
    - Consider the kit, pending systems, and the final return to control center
    - Balance heuristic strength vs. computation time (do experiments!)
    """
    # TODO: Add your code here
    utils.raiseNotDefined()
