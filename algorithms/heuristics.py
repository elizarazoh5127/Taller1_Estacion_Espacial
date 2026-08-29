from typing import Tuple
from algorithms import utils
from algorithms.problems import SystemRepairProblem
from math import sqrt

def manhattan_distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

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
        return manhattan_distance(pos, problem.kitPosition)
    
    elif len(pendingSystems) != 0:
        closest_t = pendingSystems[0]
        closest_t_distance = manhattan_distance(pos, closest_t)
        
        for t in pendingSystems:
            current_t_distance = abs(pos[0] - t[0]) + abs(pos[1] - t[1])
            if closest_t_distance > current_t_distance:
                closest_t = t
                closest_t_distance = current_t_distance
        
        return closest_t_distance
    
    else:
        return manhattan_distance(pos, problem.controlPosition)
        


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
    
    elif pendingSystems:
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


def systemRepairHeuristic(state: Tuple[Tuple, bool, Tuple], problem: SystemRepairProblem):
    """
    Distancia al punto obligatorio más cercano (K o T) + la distancia directa de ahí a C. 
    Admisible porque nunca sobreestima y a diferencia de manhattan/euclidean sí cuenta el regreso a C.
    """
    position, hasKit, pendingSystems = state

    if problem.isGoalState(state):
        return 0

    if not hasKit:
        nextPoint = problem.kitPosition
    
    elif pendingSystems:
       
        nextPoint = pendingSystems[0]
        nextPointTotal = manhattan_distance(position, nextPoint) + manhattan_distance(nextPoint, problem.controlPosition)

        for t in pendingSystems:

            t_total = manhattan_distance(position, t) + manhattan_distance(t, problem.controlPosition)
            if t_total < nextPointTotal:
                nextPoint = t
                nextPointTotal = t_total
    
    else:
        nextPoint = problem.controlPosition

    return manhattan_distance(position, nextPoint) + manhattan_distance(nextPoint, problem.controlPosition)
