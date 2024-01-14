import math

def shortest_angle_difference( angle1: float, angle2: float) -> float:
    '''Returns the shortest angle difference between two angles'''
    diff = (angle2 - angle1 + math.pi) % (math.pi * 2) - math.pi
    return diff