# Find your printers max speed before losing steps
#
# Copyright (C) 2024 Anonoei <dev@anonoei.com>
#
# This file may be distributed under the terms of the MIT license.

import math

def calculate_accel_focused_dist(velocity: float, accel: float) -> float:
    """
    Calculates the exact distance needed to accelerate to a target velocity
    and immediately decelerate back to zero. This creates a "sharp" move.
    """
    if accel == 0:
        return float('inf')
    return (velocity**2) / accel

def calculate_velo_plateau_dist(velocity: float, accel: float, coast_distance: float) -> float:
    """
    Calculates the distance for an accel/decel phase PLUS a constant
    velocity "coast" phase. This creates a "smooth" move.
    """
    accel_dist = calculate_accel_focused_dist(velocity, accel)
    return accel_dist + coast_distance

def calculate_diagonal(x: float, y: float):
    """
    Calculates the length of a diagonal move. This function is useful and can remain.
    """
    return math.sqrt(x**2 + y**2)

def calculate_graph(velocity: float, slope: int):
    """
    This is a legacy function required by the original cmd_AUTO_SPEED_GRAPH.
    It calculates a heuristic for the min/max accel search bounds at a given velocity.
    It is restored here to ensure backward compatibility.
    """
    if velocity == 0 or slope == 0:
        return 100000 # Return a large number to avoid division by zero
    return (10000 / (velocity / slope))