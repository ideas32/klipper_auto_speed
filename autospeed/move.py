# Find your printers max speed before losing steps
#
# Copyright (C) 2024 Anonoei <dev@anonoei.com>
# This file contains extensive modifications to support a new, safer, centered move pattern.
#
# This file may be distributed under the terms of the MIT license.

import math

class Move:
    home = [False, False, False]
    def __init__(self):
        self.dist = 0.0
        self.max_safe_dist: float = 0.0
        self.center = [None, None, None]
        self.corner_a = [None, None, None]
        self.corner_b = [None, None, None]

    def __str__(self):
        # A simple string representation for debugging if needed
        return (f"Move pattern: Center={self.center}, "
                f"CornerA={self.corner_a}, CornerB={self.corner_b}")

    def Init(self, axis_limits, margin, isolate_xy):
        # This method is now responsible for calculating the absolute safety limit for a move.
        pass

    def Calc(self, axis_limits, distance: float):
        # This method now takes a pre-calculated distance and determines the
        # coordinates for the safe, centered, reversing move pattern.
        pass

class MoveX(Move):
    def Init(self, axis_limits, margin, isolate_xy):
        home_y = not isolate_xy 
        self.home = [True, home_y, False]
        # Max safe distance is 75% of half the axis travel
        self.max_safe_dist = (axis_limits["x"]["dist"] / 2.0) * 0.75

    def Calc(self, axis_limits, distance: float):
        self.dist = min(distance, self.max_safe_dist)
        center_x = axis_limits["x"]["center"]
        center_y = axis_limits["y"]["center"]
        move_offset = self.dist / 2.0
        
        self.center = [center_x, center_y, None]
        self.corner_a = [center_x + move_offset, center_y, None]
        self.corner_b = [center_x - move_offset, center_y, None]

class MoveY(Move):
    def Init(self, axis_limits, margin, isolate_xy):
        home_x = not isolate_xy 
        self.home = [home_x, True, False]
        self.max_safe_dist = (axis_limits["y"]["dist"] / 2.0) * 0.75

    def Calc(self, axis_limits, distance: float):
        self.dist = min(distance, self.max_safe_dist)
        center_x = axis_limits["x"]["center"]
        center_y = axis_limits["y"]["center"]
        move_offset = self.dist / 2.0

        self.center = [center_x, center_y, None]
        self.corner_a = [center_x, center_y + move_offset, None]
        self.corner_b = [center_x, center_y - move_offset, None]

class MoveDiagX(Move):
    home = [True, True, False]
    def Init(self, axis_limits, margin, _):
        max_dist = min(axis_limits["x"]["dist"], axis_limits["y"]["dist"])
        self.max_safe_dist = (max_dist / 2.0) * 0.75

    def Calc(self, axis_limits, distance: float):
        self.dist = min(distance, self.max_safe_dist)
        center_x = axis_limits["x"]["center"]
        center_y = axis_limits["y"]["center"]
        # The distance to move from the center along one axis of the diagonal
        move_offset = (self.dist / 2.0) * math.sin(math.radians(45))

        self.center = [center_x, center_y, None]
        self.corner_a = [center_x + move_offset, center_y + move_offset, None]
        self.corner_b = [center_x - move_offset, center_y - move_offset, None]

class MoveDiagY(Move):
    home = [True, True, False]
    def Init(self, axis_limits, margin, _):
        max_dist = min(axis_limits["x"]["dist"], axis_limits["y"]["dist"])
        self.max_safe_dist = (max_dist / 2.0) * 0.75

    def Calc(self, axis_limits, distance: float):
        self.dist = min(distance, self.max_safe_dist)
        center_x = axis_limits["x"]["center"]
        center_y = axis_limits["y"]["center"]
        move_offset = (self.dist / 2.0) * math.sin(math.radians(45))

        self.center = [center_x, center_y, None]
        self.corner_a = [center_x - move_offset, center_y + move_offset, None]
        self.corner_b = [center_x + move_offset, center_y - move_offset, None]

class MoveZ(Move):
    home = [False, False, True]
    def Init(self, axis_limits, margin, _):
        self.max_safe_dist = (axis_limits["z"]["dist"] / 2.0) * 0.75

    def Calc(self, axis_limits, distance: float):
        self.dist = min(distance, self.max_safe_dist)
        center_z = axis_limits["z"]["center"]
        move_offset = self.dist / 2.0

        self.center = [None, None, center_z]
        self.corner_a = [None, None, center_z + move_offset]
        self.corner_b = [None, None, center_z - move_offset]