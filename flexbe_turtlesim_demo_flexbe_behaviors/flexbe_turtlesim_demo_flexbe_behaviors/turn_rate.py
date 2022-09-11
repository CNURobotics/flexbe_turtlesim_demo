#!/usr/bin/env python
"""
Simple calculate of parameters for FlexBE TurtleSim demo figure eight pattern.

Manually enter these into the FlexBE parameters for figure eight

"""
import math

desired_velocity = 0.5  # m/s
fwd_distance_traveled = 2.0 # m
desired_radius_of_curvature = 0.75 # m


print(f"Desired velocity = {desired_velocity} m/s")
print(f"Desired Forward travel  = {fwd_distance_traveled} m/s")
print(f"Desired radius of curvature  = {desired_radius_of_curvature} m")


fwd_travel_time = fwd_distance_traveled/desired_velocity
rate_of_turn = desired_velocity/desired_radius_of_curvature

print(f"Required Forward travel time  = {fwd_travel_time} seconds")
print(f"Required Rate of turn    = {rate_of_turn} radians/s")


# Angle between lines connecting the center of rotation to starting point and initial tangent point at start of turn
bisecting_angle = math.pi/2 - math.atan2(desired_radius_of_curvature, fwd_distance_traveled)

# Total turning angle
turning_angle = 2*math.pi - 2*bisecting_angle
turning_travel_time = turning_angle/rate_of_turn
print(f"Required turning time   = {turning_travel_time} seconds")
