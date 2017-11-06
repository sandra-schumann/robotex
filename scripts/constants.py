import math

ball_diameter = 4.0
goal_diameter = 16.0
goal_height = 80.0
field_width = 310.0
field_length = 460.0
robot_radius = 15.0

h = 32.0
a = 22.0
phi_deg = 51.0
phi = phi_deg*math.pi/180
dist_from_robot_midpoint = 9.0
ra_deg = 65.0
ra = ra_deg*math.pi/180
dist_correction = 9
ang_correction_deg = -1.5

turn_omega = 0.2
v = 0.2

robot_ID = 'C'
field_ID = 'B'

robot_start_x = field_length - robot_radius
robot_start_y = robot_radius
