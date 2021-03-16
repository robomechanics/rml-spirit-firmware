# This is a message to hold the state of a single foot of a legged robot
#
# The states of each foot are stored in a vector of FootState messages
# Accurate timing information is stored in the header

Header header
float64 solve_time
float64 plan_length
float64 plan_duration
float64 vertices_generated