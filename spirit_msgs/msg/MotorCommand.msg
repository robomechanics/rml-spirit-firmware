# This is a message to hold the desired position, feedforward torques and PD gains for a joint on Spirit
#
# Accurate timing information is stored in the header

Header header

float64 position # Position command
float64 torque_ff # Feedforward torque
float32 kp # Position gain on PD Controller
float32 kd # Derivative gain on PD Controller
