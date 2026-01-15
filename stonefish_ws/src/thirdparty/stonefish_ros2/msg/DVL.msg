# Extracted from UUV Simulator:
#   https://github.com/uuvsimulator/uuv_simulator

# This is a message to hold data from a DVL sensor (Doppler Velocity Log).
#
# Distances are in [m], velocities in [m/s]
#
# If the covariance is known, it should be filled.
# If it is unknown, it should be set to all zeros.
# If a measurement was invalid, its covariance should be set to -1 so it can be
# disregarded.
#
# DVLBeams are optional. If they are set they contain individual ranges and 1D
# doppler velocity estimates orthogonal to the ray.

std_msgs/Header header
# Measured velocity [m/s]
geometry_msgs/Vector3 velocity
# Row major, xyz axes
float64[9] velocity_covariance
# Altitude of the vehicle
float64 altitude
stonefish_ros2/DVLBeam[] beams
