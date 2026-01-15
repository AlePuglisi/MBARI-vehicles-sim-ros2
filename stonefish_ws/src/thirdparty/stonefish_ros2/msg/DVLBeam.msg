# Extracted from UUV Simulator:
#   https://github.com/uuvsimulator/uuv_simulator

# measured range [m] or < 0 if invalid
float64 range
float64 range_covariance

float64 velocity # measured velocity [m/s] of corr. beam
float64 velocity_covariance

# Beam link pose wrt DVL link frame
geometry_msgs/PoseStamped pose
