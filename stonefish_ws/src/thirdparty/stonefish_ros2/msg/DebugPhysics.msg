std_msgs/Header header

float64 mass
float64 volume
float64 surface

# In local frame (origin)
geometry_msgs/Vector3 cog
geometry_msgs/Vector3 cob
geometry_msgs/Vector3 inertia
geometry_msgs/Vector3 damping_coeff
geometry_msgs/Vector3 skin_friction_coeff

geometry_msgs/Twist velocity

geometry_msgs/Wrench buoyancy
geometry_msgs/Wrench damping
geometry_msgs/Wrench skin_friction

float64 wetted_surface
float64 submerged_volume