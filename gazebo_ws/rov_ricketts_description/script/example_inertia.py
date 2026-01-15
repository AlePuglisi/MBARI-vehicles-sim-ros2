import trimesh

# Define cylinder parameters
radius = 0.05  # [m] radius of the cylinder
height = 0.2   # [m] height of the cylinder

density = 4500 #[kg/m^3]

# Create a cylinder mesh
cylinder = trimesh.creation.cylinder(radius=radius, height=height, sections=100) 
mass = cylinder.volume * density
V = cylinder.volume
# Display cylinder properties
print(f"Volume: {cylinder.volume} m^3")
print(f"Center of Mass: {cylinder.center_mass}")
inertia = cylinder.moment_inertia*mass/V
print(f"Inertia Matrix:\n{inertia}")

J = [[mass*(1/12*(height*height) + 1/4*(radius*radius)), 0, 0],[0, mass*(1/12*(height*height) + 1/4*(radius*radius)), 0],[0, 0, mass*1/2*(radius*radius)]]
print(f"Analythic Inertia Matrix:\n{J}")

# Visualize the cylinder
#cylinder.show()