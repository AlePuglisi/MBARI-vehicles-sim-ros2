# Python script to compute links inertia from material density and mesh file
import trimesh
import numpy as np
from ament_index_python.packages import get_package_share_directory


# Compute TITAN 4 Inertia: 
density = 4.5  #[g/cm^3], Titanium

arm_links = ["arm_base", "arm_link1", "arm_link2", "arm_link3", "arm_link4", "arm_link5", "grip_claw", "grip_claw"]
#arm_CoM_links = ["arm_base_CoM", "arm_link1_CoM", "arm_link2_CoM", "arm_link3_CoM", "arm_link4_CoM", "arm_link5_CoM", "grip_claw_CoM", "grip_claw_CoM"]
volume = []
mass = []
CoM = []
inertia_matrix = []
#inertia_matrix_CoM = []

i = 0
for link in arm_links:
    mesh_path = get_package_share_directory('rov_ricketts_description')
    stl_file = mesh_path + "/meshes/" + link + ".stl"
    #stl_CoM_file = mesh_path + "/meshes/CoM_centered/" + link + "_CoM.stl"
    mesh = trimesh.load(stl_file)
    #mesh_CoM = trimesh.load(stl_CoM_file)
    volume.append(mesh.volume)              # [m^2]
    mass.append(mesh.volume*(density*1000)) # [Kg]
    CoM.append(mesh.center_mass)            # [m] (1x3)
    # inertia_matrix.append(mesh.moment_inertia) # [(Kg*m^2)/(kg*m^3)] (3x3) Normalized Inertia
    inertia_matrix.append(mesh.moment_inertia*mass[i]/volume[i]) # (3x3) [Kg*m^2] 
    #inertia_matrix_CoM.append(mesh_CoM.moment_inertia*mass[i]/volume[i])

    print(f"\nLink {link}: \n Volume= {volume[i]}\nMass= {mass[i]}\nCenter of Mass= {CoM[i]}\nInertia Matrix Origin= {inertia_matrix[i]}\n------------------------")#Inertia Matrix CoM= {inertia_matrix_CoM[i]}\n------------------")
    i+=1

print("\ntotal mass: " + str(sum(mass)))

# Compute BASE Inertia
base_mesh_name = "body"
# base mass taken from MBARI's website
base_mass = 4700
mesh_path = get_package_share_directory('rov_ricketts_description')
base_stl_file = mesh_path + "/meshes/stl/" + base_mesh_name + ".stl"
base_mesh = trimesh.load(base_stl_file)
base_volume = base_mesh.volume   
base_CoM = base_mesh.center_mass          # [m] (1x3)
base_inertia_matrix = base_mesh.moment_inertia*base_mass/base_volume # (3x3) [Kg*m^2] 

print(f"\nBase Link: \n Volume= {base_volume}\nMass= {base_mass}\nCenter of Mass= {base_CoM}\nInertia Matrix Origin= {base_inertia_matrix}\n------------------------")

# Compute Propeller Inertia
propeller_mesh_name = "propeller"
propeller_density = 2.70 #[g/cm^3], hyp: Alluminum 
mesh_path = get_package_share_directory('rov_ricketts_description')
propeller_stl_file = mesh_path + "/meshes/" + propeller_mesh_name + ".stl"
propeller_mesh = trimesh.load(propeller_stl_file)
propeller_volume = propeller_mesh.volume   
propeller_mass = propeller_volume * propeller_density * 1000 
propeller_CoM = propeller_mesh.center_mass          # [m] (1x3)
propeller_inertia_matrix = propeller_mesh.moment_inertia*propeller_mass/propeller_volume # (3x3) [Kg*m^2] 

print(f"\nPropeller Link: \n Volume= {propeller_volume}\nMass= {propeller_mass}\nCenter of Mass= {propeller_CoM}\nInertia Matrix Origin= {propeller_inertia_matrix}\n------------------------")

# EXPERIMENTS ON SEPRATED BASE MESHES

# Compute BASE-FLOAT separated Inertia
# base_mesh_name = "body_no_float"
# # base mass taken from MBARI's website
# base_mass = 4000
# mesh_path = get_package_share_directory('rov_ricketts_description')
# base_stl_file = mesh_path + "/meshes/stl/" + base_mesh_name + ".stl"
# base_mesh = trimesh.load(base_stl_file)
# base_volume = base_mesh.volume   
# base_CoM = base_mesh.center_mass          # [m] (1x3)
# base_inertia_matrix = base_mesh.moment_inertia*base_mass/base_volume # (3x3) [Kg*m^2] 

# frame_mesh_name = "float"
# # base mass taken from MBARI's website
# frame_mass = 700
# frame_path = get_package_share_directory('rov_ricketts_description')
# frame_file = frame_path + "/meshes/" + frame_mesh_name + ".dae"
# frame_mesh = trimesh.load(frame_file)
# frame_volume = frame_mesh.volume   
# frame_CoM = frame_mesh.center_mass          # [m] (1x3)
# frame_inertia_matrix = frame_mesh.moment_inertia*frame_mass/frame_volume # (3x3) [Kg*m^2] 

# print(f"\nBase Link: \n Volume= {base_volume}\nMass= {base_mass}\nCenter of Mass= {base_CoM}\nInertia Matrix Origin= {base_inertia_matrix}\n------------------------")
# print(f"\nFloat Link: \n Volume= {frame_volume}\nMass= {frame_mass}\nCenter of Mass= {frame_CoM}\nInertia Matrix Origin= {frame_inertia_matrix}\n------------------------")
