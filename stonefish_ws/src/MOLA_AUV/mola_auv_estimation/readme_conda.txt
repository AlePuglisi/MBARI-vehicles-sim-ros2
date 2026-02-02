# Running the code in a conda env  

# the conda env must have python>=3.12

conda activate env_name

source /opt/ros/jazzy/setup.bash

# Force ROS to use conda python packages 
export PYTHONPATH=$CONDA_PREFIX/lib/python3.12/site-packages:$PYTHONPATH

# source ros2 overlay 
source ~/project_ws/install/setup.bash

# run the node ...