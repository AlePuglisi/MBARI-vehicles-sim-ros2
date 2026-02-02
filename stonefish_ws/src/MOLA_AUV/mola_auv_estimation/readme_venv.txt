# Running the code in a venv

# e.g. To run the mola_auv_apriltag node :
# venv cretaed with python3-virtualenv

# venv creation
mkdir -p ~/ros2_ws_venv/src
cd ~/ros2_ws_venv
virtualenv -p python3 ./venv

# venv activation
source ~/ros2_ws_venv/venv/bin/activate

# install packages 
pip install apriltag 
pip install "numpy<2"

# Run the node in the venv 

export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages:$PYTHONPATH
source /opt/ros/jazzy/setup.bash
source ~/projects/MBARI-vehicles-sim-ros2/stonefish_ws/install/setup.bash

ros2 run mola_auv_estimation mola_auv_apriltag