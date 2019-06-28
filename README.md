scenario_n1_v1

To run:
In the code:
change nao_IP in main)scenario.py line 1009 to the current IP


In the launcher:
source ~/catkin_ws/devel/setup.bash

roslaunch scenario_n1_v1 camera_location.launch
roslaunch scenario_n1_v1 event_node.launch

For help:
python read_lines_location_event.py

For updating the Animations go to poses:
python setup.py --help
python setup.py install


go to sdk run ./naoqi
in chrographe connect to the pc IP