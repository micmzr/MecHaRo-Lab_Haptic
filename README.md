Integration of 3D Systems Geomagic Touch with the MoveIt servo. 
For limited use only with system installed in MecHaRo-Lab at Technical University of Gdańsk.  

IN DEVELOPMENT

**PC**

Everything presented here is mean to be run on the PC.
 
_Install_

``` bash
sudo apt install -y ros-$ROS_DISTRO-example-interfaces ros-$ROS_DISTRO-test-msgs ros-$ROS_DISTRO-test-interface-files ros-$ROS_DISTRO-ament-cmake-google-benchmar
git clone --recursive -b jazzy https://github.com/micmzr/MecHaRo-Lab_Haptic.git
python3 -m venv ./vev_cargo
source ./vev_cargo/bin/activate
pip install colcon-cargo colcon-ros-cargo
pip install numpy lark # workaround for venv
cd MecHaRo-Lab_Haptic/ws_ros2_hapitic
colcon build
source install/setup.sh
```

**_Run - example_**

_XBOX Controller_
``` bash
ros2 run joy joy_node
ros2 run haptic_servo joy_servo --ros-args -p ref_frame:=base_link  --ros-args -p eef_frame:=tool0
```
_MoveIT_
``` bash
ros2 run omni_common omni
ros2 run haptic_servo phantom_servo --ros-args -p ref_frame:=base_link  --ros-args -p eef_frame:=tool0
```
