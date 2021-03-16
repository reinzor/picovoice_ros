# Picovoice ROS

ROS wrappers for the Picovoice libraries

## Installation

```
roscd && cd ../src  # Navigate to the 'src' dir of your catkin workspace 
git clone https://github.com/reinzor/picovoice_ros.git
cd picovoice_ros
rosdep install --from-path -y -i .
catkin_make  # Compile workspace (or catkin build)
```
