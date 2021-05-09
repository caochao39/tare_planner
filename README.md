This is a ROS package that implements an exploration planner for ground robots.

## Usage
To setup TARE Planner, clone our repository.
```
git clone https://github.com/caochao39/tare_planner.git
```
In a terminal, go to the folder and checkout the correct branch. Replace '\<distribution\>' with 'melodic' or 'noetic'. Then, compile.

```
cd tare_planner
git checkout <distribution>
catkin_make
```
To run the code, go to the [development environment](cmu-exploration.com) folder in a terminal, source the ROS workspace, and launch.
```
source devel/setup.sh
roslaunch vehicle_simulator system_garage.launch
```
In another terminal, go to the TARE Planner folder, source the ROS workspace, and launch with the corresponding scenario.
```
source devel/setup.sh
roslaunch tare_planner ground_system.launch scenario:=garage
```
Now, users should see autonomous exploration in action. To launch with a different environment, use the command lines below instead and replace '\<environment\>' with one of the environment names in the development environment, i.e. 'campus', 'indoor', 'garage', 'tunnel', and 'forest'.
```
roslaunch vehicle_simulator system_<environment>.launch
roslaunch tare_planner ground_system.launch scenario:=<environment>
```
#### Launch with arguments
* rosbag_record: record a rosbag or not. If ```=true```, rosbags will be recorded to ```[home]/<bag_path>/<bag_name_prefix>_<timestamp>.bag``` with both the input and output topics from the planner. Note that '\<bag_path\>' and '\<bag_name_prefix\>' are both launch arguments that can be specified by the user.

* rviz: launch Rviz for visualization or not. If ```=true```, Rviz will be launched.

### Authors 
Chao Cao (ccao1@andrew.cmu.edu)\
Ji Zhang (zhangji@cmu.edu)

### Credit
[OR-Tools](https://developers.google.com/optimization) is from Google