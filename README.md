# This package contains code for the paper: ["Towards Automated Scenario Testing of Social Navigation Algorithms"](https://unsolvedsocialnav.org/papers/Marpally.pdf)
A ROS2-Gazebo scenario generator for evaluating social navigation algorithms utilizing LLMs and [Hunavsim](https://github.com/robotics-upo/hunav_sim)
![](imgs/sample_generated_scenarios.jpeg)
## Installation Instructions
- This package is tested on ROS2 Humble, Gazebo v11.10.2 with python 3.9. Currently, we support querying any GPT-based model.
- Install the required python packages from requirements.txt

SoNAS consists of 3 submodules: 
    - A scenario generation module that queries LLMs in a structured manner to generate parameters and files for simulating a scenario
    - [Modified version of Hunavsim](https://github.com/robotics-upo/hunav_sim)
    - [Modified version of the Hunavsim wrapper for gazebo](https://github.com/robotics-upo/hunav_gazebo_wrapper)
    - For the 2 ROS2 packages, clone the submodules into your catkin workspace and build with the ROS2 ```colcon``` tool
## Generating your own scenarios
To generate scenarios in your own map, follow these steps:
1. run ``` python annotator.py --img=<path to map.pgm file> --out=<location for output image> --zoom=<required zoom>``` and follow the instructions to generate a map image annotated with a scene graph. An example is shown in the [warehouse annotated map](locations/Warehouse/scene_graph/scene_graph.png). 
2. Edit the ```inputs.yaml``` file and specify the parameters for generating the scenario. Remember to specify the paths for the ROS2 packages depending on your ROS2 workspace
3. Run the scenario generator with ``` python main.py ```
4. Build your ROS2 workspace with ``` colcon build ``` 
5. ``` ros2 launch hunav_gazebo_wrapper scenario.launch.py ```
6. Control the robot with teleop node. E.g.: ``` ros2 run turtlebot3_teleop teleop_keyboard ``` (we are adding support for running any ROS2 based navigation algorithm soon.)