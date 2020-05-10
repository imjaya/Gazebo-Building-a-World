# Pursuit-Evasion-Game

# Task 1:
The house model in this task was built using a 3-bedroom layout of the Fleetwood apartments. We used the building editor in the gazebo environment to load in the 3b3b layout and set its scale. Scale is set by selecting a reference of an average American bedroom. Using the wall building tool in the building editor all the walls of the house have been built. The outer walls have been built with 1.5 m thickness and all the inner walls are 1m thick. Using the door and window tools all the doors and windows of rooms have been constructed. Once the model is complete, we exit the building editor, check all the parameters such as the global lighting, camera position and view type (i.e. perspective or orthographic) and save this world configuration. We save this file along with the house sdf model as a ‘. world’ file. We then include this file along with its path as a parameter in the empty world file in the gazebo_ros file.

# Task 2:
In this project we built two nodes, one for the movements of the turtle bot and the other for the camera functionality. In the first node created for turtle bot movements we subscribe to the rostopic ‘/odom’ to get the odometry messages of the turtle bot i.e. the position and the angular information.
The angular information of the turtle bot is in quaternion space and hence it needs to be converted to cartesian space to get the Euler angle yaw. In the first node we publish to the rostopic ‘/cmd_vel’ and precisely the ros message Twist. This message contains the linear and angular movement values of the turtle bot.
We have three main movement functions in the node, move, rotate and arc. The move function takes the speed, distance and a third argument for forward or backward directions and uses the odometry data to move the turtle bot by the given distance. The rotate function takes the angular speed, angle and a clockwise or anti-clockwise argument and rotates the turtle bot by that angle using the time library in python.
The arc function is a combination of these two functions where the function takes linear speed, angular speed, arc angle and clockwise arc or anti-clockwise as inputs. These movement functions publish the x values of the linear velocity and/or the z values of the angular velocity to control the movements of the turtle bot.
Since we decided to get a video of the turtle bot, we could not use the same node for streaming as this will create many timing issues. In the second node we subscribed to the rostopic ‘/camera/rgb/image_raw’ and get the ros image continuously. We implemented a subscriber callback where we are using an OpenCV library and cv bridge to convert the ros image to a cv image and display it with a 1 millisecond delay.
We created a launch file to launch the world file, turtle bot simulation, house movement node and the camera node.
Steps to Execute the file:
1) Un-zip and paste the ‘group_17_project_3a_resources’ file in the catkin_ws/src
2) Catkin_make
3) In a new terminal:
Export TURTLEBOT3_MODEL=waffle
roslaunch group_17_project_3a_resources project_3a.launch
Note: OpenCV and CVBridge libraries are required.
