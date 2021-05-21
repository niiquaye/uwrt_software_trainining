# uwrt_software_trainining

This is a solution of the software training with some added bugs that discourages one from copying and pasting the code. This code is meant to 
serve as a guide to those starting out the training. It makes use of some ROS2 concepts not covered in the training as it is meant to encourage you to 
go out and learn different features of ROS2.

This code covers core concepts of ROS2 such as 
components, topic statistic, callback groups, extending rclcpp::Node,
the creation of pubs/subs, services/clients, action servers, and multi-threaded callbacks.

The use of the turtlesim package is needed to complete this training. https://docs.ros.org/en/foxy/Tutorials/Turtlesim/Introducing-Turtlesim.html

**The training challenge consists of the following:**

**That training relies heavily on components - as this is the newer and better way to go about ROS2 design**

**Write 6 components that do the following: (Note: Some of the components do not need to be made in the following order)**
1. Clears any existing turtles
2. Create a component that moves 'turtle1' in a circular motion
3. Spawns a turtle named "stationary_turtle" at x = 5, y = 5
   Spawns a second turtle named "moving_turtle" at x = 25, y = 10
4. Create a service that resets the "moving_turtle" to its starting position. The service response should be whether or not it was successful.
5. Create a publisher that publishes a custom msg. This custom msgs should have 3 integer fields that correspond with the x and y distances of "stationary_turtle" to "moving turtle", as well as the distance between the two turtles.

6. Create an action that moves "moving_turtle" to a waypoint in a straight line by publishing geometry_msgs/Twist msgs to turtleX/cmd_vel.The action's goal command is an absolute position of the waypoint, feedback is distance to the goal, and result is the time it took to reach the goal. You should define a custom action file. 

Lastly, create a launch file that will start up all the components and the turtlesim node (configure the parameters of the turtlesim node to however you see fit). Ensure that the turtlesim node is launched first as the other components are dependent upon it. 


