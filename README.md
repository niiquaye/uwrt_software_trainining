# uwrt_software_trainining

The current working solution of the software training.
The use of the turtlesim package is needed to complete this training. https://docs.ros.org/en/foxy/Tutorials/Turtlesim/Introducing-Turtlesim.html
The training covers core concepts of ROS2 such as 
components, topic statistic, callback groups, extending rclcpp::Node, and 
the creation of pubs/subs, services/clients, action servers, multi-threaded callbacks.

**The training challenge consists of the following:**

**That training relies heavily on components as this is the newer and better way to go about ROS2 design**

**Write 6 components that do the following:**
1. Clears any existing turtles
2. Create a component that moves 'turtle1' in a circular motion
3. Spawns a turtle named "stationary_turtle" at x = 5, y = 5
   Spawns a second turtle named "moving_turtle" at x = 25, y = 10
4. Create a service that resets the "moving_turtle" to its starting position. The service response should be whether or not it was successful.
5. Create a publisher that publishes a custom msg. This custom msgs should have 3 integer fields that correspond with the x and y distances of "stationary_turtle" to "moving turtle", as well as the distance between the two turtles.

6. Create an action that moves "moving_turtle" to a waypoint in a straight line by publishing geometry_msgs/Twist msgs to turtleX/cmd_vel.The action's goal cd is an   absolute position of the waypoint, feedback is distance to the goal, and result is the time it took to reach the goal. You should define a custom action file. 


