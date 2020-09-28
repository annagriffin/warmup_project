# CompRobo: Warmup Project
##### Anna Griffin, Sherrie Shen
##### September 28, 2020

<br>

### Introduction
For the first project of the course Computational introduction to Robotics, our goal was to familiarize ourselves with using ROS to control a Neato robot vacuum in a virtual world. We programmed the robot to perform a variety of behaviors in response to both environmental cues or input control. The main topic that we utilized to achieve these behaviors was the \verb|cmd_vel| topic which communicates the robot's velocity. We also heavily relied on the Lidar sensor to give us an idea of the robot's surroundings. 

<br>

### Teleop
Our teleop node controls the robot from a variety of keyboard commands. We have chosen specific keys to map to different behaviors allowing us to control the robot with keyboard commands. This node has commands that can control both the movement of the robot and the speed. The \verb|cmd_vel| topic, of type Twist, was the only one we needed to publish for this node. This allowed us to modify the both the linear and angular velocities of the robot. While the program is running, it continuously checks to see if a key has been pressed. Upon detection of a valid key command, a new Twist object is created and the corresponding values from the look up dictionary are assigned. Movement bindings modify the x,y,z, and theta values while the speed bindings just take into account the linear and angular velocities. Once the new Twist has been initialized accordingly, we publish it in order for the robot to handle the change.

![alt text](img/keybindings.png "Figure 1")
Figure 1: Key binding mappings for direction and speed

<br>

### Driving in a Square
This node moves the robot in a 1m by 1m square using velocity and timing. The robot drives straight and then turns 90 degrees and continues until it has completed all four sides. Again, just the `cmd_vel` topic is being published since only the velocity needs to known and changed. We can set the distance it moves forward by using the equation \(distance = velocity \times time\). We know that we can change the velocity by publishing a new Twist object and use rospy timer package to control how much time elapses. When we want the robot to move in a straight line, we set the linear velocity to 1 for 1 second. Then we change just the angular velocity to 1 to initiate the turn. The Neato measures the angles in radians so we have it turn for $\frac{\pi}{2}$ seconds since the velocity is 1 radian per second and we want it to turn 90 degrees. Therefore, we only need to repeat the execution of two different steps. 

While the math that we did checks out for this behavior, it is not perfect since there is no way to determine whether or not the robot performed accurately. There is no feedback and factors like the surface on which it is running could potentially alter the precision of the calculations. We also could have used a loop in our \verb|run()| function to make it look a little bit neater. 

<br>

### Wall Following

<br>

### Person Following

<br>

### Obstacle Avoidance

<br>

### Combining Behaviors and Finite-State Control

<br>

### Improvement and Takeaways