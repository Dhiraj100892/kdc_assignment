# CMU Robotics Institute 16-745: Dynamic Optimization: Assignment 1 
# Dhiraj Gandhi
#Part 1
To convert the IK solving problem into optimization problem, I defined  
### Optimization Objective Function (part_1/dist.m)
It includes to dist terms
* Dist from desired position
    * For translation part ([X Y Z]), I used L2 norm between desired and current position 
    * For orientation [q0 q1 q2 q3], I used difference between dot product of desired and current quaternion and 1 as measure of closeness.

    > To know the position and orientation of end effactor given the joint angle and link length, I reccursively used Forward Transformation matrix (part_1/forward_kine.m). This method provided the orientation of end effactor in roation matrix. To convert the roation matrix to quaternion I used the formula defines on this [site](http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/).       
* Dist from joint angles extremes
I used L2 norm from the mean position of joint angle as closesness measure. The 0.001 weight is applied to this criterion as compared to the above one. 


### Optimization Constraints
* In this I checked if the configuration of robot is colliding with obstacle (part_1/line_sphere_interection.m). If it does, I retunred high value, otherwise 0. To check line sphere intersection I used method defined on this [site](https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection)


### Testing
* For otimization purpose I used fmincon active set algorithm. The algorithm uses a Sequential Quadratic Programming method to solve Karush-Kuhn-Tucker(KKT) equations.
* I have tested it on the 3 link and 4 link snake robot.

[![Alt text](https://img.youtube.com/vi/V6zTDjGVavY/0.jpg)](https://www.youtube.com/watch?v=V6zTDjGVavY/) 
----

#Part 2

----
#Part 3

![picture alt](https://cloud.githubusercontent.com/assets/11137004/22453284/e064ae78-e74b-11e6-8baf-4bcd3acc7fa8.jpg "Title is optional")

----
#Part 4

![picture alt](https://cloud.githubusercontent.com/assets/11137004/22453630/1cab60b4-e74e-11e6-91a7-501725e50761.jpg "Title is optional") 

