# CMU Robotics Institute 16-711: Kinematics, Dynamic Systems, and Control: Assignment 1 
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

### Running the code
To run the code for 3 link arm, you can run part_1/que_1.m.

[![Alt text](https://img.youtube.com/vi/V6zTDjGVavY/0.jpg)](https://www.youtube.com/watch?v=V6zTDjGVavY/) 
----

#Part 2 Analytic Derivatives
With additional knowledge of derivatives of cost wrt. to joint parameters, optimization took less time, as solver won't spend time on calculating it via finite difference method. To culculate the derivatives I used symbolics to represent the end effactor in terms of joint angle varibles and used Jacobian to calculate the derivatives.  

In SQP there was not noticable difference in time for part_1 and part_2, but in case of interior point method, the derivative basec approach is ~2 faster. I think the derivative based approach will scale up well with increase in number of parameters(joints) as in that case calculating forward kinematics will be expensive and using that calculating gradient based on finite difference method will be much more expensive.

### Running the code
To run the code for 3 link arm, you can run part_2/que_2.m.

----
#Part 3
I comapre the active set, sequential quadratic programming & intrior point algotrithm in terms of computation time and solution quality. For comparison purposes I specify a desired position and orientation. I set maximum number of function evaluation to be 1000 for all algorithms. I found that the time taken and accuracy of solution is inversly proportional for each method. The time requirment increases from active-set to sqp to interior point and accuracy follows the exactly reverse order.

![picture alt](https://cloud.githubusercontent.com/assets/11137004/22453284/e064ae78-e74b-11e6-8baf-4bcd3acc7fa8.jpg "Title is optional")

----
#Part 4
In order to generate multiple trajectories I did the random restarts. I also additionaly stored the previously founded solution and used them in objective function such that new solution will try to stay away from it. This forces solver to search different space.

The follwing are the result for reaching the same goal state in 4 distinct orientation. 
![picture alt](https://cloud.githubusercontent.com/assets/11137004/22453630/1cab60b4-e74e-11e6-91a7-501725e50761.jpg "Title is optional") 

### Running the code
To get the above result, try to run part_4/que_4.m
