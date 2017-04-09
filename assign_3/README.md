# CMU Robotics Institute 16-711:  Assignment 2 
# Arpit Agarwal, Adam Harley, Dhiraj Gandhi, Peiyun Hu
----
# Part 1
According to the tennis racket theorem, to maximally tumble the alien artifact should be given large angular velocity about the axis on which it is intermediate moment of inertia(y-axis) in this case. Therefore the unit vector can be (0.0011,1,0) in alien artifact coordinate system. With this angular velocity if the alien artifact is given a small flick in any other direction namely x or z, it will tumble maximally. To make the alien rotate about a fixed axis it should be given angular velocity only along a single axis x,y,z because initially the axis of alien are given to be aligned.
[![youtube link](http://img.youtube.com/vi/5jx8qBW6Ke4/0.jpg)](https://www.youtube.com/watch?v=5jx8qBW6Ke4). 

----
# Part 2 
----
## Part 2A
### Center of mass location
As there is no external force acting on the body, the center of mass will only be having constant translation velocity and there will be no effect of rotational velocity of body on its motion. We used this constraint to find a point in the body whose motion only gets affected by the translational velocity component between two frames. 
### Orientation of artifact
TO find out the orientation of the artifact we tried to find out the rotation matrix between the artifact and assigned co-ordinate system. To find out the rotation matix we take into account the marker location. As markers are placed at the corner of rectangular cuboid, its easy to get the orthogonal vector for the cordinate system which is rigidly attached to artifact in terms of assignemnt co-ordinate system. We selected the markers based on the intial quaternion information provided in the question. We calculted the rotation matrix for each time step by subtracting the position of 3 pair of markers. This rotation matrix is then converted to quaternion using internal matlab function rotm2quat.

## Part 2B Angular Velocity
To find out the angular velocity we used the identity property of rotation matrix i.e R*R^(-1) = I. If we take derivate this equation wrt to time we get R_dot = W x R. We converted this cross product to matrix multiplication to get W[skew_form] = R_inv * R_dot. We did this for each time step to get the angular velocity at each point.

## Part 2C Angular Acceleration
we used above calculated angular velocities and finite difference method to calculate the angular acceleration.

## Part 2D Inertia Tensor
We used the Iwdot + w x Iw = torue relation and zero external torque to find out the moment of inertia from the marker position. We tried to simplify the relation in the form A*b = 0, by taking into account that the inertia tensor is symmetric, where A is consist of both angular velocity and acceleration (we used above calculated values at their place) and b contains 6 element to express inertia. As b = 0  is a trivial solution of this problem, we did SVD decomposition of A to search in null space of A. We took the last column value of V matrix as a value of I.
The value of I =
    0.5960   -0.0001    0.0001
   -0.0001    0.1747    0.0010
    0.0001    0.0010    0.7838

## Part 2E Future trajectory calculation
We divided this problem into three sub problems
i) calculate the W_dot(T-1)
We used the relation IW_dot + WxIW to find the W_dot(T-1) from the W and I at T-1 time step.
ii) calculate W(T)
We used W_dot(T-1) and W(T-1) to find W(T) using euler integration method 
iii) calculate R(T+1)
We calculated R_dot(T) = W(T)xR(T) and wusing that R(T+1) by euler forward integration

![picture alt](https://raw.githubusercontent.com/Dhiraj100892/kdc_assignment/master/assign_3/2E/cm.jpg "Title is optional") 

![picture alt](https://raw.githubusercontent.com/Dhiraj100892/kdc_assignment/master/assign_3/2E/quat.jpg "Title is optional") 
 
# Part 3
For this part, the trajectory of the COM of the alien artifact was calculated by using the forward propagation using the parameters found in 2. A slight offset was added to the answer in part 2e such that the lander doesn't collide with the alien. The offset we used was [0,2.2,0] in body coordinate frame.
This trajectory was read from file named problem_3.dat into the simulate.c file. This trajectory was then provided as the desired trajectory of the lander in sim.lander_x_d and sim.lander_q_d for each simulation time step. 
For control the given PD control was used with slight change in k_x = 2.0, b_x = 2.5, k_r = 100.0, b_r = 20.0 for better performance.
[![youtube link](http://img.youtube.com/vi/lmFVAADCh6I/0.jpg)](https://www.youtube.com/watch?v=lmFVAADCh6I). 

