### Optimazation to do Inverse Kinematics
#Part 1
To convert the IK solving problem into optimization problem, I defined a cost which take into account 
* Dist from desired position
    * For translation part ([X Y Z]), I used L2 norm between desired and current position 
    * For orientation [q0 q1 q2 q3], I used difference between dot product of desired and current quaternion and 1 as measure of closeness.

    > To know the position and orientation of end effactor given the joint angle and link length, I reccursively used Forward Transformation matrix (part_1/forward_kine.m). This method provided the orientation of end effactor in roation matrix. To convert the roation matrix to quaternion I used the formula defines on this [site](http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/).       
* Dist from joint angles extremes 

----

#Part 2



----
#Part 3



----
#PART 4
