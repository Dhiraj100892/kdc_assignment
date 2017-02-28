# CMU Robotics Institute 16-711:  Assignment 2 
# Arpit Agarwal, Adam Harley, Dhiraj Gandhi, Peiyun Hu
#Part 1
----
The Lagrangian method was used to derive the inverse dynamics of the system.
#Part 2 
----
The simulation was performed in ODE and visualization was made in pygame. A short demo video [![youtube link](http://img.youtube.com/vi/xkcGlrkoe14/0.jpg)](http://www.youtube.com/watch?v=xkcGlrkoe14). 
#Part 3
To manually find gains that kept the system upright, we did a grid search over large ranges for the four gain parameters. Gradually, we constrained the search until we were left with smaller parameter ranges, in which all configurations caused convergence.

We arrived at the following approximate parameter ranges, for the four parameters respectively: [-2,0], [-4,-2], [35,65], [15,20]. Any sample of parameters within those bounds produces a model that converges. Many such samples are plotted in the figure below. A sample near the middle of the bounds (-1,-2,50,20) is plotted in a thick red line. 

![picture alt](https://raw.githubusercontent.com/Dhiraj100892/kdc_assignment/master/assign_2/part_3/converging_gains.png "Converging gains") 

----
#Part 4 LQR to obtain control gains to keep system upright
To keep the pole in upright position we used lqr. To apply lqr we linearize the model when pole is in its vertical upright position. To obtain the optimal gain we used matlab build in lqr function. We used identity matrix as Q and fixed R to 0.1

Following graph depicts the perfromace of controller in making the pole upright when we start from different intial angle of pole
![picture alt](https://raw.githubusercontent.com/Dhiraj100892/kdc_assignment/master/assign_2/part_4/ang_comp.jpg "Title is optional") 

We used simscape to visualize the system behaviour under this controller.
## initial Angle 30 deg
[![Alt text](https://img.youtube.com/vi/r-_sYmAdkc8/0.jpg)](https://www.youtube.com/watch?v=r-_sYmAdkc8)

## initial Angle 60 deg
[![Alt text](https://img.youtube.com/vi/2qa0rhQoyfg/0.jpg)](https://www.youtube.com/watch?v=2qa0rhQoyfg)

## initial Angle 90 deg
[![Alt text](https://img.youtube.com/vi/RoG2T2gHZEw/0.jpg)](https://www.youtube.com/watch?v=RoG2T2gHZEw)

In case of intial angle equal to 90 degree, lqr completely failed to m make it upright

----
#Part 5

----
#Part 6 Swing Up problem

## Sol_1 - Iterative lqr
To find out the nominal trajectory which will swing the pendulum from stable down position to unstable upright position we used the Iterative lqr (ilqr). We started with intial guess (contol action 0 for all the time). We roll out the trajectory for this control action using system dynamics. Ilqr linearized the model along the btained trajectory and provided us the improved control action that will minimize the quadratic cost function. For implementation we refered to .[this paper](https://homes.cs.washington.edu/~todorov/papers/LiICINCO04.pdf) 

To make it upright we pot more weightage on angular position of pedulum to remmaining three states in Q.
Following graph shows the nominal trajectory obtained by ilqr  
![picture alt](https://github.com/Dhiraj100892/kdc_assignment/blob/master/assign_2/part_6/ilqr/ilqr.jpg?raw=true "Title is optional") 

### Animation 
[![Alt text](https://img.youtube.com/vi/u7QP_OsIim8/0.jpg)](https://www.youtube.com/watch?v=u7QP_OsIim8)

## Sol_2 - Direct collocation
To find the nominal trajectory we used the direct collocation method that helps to solve the differential equation to achive the desired state. We used [OptimTraj](https://github.com/MatthewPeterKelly/OptimTraj) library for matlab.

### Animation
[![Alt text](https://img.youtube.com/vi/vBi5RrMyUhY/0.jpg)](https://www.youtube.com/watch?v=vBi5RrMyUhY)


----
