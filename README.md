# Laser-Based-Perception-and-Navigation-with-Obstacle-Avoidance
The objective of this project is to perform perception using a laser range finder, and use the perceived information to avoid obstacles and navigate to a given destination. 

**Perception Using Laser Range Finder**

For this section, we will implement the RANSAC algorithm to determine the walls “visible” to the robot from the data obtained from the laser range finder. Our program will take the laser scans as inputs and output a set of lines seen by the robot identifying the obstacles in view.

The RANSAC algorithm is as described in class. From the set of points the laser range finder gives, pick two at random and draw a line. Find out the distance of each of the other points from this line, and bin them as inliers and outliers based on if the distance is lesser or greater than a threshold distance. Repeat this for kiterations. After k iterations, pick the line that has the most number of inliers. Drop those points, andcrepeat the algorithm to the remaining set of points until you have lower than a threshold number of points. We’ll need to experiment with these parameters to find values for the number of iterations, the threshold distance for inliers, and the threshold for the number of points below which you cannot detect lines.

Read through the rviz tutorials. You have to read through the user guide, and built-in data types. Then read through the first two tutorials that explain how you can use markers. We are programming in python, there is a dated python rviz tutorial that might give you examples to start with.
As a demonstration of our implementation of the RANSAC algorithm, publish the detected lines as lines that can be visualized in rviz . rviz should visualize the detected lines in the robot’s local frame. A simple way to verify that your published lines are correct is to enable both the published lines as well as the laser scan in rviz . If they overlap, then you are detecting the lines correctly.

The result of the perception is shown below:

