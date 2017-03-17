# CS76 Motion Planning Assignment

Author:   Carter J. Bastian
          February, 2016. 16W.

## Honor Code Violation Disclaimer:                                                
This is work done as an assignment for Dartmouth College's CS56.                   
If you are currently enrolled at Dartmouth college and will ever take this course, **downloading, using, modifying, running, or even looking at this code is an honor code violation.** Please don't; it is not for you.
                                                                                   
If you are a professor teaching this course and wish for me to take down the REPO, shoot me an email at carter.bastian1@gmail.com, and I'll be happy to do so. 

## Description
To run the simulation, all you need to do is run
  make
from the command line.


To switch to the Hanover city map demo (takes about three minutes):
- Change the defaultSize value in MotionPlanner.java to 20,000
- Change the IsPlanarRobot value in MotionPlannerDriver.java to true
- Change the RESOLUTION value in MotionPlanner.java to 0.01
And run
  make
from the command line.


To switch to the Extra Credit demo:
- Change the defaultSize value in MotionPlanner.java to 100
- Set the IsPlanarRobot value in MotionPlannerDriver.java to false
And run
  make extra
from the command line.
