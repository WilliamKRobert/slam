# Robot trajectory optimization using LMA C++ library.
This C++ project aims to optimize robot trajectories with multiple constraints from sensors, such as GPS, IMU, altimeter, and so on. Trajectory optimization has crutial applications in robotics.

## Papers Describing the Approach:
Lovegrove, Steven, Alonso Patron-Perez, and Gabe Sibley. "Spline Fusion: A continuous-time representation for visual-inertial fusion with application to rolling shutter cameras." BMVC. 2013.

Ramadasan, Datta. SLAM temporel à contraintes multiples. Diss. Université Blaise Pascal-Clermont-Ferrand II, 2015.

## Requirements
This project requires LMA (Levenberg-Marquardt Algorithm), Eigen, Sophus and Geodesy. Also, it is build in ROS. Please refer to the following websites:  
LMA: https://github.com/bezout/LMA  
Eigen: http://eigen.tuxfamily.org/index.php?title=Main_Page  
Sophus: https://github.com/strasdat/Sophus  
Geodesy: http://wiki.ros.org/geodesy  
ROS: http://wiki.ros.org/  

## Structure
The project folder "./slam" is a package under ROS workspace. Under the folder "/slam", there are two directories: '/slam/src', '/slam/include', and two configuration files: 'CMakeLists.txt', 'package.xml'.  

For that moment, I have uploaded a "main.cpp" under "/slam/src" which is used only for test and demo (optimizatioin spline trajectories within ten seconds window). I will upload the module for complete optimization process lately this week.






