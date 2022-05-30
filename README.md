# slam-playground
Educational 2D SLAM implementation based on ICP and Pose Graph

![slamgui](https://github.com/Kolkir/slam-playground/blob/main/assets/slam.gif)

Our simulator is based on an [open source simulator](https://github.com/Kolkir/slam-playground.git) with some improvements:
1. Add some sensors (updating in 10 hz):
  - 4 lidars (front, left, back , right) in order to calculate distance between the drone and the walls arround it.
  - 2 lidars in order to calcolate it hight and the distance betwwen the drone and the roof.
  - Optical Flow.
  - Gyro.
  - Battery.
  - Pitch, Roll, Yaw.
2. Add vectorial movement (in both x and y axis).
3. Improve GUI: merge between the sensor view map and the real word map.
4. Create simple API in order to comunicate with the simulation and get the sensors data.


Our main movement algorithm can be found [here](https://github.com/RoiPeleg/Drone2.5D/blob/main/playground/Algorithms.py).

Our algorithm is based on the algorithm in the paper "Vision-Less Sensing for Autonomous Micro-Drones".

We did the following improvements:
1. Modify the PID controller to four different little pids:
  - Two for the pitch and roll angle for a flyforward state in order to increase the drone speed.
  - Two for the pitch and roll angle for a tunnel state in order to get more smooth movements.
2. Identify large derivative in the sides in order to identify intersections and "Point Of Interset" (POI).
3. Make little yaw adjustments in order to fix noise.
4. Implement Particle filter for localization.
### How to use:

Run at the terminal the command: python simulation.py

To load specific map add to the command the road to the map file.
