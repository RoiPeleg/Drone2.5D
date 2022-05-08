# slam-playground
Educational 2D SLAM implementation based on ICP and Pose Graph

![slamgui](https://github.com/Kolkir/slam-playground/blob/main/assets/slam.gif)

Our simulator is based on an [open source simulator](https://github.com/Kolkir/slam-playground.git) with some improvements:
1. Add some sensors:
  - 4 lidars (front, left, back , right) in order to calculate distance between the drone and the walls arround it.
  - 2 lidars in order to calcolate it hight and the distance betwwen the drone and the roof.
  - optical flow.
  - gyro.
  - battery.
  - pitch, roll, yaw.
2. Add vectorial movement (in both x and y axis).


Our main movement algorithm can be found [here](https://github.com/RoiPeleg/Drone2.5D/blob/main/playground/Algorithms.py).

Our algorithm is based on the algorithm in the paper "Vision-Less Sensing for Autonomous Micro-Drones".

We did the following improvements:
1. Modify the PID controller to four different little pids:
  - Two for the pitch angle (forward/backward) for a flyforward state and for a tunnel state.
  - Two for the roll angle (left/right) for a flyforward state and for a tunnel state.
3. Identify large derivative in the sides in order to identify intersections and "point of interset" (POI).
4. Make little yaw adjustments in order to fix noise.

### How to use:

Run at the terminal the command: python simulation.py

To load specific map add to the command the road to the map file.
