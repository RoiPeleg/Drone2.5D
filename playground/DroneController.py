from playground.robot import Robot
import time
import threading

import numpy as np

# Drone API
class DroneController:
    def __init__(self, __robot,__sensor_view):
        self.__robot = __robot
        self.__sensor_view = __sensor_view
        
        self.__angle_inc = 10
        self.__resolution = 2.5 # 1 pixel = 2.5 cm
        self.__max_velocity = 120 # 3 meter/second
        self.__pitch = 0 
        self.__roll = 0
        self.__yaw = 0
        # self.__acceleration = 1 # m/s^2
        # self.__angular_velocity = 100 # angle

        # self.__time_to_change = self.__angular_velocity / self.__acceleration

        self.t_move = threading.Thread(target=self.move, args=())
        self.__running = True


    def move(self):
        while self.__running:
            velocity = self.__robot.speed

            dist = velocity / self.__resolution
            # velocity = self.__pitch/100
            self.__robot.move(dist)   
    
    def stop(self):
        self.__running = False
        self.t_move.join()

    def yaw(self,dieraction):
        assert dieraction == 1 or dieraction == -1, "dieraction should be 1 or -1, dieraction= " + str(dieraction)
        self.__robot.rotate(dieraction * self.__angle_inc)

    def pitch(self,sign):
        assert sign == 1 or sign == -1, "sign should be 1 or -1, sign= " + str(sign)
        
        self.__pitch += sign * self.__angle_inc
        velocity_x =  (np.sin(self.__pitch) * self.__angle_inc * self.__robot.size) // (self.__resolution*100)
        
        if velocity_x < 0:
            velocity_x = 0
        if velocity_x > self.__max_velocity:
            velocity_x = self.___max_velocity
            
        self.__robot.c_speed_x(velocity_x)
        

    def roll(self, sign):
        assert sign == 1 or sign == -1, "sign should be 1 or -1, sign= " + str(sign)
        
        self.__pitch += sign * self.__angle_inc
        velocity_y =  (np.cos(self.__pitch) * self.__angle_inc * self.__robot.size) // (self.__resolution*100)
        
        if velocity_y < 0:
            velocity_y = 0
        if velocity_y > self.__max_velocity:
            velocity_y = self.___max_velocity
            
        self.__robot.c_speed_y(velocity_y)

    def takeoff(self):
        time.sleep(1)
        self.__robot.set_altitude(1)
        self.t_move.start()
      

    def land(self):
        time.sleep(1)
        self.__robot.set_altitude(0)

    def battery_level(self):
        return self.__sensor_view.battery
    
    def sensors_data(self):
        # [d0-d4, yaw, Vx, Vy, Z, baro, bat, pitch, roll, accX, accY, accZ]

        ds = self.__sensor_view.distance_from_obstacles * self.__resolution

        data = {
            "d_left": ds[1],
            "d_right": ds[3],
            "d_front": ds[0],
            "d_back": ds[2],
            "d_down": self.__sensor_view.drone_height,
            "d_up": self.__sensor_view.dis_from_roof,

            "v_x": self.__sensor_view.opticalflow[0],
            "v_y": self.__sensor_view.opticalflow[1],

            "battary": self.__sensor_view.battery,

            "pitch": self.__pitch,
            "roll": self.__roll,
            "yaw": self.__yaw,

            "acc_x": None,
            "acc_y": None,
            "acc_z": None,
        }

        print(data)
        print(np.inf)

        return data
