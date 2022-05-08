from playground.robot import Robot
import time
import threading
import math
import numpy as np

# Drone API
class DroneController:
    def __init__(self, __robot,__sensor_view, delta_t = 0.1):
        self.__robot = __robot
        self.__sensor_view = __sensor_view
        
        self.__delta_t = delta_t
        self.__resolution = 2.5 # 1 pixel = 2.5 cm

        self.__max_rotate = 10
        self.__min_rotate = -10

        self.__max_speed = 3 * 100 / self.__resolution # 3 meters/seconds
        self.__min_speed = - 3 * 100 / self.__resolution # 3 meters/seconds

        self.__speed_x = 0
        self.__speed_y = 0
        self.__acceleration_x = 1 * 100 / self.__resolution
        self.__acceleration_y = 1 * 100 / self.__resolution
        self.__angle_inc = delta_t * 1000

        self.__pitch = 0 
        self.__roll = 0
        self.__yaw = 0
        self.__counter = 0

        # self.t_move = threading.Thread(target=self.move, args=())
        self.__running = False
    
    @property
    def position(self):
        return self.__robot.position

    def move(self):
        # while self.__running:

            sign = 0
            if self.__counter > 0:
                if self.__yaw < 0:
                    sign = -1
                elif self.__yaw > 0:
                    sign = 1

                self.__robot.rotate(sign * self.__angle_inc * self.__delta_t)
                self.__counter -= 1
                self.__yaw -= sign * self.__angle_inc * self.__delta_t

            w_speed_x = (self.__pitch - self.__min_rotate) / (self.__max_rotate - self.__min_rotate) * (self.__max_speed - self.__min_speed) + self.__min_speed

            if self.__speed_x < round(w_speed_x):
                self.__speed_x = round(self.__speed_x + self.__acceleration_x * self.__delta_t)
            elif self.__speed_x > round(w_speed_x):
                self.__speed_x = round(self.__speed_x - self.__acceleration_x * self.__delta_t)
                
            if self.__speed_x > self.__max_speed:
                self.__speed_x = self.__max_speed
            elif self.__speed_x < self.__min_speed:
                self.__speed_x = self.__min_speed

            w_speed_y = (self.__roll - self.__min_rotate) / (self.__max_rotate - self.__min_rotate) * (self.__max_speed - self.__min_speed) + self.__min_speed

            if self.__speed_y < round(w_speed_y):
                self.__speed_y = round(self.__speed_y + self.__acceleration_y * self.__delta_t)
            elif self.__speed_y > round(w_speed_y):
                self.__speed_y = round(self.__speed_y - self.__acceleration_y * self.__delta_t)

            if self.__speed_y > self.__max_speed:
                self.__speed_y = self.__max_speed
            elif self.__speed_y < self.__min_speed:
                self.__speed_y = self.__min_speed
            
            x = self.__speed_x * self.__delta_t
            y = self.__speed_y * self.__delta_t
            self.__robot.move(x, y)

            # time.sleep(0.1)

    def stop(self):
        self.__running = False
        # self.t_move.join()


    def yaw(self, angle):
        self.__yaw = angle
        if angle > 0:
            self.__counter = angle / (self.__angle_inc / 10)
        elif angle < 0:
            self.__counter = -1 * angle / (self.__angle_inc / 10)
        
    def pitch(self, angle):
        
        self.__pitch = angle
        if self.__pitch < self.__min_rotate:
            self.__pitch = self.__min_rotate
        if self.__pitch > self.__max_rotate:
            self.__pitch = self.__max_rotate

    def roll(self, angle):

        self.__roll = angle
        if self.__roll < self.__min_rotate:
            self.__roll = self.__min_rotate
        if self.__roll > self.__max_rotate:
            self.__roll = self.__max_rotate

    def takeoff(self):
        time.sleep(1)
        self.__running = True
        self.__robot.set_altitude(1)
        # self.t_move.start()

    def land(self):
        time.sleep(1)
        self.__robot.set_altitude(0)
        # self.stop()

    def battery_level(self):
        return self.__sensor_view.battery
    
    def sensors_data(self):
        # [d0-d4, yaw, Vx, Vy, Z, baro, bat, pitch, roll, accX, accY, accZ]

        ds = self.__sensor_view.distance_from_obstacles * self.__resolution / 100

        data = {
            "d_left": round(ds[1], 2),
            "d_right": round(ds[3], 2),
            "d_front": round(ds[0], 2),
            "d_back": round(ds[2], 2),
            "d_down": round(self.__sensor_view.drone_height, 2),
            "d_up": round(self.__sensor_view.dis_from_roof, 2),

            "v_x": round(self.__sensor_view.opticalflow[0], 2),
            "v_y": round(self.__sensor_view.opticalflow[1], 2),

            "gyro": round(self.__sensor_view.gyro, 2),
            "battery": self.__sensor_view.battery,

            "pitch": round(self.__pitch, 2),
            "roll": round(self.__roll, 2),
            "yaw": self.__yaw,

            "acc_x": self.__acceleration_x,
            "acc_y": self.__acceleration_y,
            "acc_z": None,

        }
        return data