from playground.robot import Robot
import time
import threading
import math
import numpy as np

# Drone API
class DroneController:
    def __init__(self, __robot,__sensor_view):
        self.__robot = __robot
        self.__sensor_view = __sensor_view
        
        self.__resolution = 2.5 # 1 pixel = 2.5 cm

        self.__max_rotate = 100
        self.__min_rotate = -100

        self.__max_speed = 3 * 100 / self.__resolution # 3 meters/seconds
        self.__speed_x = 0
        self.__speed_y = 0
        self.__acceleration_x = 1 * 100 / self.__resolution
        self.__acceleration_y = 1 * 100 / self.__resolution
        self.__angle_inc = 10

        self.__pitch = 0 
        self.__roll = 0
        self.__yaw = 0

        self.t_move = threading.Thread(target=self.move, args=())
        self.__running = False

    def move(self):
        delta_t = 0.1
        while self.__running:
            w_speed_x =   ((math.sin(math.radians(self.__pitch))) * (math.sin(math.radians(self.__max_rotate)))) * self.__max_speed
            # print('pitch amount',((math.sin(math.radians(self.__pitch)) * (math.sin(math.radians(self.__max_rotate))))))
            # print('desired speed',w_speed)

            if self.__speed_x < round(w_speed_x):
                self.__speed_x = round(self.__speed_x + self.__acceleration_x * delta_t)
            elif self.__speed_x > round(w_speed_x):
                self.__speed_x = round(self.__speed_x - self.__acceleration_x * delta_t)
                
            if self.__speed_x > self.__max_speed:
                self.__speed_x = self.__max_speed
            if self.__speed_x < 0:
                self.__speed_x = 0

            w_speed_y = ((math.sin(math.radians(self.__roll))) * (math.sin(math.radians(self.__max_rotate)))) * self.__max_speed

            if self.__speed_y < round(w_speed_y):
                self.__speed_y = round(self.__speed_y + self.__acceleration_y * delta_t)
            elif self.__speed_y > round(w_speed_y):
                self.__speed_y = round(self.__speed_y - self.__acceleration_y * delta_t)

            if self.__speed_y < 0:
                self.__speed_y = 0
            if self.__speed_y > self.__max_speed:
                self.__speed_y = self.__max_speed
            
            speed = math.sqrt(self.__speed_x**2 + self.__speed_y**2)
            
            # print("speed: ", speed)

            x = speed * delta_t
            self.__robot.move(x)
            time.sleep(0.1)
            
            # speed decay
            # if self.__speed_x > 0:
            #     self.__speed_x -= self.__acceleration_x
            #     self.__pitch -= self.__angle_inc
            # if self.__speed_x < 0:
            #     self.__speed_x += self.__acceleration_x
            #     self.__pitch += self.__angle_inc


    def stop(self):
        self.__running = False
        self.t_move.join()

    def reset_movement(self):
        self.__pitch = 0
        self.__roll = 0
        self.__yaw = 0

    def yaw(self, dieraction):
        assert dieraction == 1 or dieraction == -1, "dieraction should be 1 or -1, dieraction= " + str(dieraction)
        self.__yaw += dieraction * self.__angle_inc

        if self.__yaw < self.__min_rotate:
            self.__yaw = self.__min_rotate
        if self.__yaw > self.__max_rotate:
            self.__yaw = self.__max_rotate
        
        self.__robot.rotate(dieraction * self.__angle_inc)
        
    def pitch(self,sign):
        assert sign == 1 or sign == -1, "sign should be 1 or -1, sign= " + str(sign)
        
        self.__pitch += sign * self.__angle_inc
        if self.__pitch < self.__min_rotate:
            self.__pitch = self.__min_rotate
        if self.__pitch > self.__max_rotate:
            self.__pitch = self.__max_rotate
        

    def roll(self, sign):
        assert sign == 1 or sign == -1, "sign should be 1 or -1, sign= " + str(sign)
        
        self.__roll += sign * self.__angle_inc
        if self.__roll < self.__min_rotate:
            self.__roll = self.__min_rotate
        if self.__roll > self.__max_rotate:
            self.__roll = self.__max_rotate

    def takeoff(self):
        time.sleep(1)
        self.__running = True
        self.__robot.set_altitude(1)
        self.t_move.start()
      

    def land(self):
        time.sleep(1)
        self.__robot.set_altitude(0)
        self.stop()

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

            "battary": self.__sensor_view.battery,

            "pitch": self.__pitch,
            "roll": self.__roll,
            "yaw": self.__yaw,

            "acc_x": self.__acceleration_x,
            "acc_y": self.__acceleration_y,
            "acc_z": None,
        }
        return data