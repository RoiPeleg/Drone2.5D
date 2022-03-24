from cv2 import FlannBasedMatcher
import pygame
from playground.environment.body import Body
from playground.utils.transform import to_screen_coords, make_direction
import threading
import time
import random

class Robot(Body):
    def __init__(self, odometry, sensors):
        super().__init__()
        self.__radius = 10
        self.__odomentry = odometry
        self.__speed = 10
        self.__auto = False
        self.t_rd = None
        self.gyro = 0 
        self.__sensors = sensors

    def rotate(self, angle, world):
        super().rotate(angle)
        self.__odomentry.track_rotate(angle)
        for sensor in self.__sensors:
           sensor.scan(self.position, self.rotation, world)

    def move(self, dist, world):
        if world.allow_move(self.try_move(dist), self.size):
            super().move(dist)
            self.__odomentry.track_move(dist)
            self.__opticalflow = 1
            for sensor in self.__sensors:
                sensor.scan(self.position, self.rotation, world)

    def random_walk(self,dist,world):
        while self.__auto:
            self.move(dist,world)
            d = random.randint(-45, 45)
            self.rotate(d,world)
            time.sleep(0.8)
    
    def init_rd(self,dist,world):
        self.__auto = True
        self.t_rd = threading.Thread(target=self.random_walk, args=(dist,world))
        self.t_rd.start()

    def stop_rd(self):
        self.__auto = False
        self.t_rd.join()

    def c_speed(self,inc):   
        self.__speed += inc
    
    @property
    def speed(self):
        return self.__speed

    @property
    def odomentry(self):
        return self.__odomentry
    @property
    def size(self):
        return self.__radius

    def draw(self, screen, h, w):
        # Draw robot in the real environment
        position = to_screen_coords(h, w, self.position)
        pygame.draw.circle(screen, color=(0, 0, 255), center=position, radius=self.__radius)
        direction = make_direction(self.rotation)
        dir_pos = self.position + direction * self.__radius * 2
        dir_pos = to_screen_coords(h, w, dir_pos)
        pygame.draw.line(screen, color=(0, 255, 0), start_pos=position, end_pos=dir_pos, width=2)
        for sensor in self.__sensors: 
            sensor.draw(screen, h, w, self.position, direction)
