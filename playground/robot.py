from cv2 import FlannBasedMatcher
import pygame
from playground.environment.body import Body
from playground.utils.transform import to_screen_coords, make_direction

class Robot(Body):
    def __init__(self, odometry, sensor, world):
        super().__init__()
        self.__radius = 4 # every pixel is 2.5 cm -> 4 pixles * 2.5 = 10 cm raduis of the drone
        self.__odomentry = odometry
        self.__speed_x = 0
        self.__speed_y = 0
        self.gyro = 0 
        self.__sensor = sensor
        self.__world = world

    def rotate(self, angle):
        super().rotate(angle)
        self.__odomentry.track_rotate(angle)
        self.__sensor.scan(self.position, self.rotation, self.__world)

    def move(self, dist):
        if self.__world.allow_move(self.try_move(dist), self.size):
            super().move(dist)
            self.__odomentry.track_move(dist)
            self.__sensor.scan(self.position, self.rotation, self.__world)

    def set_altitude(self, new_alt):
        super().set_altitude(new_alt)
        self.__odomentry.track_altitude(new_alt)

    def c_speed_x(self, velo):   
        self.__speed_x = velo
    
    def c_speed_y(self, velo):   
        self.__speed_y = velo
    
    @property
    def speed_x(self):
        return self.__speed_x

    @property
    def speed_y(self):
        return self.__speed_y
        
    @property
    def odomentry(self):
        return self.__odomentry
        
    @property
    def size(self):
        return self.__radius
    
    @property
    def sensor(self):
        return self.__sensor

    def draw(self, screen, h, w):
        # Draw robot in the real environment
        position = to_screen_coords(h, w, self.position)
        pygame.draw.circle(screen, color=(0, 0, 255), center=position, radius=self.__radius)
        direction = make_direction(self.rotation)
        dir_pos = self.position + direction * self.__radius * 2
        dir_pos = to_screen_coords(h, w, dir_pos)
        pygame.draw.line(screen, color=(0, 255, 0), start_pos=position, end_pos=dir_pos, width=10)
        self.__sensor.draw(screen, h, w, self.position, direction)
