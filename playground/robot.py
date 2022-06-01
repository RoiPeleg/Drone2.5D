from cv2 import FlannBasedMatcher
import pygame
from playground.environment.body import Body
from playground.utils.transform import to_screen_coords, make_direction

class Robot(Body):
    def __init__(self, odometry, sensor, world):
        super().__init__()
        self.__radius = 4 # every pixel is 2.5 cm -> 4 pixles * 2.5 = 10 cm raduis of the drone
        self.__odomentry = odometry
        
        self.__sensor = sensor
        self.__world = world

    def rotate(self, angle):
        super().rotate(angle)
        if self.__odomentry != None:
            self.__odomentry.track_rotate(angle)

    def move(self, dist_x, dist_y):
        if self.__world.allow_move(self.try_move(dist_x, dist_y), self.size):
            super().move(dist_x, dist_y)
            if self.__odomentry != None:
                self.__odomentry.track_move(dist_x, dist_y)

    def set_altitude(self, new_alt):
        super().set_altitude(new_alt)
        if self.__odomentry != None:
            self.__odomentry.track_altitude(new_alt)
        
    @property
    def odomentry(self):
        return self.__odomentry
        
    @property
    def size(self):
        return self.__radius
    
    @property
    def sensor(self):
        return self.__sensor
    
    def set_world(self, new_world):
        self.__world = new_world

    def draw(self, screen, h, w, start):
        # draw the init drone position
        init_position = to_screen_coords(h, w,[0,0],start)
        pygame.draw.circle(screen, color=(255, 0, 0), center=init_position, radius=10)

        # Draw robot in the real environment
        position = to_screen_coords(h, w, self.position,start)
        pygame.draw.circle(screen, color=(0, 0, 255), center=position, radius=self.__radius)
        direction = make_direction(self.rotation)
        dir_pos = self.position + direction * self.__radius * 2
        dir_pos = to_screen_coords(h, w, dir_pos,start)
        pygame.draw.line(screen, color=(0, 255, 0), start_pos=position, end_pos=dir_pos, width=5)
        self.__sensor.draw(screen, h, w, self.position, direction,start)
