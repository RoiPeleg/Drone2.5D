import numpy as np
import pygame

from skimage.draw import line_aa
from playground.utils.transform import to_screen_coords, transform_points
np.random.seed(42)

class RawSensorsView:
    def __init__(self, world_h, world_w, world_z):
        self.__h = world_h
        self.__w = world_w
        self.__z = world_z
        self.__map = np.full((world_h, world_w), 255, dtype=np.uint8)
        self.__local_map = np.full((world_h, world_w), 255, dtype=np.uint8)
        self.__y, self.__x = np.empty(0), np.empty(0)

        self.__prev_pos = None
        self.__prev_pos_real = None
        self.__opticalflow = [0, 0]
        self.__gyro = 0
        self.__battery = 100
        self.__prev_angle = 0

        self.__drone_height = 0
        self.__dis_from_roof = self.__z

        self.__distance_from_obstacles = []

    def take_measurements_battery(self,battery_prectange):
        self.__battery = battery_prectange
    
    # measures barometer
    def take_measurements_barometer(self, odometry):
        if odometry.altitude != 0:
            odometry.track_altitude(1)
        self.__drone_height = odometry.altitude
        self.__dis_from_roof = self.__z - self.__drone_height

    # measures optical flow 
    def take_measurements_optical(self,odometry):
        if self.__prev_pos_real is not None:
            p =  odometry.position
            self.__opticalflow = abs(p - self.prev_pos_real)
        self.__prev_pos_real = odometry.position
        
    def take_measurements_gyro(self, odometry):
        current_ang = odometry.angle
        self.__gyro = current_ang - self.__prev_angle
        self.__prev_angle = current_ang
        
    # mesures lidar distances
    def take_measurements(self, odometry, sensor, start, position=None, rotation=None):
        # Process odometery
        if odometry != None:
            position = to_screen_coords(self.__h, self.__w, odometry.position, start)
        else:
            position = to_screen_coords(self.__h, self.__w, position, start)
        
        if self.__prev_pos is not None:
            rr, cc, _ = line_aa(self.__prev_pos[1], self.__prev_pos[0], position[1], position[0])
            # noise in the odometry can break coordinates
            rr = np.clip(rr, 0, self.__h - 1)
            cc = np.clip(cc, 0, self.__w - 1)
            self.__map[rr, cc] = 0
        else:
            self.__map[position[1], position[0]] = 0
        self.__prev_pos = position

        # Process sensor
        obstacles = sensor.get_obstacles()
        if obstacles is not None:
            obstacles = obstacles.copy()
            self.__distance_from_obstacles = np.empty(len(obstacles))
            ls = []

            # compute the distance between the robot and each obstacles
            for obs_index in range(len(obstacles)):
                if obstacles[obs_index][-1] == -1: #if obstacle not found
                    self.__distance_from_obstacles[obs_index] = np.inf
                    ls.append(obs_index)
                else:
                    obs = np.array(obstacles[obs_index, :2])
                    d = np.sqrt((obs[0])**2 + (obs[1])**2 )
                    self.__distance_from_obstacles[obs_index] = d
            
            # remove inf form real obsticals
            if ls != []:
                obstacles = np.delete(obstacles, ls, axis=0)

            if 0 in self.__distance_from_obstacles:
                print("distance: ", self.__distance_from_obstacles)
                print("obstacles: ", obstacles)

            # convert points into world coordinate system
            if odometry != None:
                obstacles = transform_points(obstacles[:, :2], odometry.rotation)
                obstacles += odometry.position[:2].astype(int)
            else:
                obstacles = transform_points(obstacles[:, :2], rotation)
                obstacles += np.array(position)[:2].astype(int)

            # self.__local_map[obstacles[:, 0], obstacles[:, 1]] = 0
            self.__y = np.concatenate((self.__y, obstacles[:, 0]), axis=None)
            self.__x = np.concatenate((self.__x, obstacles[:, 1]), axis=None)
            
            obstacles[:, 0] = start[0] - obstacles[:, 0]
            obstacles[:, 1] = start[1] + obstacles[:, 1]
            #noise in the odometry can break coordinates
            obstacles = np.clip(obstacles, [0, 0],
                                [self.__h - 1, self.__w - 1])
            self.__map[obstacles[:, 0], obstacles[:, 1]] = 0
            
            
            
    @property
    def distance_from_obstacles(self):
        return self.__distance_from_obstacles

    @property
    def drone_height(self):
        return self.__drone_height

    @property
    def battery(self):
        return self.__battery
        
    @property
    def opticalflow(self):
        return self.__opticalflow
    
    @property
    def gyro(self):
        return self.__gyro

    @property
    def prev_pos_real(self):
        return self.__prev_pos_real
    
    @property
    def dis_from_roof(self):
        return self.__dis_from_roof

    @property
    def map(self):
        return self.__map
    
    @property
    def local_map(self):
        return self.__local_map

    @property
    def x(self):
        return self.__x

    @property
    def y(self):
        return self.__y

    def draw(self, screen, offset):
        transposed_map = np.transpose(self.__map)
        surf = pygame.surfarray.make_surface(transposed_map)
        screen.blit(surf, (offset, 0))
