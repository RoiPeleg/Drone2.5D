import math

from cv2 import add

import pygame
import numpy as np
from skimage.draw import line_aa

from playground.utils.transform import to_screen_coords, make_direction, transform_points, create_rotation_matrix_yx
np.random.seed(42)

class Lidar:
    def __init__(self, dist_range, fov, mu, sigma):
        self.__dist_range = dist_range
        self.__fov = fov
        self.__mu = mu
        self.__sigma = sigma
        self.__obstacles = None
        # generate scan arc coordinates
        self.__thetas = np.array([0, np.pi/2, np.pi, 1.5 * np.pi])

    def get_obstacles(self):
        return self.__obstacles

    def scan(self, position, rotation, world):
        # do ray tracing
        obstacles_coords = []
        obstacles_ids = []
        start_pos = position
        direction = make_direction(rotation)
        angle = np.arctan2(direction[1],direction[0])
        angle = np.degrees(angle) * (-1)
        for theta in self.__thetas:
            scan_angle = (np.degrees(theta) + angle) % 360
            scan_angle = create_rotation_matrix_yx(scan_angle)
            direction = make_direction(scan_angle)

            end_pos = (start_pos + direction * self.__dist_range).astype(int)
            start_pos = start_pos.astype(int)
            ys, xs, _ = line_aa(start_pos[0], start_pos[1], end_pos[0], end_pos[1])
            added_obs = False
            for pos in zip(ys, xs):
                is_obstacle, obstacle_id = world.is_obstacle(pos)
                if is_obstacle:
                    obstacles_coords.append(pos)
                    obstacles_ids.append(obstacle_id)
                    added_obs = True
                    break
            
            if not added_obs:
                obstacles_coords.append((np.inf , np.inf))
                obstacles_ids.append(-1)

        obstacles_coords = np.array(obstacles_coords)
        obstacles_ids = np.array(obstacles_ids).reshape((-1, 1))
        
        # Transform obstacles into the sensor/robot coordinate system
        obstacles_coords -= start_pos
        obstacles_coords = transform_points(obstacles_coords, np.linalg.inv(rotation))

        for obs in obstacles_coords:
            if obs[0] == 0 and obs[1] == 0:
                print("obstacle: ", obstacles_coords)
                break

        # Adding noise
        noise = np.random.normal(self.__mu, self.__sigma, size=obstacles_coords.shape)
        obstacles_coords += noise.astype(int)

        self.__obstacles = np.hstack([obstacles_coords, obstacles_ids])

    def draw(self, screen, h, w, position, direction,start):
        color = (128, 128, 128)
        start_pos = pygame.math.Vector2(position[0], position[1])
        dir = pygame.math.Vector2(direction[0], direction[1])

        for i in range(0,4):
            dir = dir.rotate(i*self.__fov)
            end_pos = start_pos + dir * self.__dist_range
            start_pos = to_screen_coords(h, w, start_pos,start)
            end_pos = to_screen_coords(h, w, end_pos, start, clip=False)
            pygame.draw.line(screen, color=color, start_pos=start_pos, end_pos=end_pos, width=2)
            start_pos = pygame.math.Vector2(position[0], position[1])
