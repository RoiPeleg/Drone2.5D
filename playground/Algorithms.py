import random
import time

import numpy as np
import math
import pygame
from sqlalchemy import true
from playground.utils.transform import to_screen_coords, make_direction
import collections
import matplotlib.pyplot as plt
from scipy import ndimage, interpolate
np.random.seed(42)
np.seterr("ignore")
from copy import deepcopy

from playground.pure_pursuit import *


# def smooth(path, weight_data=0.5, weight_smooth= 0.1, tolerance=0.000001):
#     """
#     source: https://medium.com/@jaems33/understanding-robot-motion-path-smoothing-5970c8363bc4
#     Creates a smooth path for a n-dimensional series of coordinates.
#     Arguments:
#         path: List containing coordinates of a path
#         weight_data: Float, how much weight to update the data (alpha)
#         weight_smooth: Float, how much weight to smooth the coordinates (beta).
#         tolerance: Float, how much change per iteration is necessary to keep iterating.
#     Output:
#         new: List containing smoothed coordinates.
#     """

#     new = deepcopy(path)
#     dims = len(path[0])
#     change = tolerance

#     while change >= tolerance:
#         change = 0.0
#         for i in range(1, len(new) - 1):
#             for j in range(dims):

#                 x_i = path[i][j]
#                 y_i, y_prev, y_next = new[i][j], new[i - 1][j], new[i + 1][j]

#                 y_i_saved = y_i
#                 y_i += weight_data * (x_i - y_i) + weight_smooth * (y_next + y_prev - (2 * y_i))
#                 new[i][j] = y_i

#                 change += abs(y_i - y_i_saved)

#     return new

def smooth(x,y):
    tck, *rest = interpolate.splprep([x,y], s=len(x)+np.sqrt(2*len(x)))
    u = np.linspace(0,1, num=len(x))
    xint, yint = interpolate.splev(u, tck)
    return xint, yint

def bfs(grid, start, goal, width, height):
    queue = collections.deque([[(int(start[1]),int(start[0]))]])
    seen = set([(int(start[1]),int(start[0]))])
    while queue:
        path = queue.popleft()
        x, y = path[-1]
        if int(goal[1]) == x and int(goal[0]) == y:
            return path
        for x2, y2 in ((x+1,y), (x-1,y), (x,y+1), (x,y-1),(x-1,y-1), (x+1,y+1), (x+1,y-1), (x-1,y+1)):
            if 0 <= x2 < width and 0 <= y2 < height and grid[y2][x2] != 1 and (x2, y2) not in seen:
                queue.append(path + [(x2, y2)])
                seen.add((x2, y2))
    
def rmse(a, b):
    MSE = np.square(np.subtract(a,b)).mean() 
    RMSE = np.sqrt(MSE)
    return RMSE

def norm(a):
    norm = np.linalg.norm(a)
    if norm == np.nan :
        norm = 0.001
    return a/norm

def clip(n, minn, maxn):
    return max(min(maxn, n), minn)

class PID():
    def __init__(self, Kp, Ki, Kd, delta_t=0.1, max_measurements=3, disired_distance=0.5):
        
        self.max_i = 5
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.__first_run = True
        self.__last_error = 0
        self.__integral = 0

        self.delta_t = delta_t
        self.__max_measurements = max_measurements
        self.__disired_distance = disired_distance
    
    def compute(self, measurement):
        error = measurement - self.__disired_distance
        if error == 0:
            self.reset()

        if np.isinf(error):
            # if measurement is inf the pid controller get crazy
            error = self.__max_measurements - self.__disired_distance

        if self.__first_run:
            self.__last_error = error
            self.__first_run = False

        self.__integral += error * self.delta_t
        diff = (error - self.__last_error) / self.delta_t
        const_integral = clip(self.__integral, -5, 5)

        u_t = self.Kp * error + self.Ki * const_integral + self.Kd * diff
        self.__last_error = error

        return u_t
    
    def set_disired_distance(self, d):
        self.__disired_distance = d

    def set_params(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def reset(self):
        self.__last_error = 0
        self.__integral = 0
        self.__first_run = True
    
class Algorithms:
    def __init__(self, controller, odometry, delta_t=0.1):
        
        self.__controller = controller
        self.__odometry = odometry
        self.__local_pos = odometry.position
        self.__min_local_pos_x = np.inf
        self.__min_local_pos_y = np.inf
        self.__max_local_pos_x = np.NINF
        self.__max_local_pos_y = np.NINF
        self.__rotation = odometry.rotation
        self.__state = None
        self.__data = None
        
        # step control indcators  
        self.__first_step = True
        self.__first_go_home = True
        self.__second_go_home = True
        self.__third_go_home = True
        self.__arrive_home = False
        
        # time constant
        self.__delta_t = delta_t
        
        # distances vector
        self.__current = None
        self.__prev = None

        #PIDs exploration
        self.PID_p = PID(1.5,0.004,0.4, disired_distance=0.1)
        self.PID_r = PID(6.0,6.0,1.0)

        # PIDs way back
        self.PID_y = PID(0.5,0.0,0.05, max_measurements=180, disired_distance=0)
        self.PID_ph = PID(0.015,0.00004,0.004, disired_distance=0)
        self.PID_rh = PID(1.5,1.5,0.25, disired_distance=0.2)

        # BAT tresholds
        self.emengercy_tresh = 0.3
        self.tunnel_tresh = 0.75
        self.front_tresh = 1
        self.right_far_tresh = 2.5
        self.left_far_tresh = 2.5

        # local map, bfs
        self.min_x, self.min_y, self.max_x, self.max_y = np.inf, np.inf, np.NINF, np.NINF
        self.local_map = None
        self.traj = np.empty(0)
        self.goal = None

        self.target_point = [0,0]
        self.path_draw = []
        self.home_coords = None
        

    @property
    def state(self):
        return self.__state

    def draw(self, screen, h, w, start):
        for p in self.path_draw :
            position = to_screen_coords(h, w, p, start)
            pygame.draw.circle(screen, color=(165, 42, 42), center=position, radius=2)

        target = (( (self.target_point[1] - 0) / (abs(self.max_y) + abs(self.min_y) - 0) ) * (self.max_y - self.min_y) + self.min_y, 
                                        ( (self.target_point[0] - 0) / (abs(self.max_x) + abs(self.min_x) - 0) ) * (self.max_x - self.min_x) + self.min_x)
        position = to_screen_coords(h, w, target, start)
        pygame.draw.circle(screen, color=(255, 0, 255), center=position, radius=3)

    def sample_data(self):
        self.__data = self.__controller.sensors_data()
        self.__current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])
        self.__local_pos = self.__odometry.position
        self.__min_local_pos_x = min(self.__min_local_pos_x, self.__local_pos[1])
        self.__min_local_pos_y = min(self.__min_local_pos_y, self.__local_pos[0])
        self.__max_local_pos_x = max(self.__max_local_pos_x, self.__local_pos[1])
        self.__max_local_pos_y = max(self.__max_local_pos_y, self.__local_pos[0])
        y,x = make_direction(self.__odometry.rotation)
        self.__rotation = np.degrees(math.atan2(y,x))
        if self.__first_step:
            self.__prev = self.__current.copy()

    def random_walk(self):
        self.__controller.takeoff()
        while self.__auto:
            self.__controller.yaw(random.choice([-1,1]))
            
            self.__controller.pitch(random.choice([-1,1]))

            self.__controller.roll(random.choice([-1,1]))
 
        self.__controller.land()

    def Emengercy(self):
        self.__state = 'Emengercy'
        self.__controller.roll(0)
        self.__controller.pitch(-1)
        
    def Tunnel(self, left, right, wall_align = 'd_right'):
        self.__state = 'Tunnel'
        u_t_p_t = self.PID_p.compute(self.__data['d_front'])
        self.__controller.pitch(u_t_p_t)        

        u_t_r_t = self.PID_r.compute(self.__data[wall_align])
        self.__controller.roll(u_t_r_t)

    def Fly_Forward(self, wall_align = 'd_right'):
        self.__state = 'Forward'
        u_t_p = self.PID_p.compute(self.__data['d_front'])
        self.__controller.pitch(u_t_p)
        sign = 1
        if wall_align == 'd_left':
            sign = -1

        u_t_r = sign * self.PID_r.compute(self.__data[wall_align])        
        self.__controller.roll(u_t_r)
        
    def RotateCCW(self):
        self.__state = 'Rotate CCW'
        self.__controller.yaw(10)
    
    def RotateCW(self):
        self.__state = 'Rotate CW'
        self.__controller.yaw(-10)

    def RotateCW_90(self):
        self.__state = 'Rotate 90CW'
        self.__controller.yaw(-90)

    def RotateCW_180(self):
        self.__state = 'Rotate 180CW'
        self.__controller.yaw(-180)
    
    def RotateCCW_90(self):
        self.__state = 'Rotate 90CCW'
        self.__controller.yaw(90)

    def step(self, x, y):
        if self.__first_step:
            self.__controller.takeoff()
            self.__first_step = False

        epsilon = 0.3
        self.__current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])

        if self.__data["battery"] > 60:
            self.BAT(epsilon)

        elif self.__first_go_home:
            self.PID_p.reset()
            self.PID_r.reset()
            self.__controller.pitch(0)
            self.__controller.roll(0)
            self.__first_go_home = False
            
        elif self.__second_go_home:
            opt = [self.__data["v_x"], self.__data["v_y"]]
            if min(opt) > 0:
                return
            
            # rotate 180 degrees
            # self.RotateCW_180()
            self.__second_go_home = False

        elif self.__third_go_home:
            if self.__data["yaw"] < 0.0 :
                return
            
            self.min_x = int(min(self.__local_pos[1], np.min(x), 0))
            self.min_y = int(min(self.__local_pos[0], np.min(y), 0))
            self.max_x = int(max(self.__local_pos[1], np.max(x), 0))
            self.max_y = int(max(self.__local_pos[0], np.max(y), 0))

            # new_value = ( (old_value - old_min) / (old_max - old_min) ) * (new_max - new_min) + new_min
            self.home_coords = np.array([( (0 - self.min_y) / (self.max_y - self.min_y) ) * (abs(self.max_y) + abs(self.min_y) - 0) + 0 ,
                            ( (0 - self.min_x) / (self.max_x - self.min_x) ) * (abs(self.max_x) + abs(self.min_x) - 0) + 0])
            
            y = ( (y - self.min_y) / (self.max_y - self.min_y) ) * (abs(self.max_y) + abs(self.min_y) - 0) + 0
            x = ( (x - self.min_x) / (self.max_x - self.min_x) ) * (abs(self.max_x) + abs(self.min_x) - 0) + 0
            
            y = y.astype(int)
            x = x.astype(int)
        
            self.local_map = np.zeros(shape=(abs(self.max_y) + abs(self.min_y) + 1, abs(self.max_x) + abs(self.min_x) + 1))
            self.local_map[y,x] = 1

            pos = np.array([( (self.__local_pos[0] - self.min_y) / (self.max_y - self.min_y) ) * (abs(self.max_y) + abs(self.min_y) - 0) + 0 ,
                            ( (self.__local_pos[1] - self.min_x) / (self.max_x - self.min_x) ) * (abs(self.max_x) + abs(self.min_x) - 0) + 0])
                            
            self.local_map = ndimage.maximum_filter(self.local_map, size=20)
            path = bfs(self.local_map, pos, self.home_coords, self.local_map.shape[1], self.local_map.shape[0])
                                   
            traj_x = []
            traj_y = []
            for pos in path:
                traj_x.append(pos[0])
                traj_y.append(pos[1])
            
            traj_x,traj_y = smooth(traj_x, traj_y)
            traj_x = np.array(traj_x).astype(np.int)
            traj_y = np.array(traj_y).astype(np.int)

            self.traj = Trajectory(traj_x, traj_y)
            self.goal = self.traj.getPoint(len(traj_x) - 1)

            for x,y in zip(traj_x,traj_y):
                self.path_draw.append((( (y - 0) / (abs(self.max_y) + abs(self.min_y) - 0) ) * (self.max_y - self.min_y) + self.min_y, 
                                        ( (x - 0) / (abs(self.max_x) + abs(self.min_x) - 0) ) * (self.max_x - self.min_x) + self.min_x))

            self.__third_go_home = False
            
        # elif self.__data["battery"] > 0 and not self.__arrive_home:
        elif not self.__arrive_home:
            self.GoHome(epsilon)

        # elif self.__data["battery"] <= 0:
        #     print("Timeout!")
        #     self.__controller.pitch(0)
        #     self.__controller.roll(0)
        #     self.__controller.land()

        # else:
        #     print("Drone returned home")
        #     self.__controller.pitch(0)
        #     self.__controller.roll(0)
        #     self.__controller.land()
        
        self.__prev = self.__current.copy()


    def BAT(self, epsilon):
        if self.__current[0] < self.emengercy_tresh:
            self.Emengercy()
        elif self.__current[0] < self.front_tresh:
            self.RotateCCW()
        elif (self.__current[3] - self.__prev[3])/self.__delta_t > epsilon:
            self.RotateCW()
        elif self.__current[1] < self.tunnel_tresh and self.__current[3] < self.tunnel_tresh:
            self.Tunnel(self.__current[1], self.__current[3])
        elif self.__current[3] > self.right_far_tresh:
            self.RotateCW_90()
        else:
            self.Fly_Forward()

    def GoHome(self, epsilon):
        pos = np.array([ ( (self.__local_pos[1] - self.min_x) / (self.max_x - self.min_x) ) * (abs(self.max_x) + abs(self.min_x) - 0) + 0 ,
                            ((self.__local_pos[0] - self.min_y) / (self.max_y - self.min_y) ) * (abs(self.max_y) + abs(self.min_y) - 0) + 0 ])
        
        if getDistance([pos[0], pos[1]], self.goal) > 5:
            if self.__current[0] < self.emengercy_tresh:
                self.Emengercy()
            elif self.__current[3] < epsilon:
                print("right")
                u_t_r = self.PID_rh.compute(self.__data['d_right'])        
                self.__controller.roll(u_t_r)
            elif self.__current[1] < epsilon:
                u_t_r = -1 * self.PID_rh.compute(self.__data['d_left'])        
                self.__controller.roll(u_t_r)
            
            self.target_point = self.traj.getTargetPoint([pos[0], pos[1]])
            yaw_err =  np.degrees(math.atan2(self.target_point[1] - pos[1], self.target_point[0] - pos[0])) - self.__rotation
            new_yaw = self.PID_y.compute(yaw_err)
            self.__controller.yaw(new_yaw)

            print('yaw_err: ', yaw_err)
            print('new_yaw: ', new_yaw)
            print('_________________________')

            if(abs(new_yaw) < 40):
                if abs(new_yaw) < 10:
                    self.PID_ph.set_params(0.3,0.0008,0.08)
                elif abs(new_yaw) < 20:
                    self.PID_ph.set_params(0.03,0.00008,0.008)
                elif abs(new_yaw) < 30:
                    self.PID_ph.set_params(0.003,0.000008,0.0008)
                else:
                    self.PID_ph.set_params(0.0003,0.0000008,0.00008)

                dis = math.hypot(self.target_point[1] - pos[1], self.target_point[0] - pos[0])
                new_pitch = self.PID_ph.compute(dis)
                
                print('pitch_error: ', dis)
                print('new_pitch: ', new_pitch)
                print('_________________________')
                self.__controller.pitch(new_pitch)

            
        else:
            print('enddddd!!!')

        
        
            

