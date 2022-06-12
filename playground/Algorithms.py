import random
import statistics
import numpy as np
import math
import pygame
from playground.utils.transform import to_screen_coords, make_direction
import collections
from scipy import ndimage, interpolate
np.random.seed(42)
np.seterr("ignore")

from playground.pure_pursuit import *

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

def is_intersection(left, right):
    threshold = 2.2
    if left > threshold or right > threshold:
        return True
    return False

def start_intersection(dis, dis_prev, delta_t=0.1):
    threshold = 15
    dis[dis == np.inf] = 3.0
    dis_prev[dis_prev == np.inf] = 3.0
    if (dis[1] - dis_prev[1])/delta_t > threshold or (dis[3] - dis_prev[3])/delta_t > threshold:
        return True
    
    return False

def done_intersection(dis, dis_prev, delta_t=0.1):
    threshold = 15
    dis[dis == np.inf] = 3.0
    dis_prev[dis_prev == np.inf] = 3.0
    if (dis_prev[1] - dis[1])/delta_t > threshold or (dis_prev[3] - dis[3])/delta_t > threshold:
        return True
    
    return False

def getDistance(p1, p2):
    """
    Calculate distance
    :param p1: list, point1
    :param p2: list, point2
    :return: float, distance
    """
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)

def closest_intersection(local_pos, intersections):
    if intersections != None:
        return None

    closest = intersections[0]
    closest_dis = getDistance(local_pos, intersections[0])

    for inter in intersections:
        d = getDistance(local_pos, inter)
        if d < closest_dis:
            closest = inter
            closest_dis = d
    
    return closest, closest_dis

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
        self.PID_ph = PID(0.015,0.00004,0.004, disired_distance=10)
        self.PID_rh = PID(1.5 ,1.5 , 0.3)

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
        self.intersections = []
        self.last_intersection = []
        self.start_inter = False
        self.cum_delta_t = 0
        

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

        for p in self.intersections : #intersection while exploration
            position = to_screen_coords(h, w, p[0],start)
            pygame.draw.circle(screen, color=(0, 255, 0), center=position, radius=5)

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

        epsilon = 0.35
        self.__current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])

        if self.__data["battery"] > 50:
            self.BAT(epsilon)

        elif self.__first_go_home:
            self.PID_p.reset()
            self.PID_r.reset()
            self.__controller.pitch(0)
            self.__controller.roll(0)
            self.__controller.yaw(0)
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
            
        elif self.__data["battery"] <= 0:
            print("Timeout!")
            self.__controller.pitch(0)
            self.__controller.roll(0)
            self.__controller.yaw(0)
            self.__controller.land()

         # elif self.__data["battery"] > 0 and not self.__arrive_home:
        elif not self.__arrive_home:
            self.GoHome(epsilon)
        else:
            print("Drone returned home")
            self.__controller.pitch(0)
            self.__controller.roll(0)
            self.__controller.yaw(0)
            opt = [self.__data["v_x"], self.__data["v_y"]]
            if min(opt) > 0 and self.__data["yaw"] < 0.0:
                return
            self.__controller.land()
        
        self.__prev = self.__current.copy()


    def BAT(self, epsilon):

        self.__new_inter = True
        self.cum_delta_t += self.__delta_t

        if is_intersection(self.__current[1], self.__current[3]):
            self.last_intersection.append((self.__local_pos, self.__rotation))
            self.start_inter = True
            self.cum_delta_t = 0
        
        elif self.start_inter and self.cum_delta_t > 1:
            # done_intersection
            self.start_inter = False
            closest = closest_intersection(self.__local_pos, self.intersections)
            if closest != None:
                threshold = 120
                if closest[1] < threshold:
                    print("im here")
                    self.__new_inter = False
                    if closest[0][1] > 0 and self.__current[1] > 2.3:
                        self.RotateCW_90()
                    elif closest[0][1] < 0 and self.__current[3] > 2.3:
                        self.RotateCCW_90()
                    else:
                        self.Fly_Forward()
            else:
                avg_pos = np.mean(np.array([pair[0] for pair in self.last_intersection]), axis=0)
                diff_rot = self.last_intersection[-1][1] - self.last_intersection[0][1]
                self.intersections.append((avg_pos,diff_rot))
                self.last_intersection = []

        if self.__new_inter:
            if self.__current[0] < self.emengercy_tresh:
                self.Emengercy()
            elif self.__current[0] < self.front_tresh:
                self.RotateCCW()
            elif (self.__current[3] - self.__prev[3])/self.__delta_t > epsilon:
                print((self.__current[3] - self.__prev[3])/self.__delta_t)
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
        
        if getDistance([pos[0], pos[1]], self.goal) > L:
            if self.__current[0] < self.emengercy_tresh:
                self.Emengercy()

            self.target_point = self.traj.getTargetPoint([pos[0], pos[1]])
            yaw_err =  np.degrees(math.atan2(self.target_point[1] - pos[1], self.target_point[0] - pos[0])) - self.__rotation
            new_yaw = self.PID_y.compute(yaw_err)
            self.__controller.yaw(new_yaw)
            
            
            
            if(abs(new_yaw) < 60):
                if abs(new_yaw) < 10:
                    self.PID_ph.set_params(0.3,0.0008,0.08)
                elif abs(new_yaw) < 20:
                    self.PID_ph.set_params(0.03,0.00008,0.008)
                else:
                    self.PID_ph.set_params(0.003,0.000008,0.0008)

                dis = math.hypot(self.target_point[1] - pos[1], self.target_point[0] - pos[0])
                new_pitch = self.PID_ph.compute(dis)
                self.__controller.pitch(new_pitch)

                if self.__current[3] < self.__current[1]:
                    u_t_r = self.PID_rh.compute(self.__data['d_right'])        
                else:
                    u_t_r = -1 * self.PID_rh.compute(self.__data['d_left'])
                self.__controller.roll(u_t_r)
            else:
                self.__controller.pitch(0)
                self.__controller.roll(0)
        else:
            self.__arrive_home = True

        
        
            

