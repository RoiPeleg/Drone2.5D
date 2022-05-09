from faulthandler import disable
import random
import time
import threading
import numpy as np
import math
import pygame
from playground.utils.transform import to_screen_coords,create_rotation_matrix_yx, make_direction
np.random.seed(42)

def done_turnning(direction,current):
    treshold = 2.5
    done = False
    if direction == 'right' and treshold < current[1]:
        done = True
    elif direction == 'left' and treshold < current[3]:
        done = True
    elif treshold < current[1]:
        done = True
    return done 

def cosine_similarity(v1,v2):
    "compute cosine similarity of v1 to v2: (v1 dot v2)/{||v1||*||v2||)"
    sumxx, sumxy, sumyy = 0, 0, 0
    for i in range(len(v1)):
        x = v1[i]; y = v2[i]
        sumxx += x*x
        sumyy += y*y
        sumxy += x*y
    return sumxy/math.sqrt(sumxx*sumyy)

def rmse(a, b):
    MSE = np.square(np.subtract(a,b)).mean() 
    RMSE = np.sqrt(MSE)
    return RMSE

def norm(a):
    norm = np.linalg.norm(a)
    if norm == np.nan :
        norm = 0.001
    return a/norm

def align_to(local_pos):
    if local_pos[1] > 0: #x
        if local_pos[0] > 0: #y
            return ('left', 'down')
        elif local_pos[0] < 0: #y
            return ('left', 'up')
    elif local_pos[1] < 0: #x
        if local_pos[0] > 0: #y
            return ('right','down')
        elif local_pos[0] < 0: #y
            return ('right','up')

def is_intersection(dis, dis_prev, delta_t=0.1):
    threshold = 20
    if (dis[1] - dis_prev[1])/delta_t > threshold:
        return "left"
    elif (dis[3] - dis_prev[3])/delta_t > threshold:
        return "right"
    else:
        return None
    # if dis[0] > threshold and dis[1] > threshold:
    #     return True
    # if dis[0] > threshold and dis[3] > threshold:
    #     return True

    return False

def turn(prev ,current):
    threshold = 2.5
    cords = np.argwhere(prev > threshold)
    for c in cords:
        if threshold <= current[c]:
            if c == 0:
                return "front"
            elif c == 1:
                return "left"
            elif c ==2 :
                return "back"
            else :
                return "right"

    
class PID():
    def __init__(self, Kp, Ki, Kd, delta_t=0.1, max_measurements=3, disired_distance=0.5):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.last_error = 0
        self.__intgral = 0

        self.delta_t = delta_t
        self.__max_measurements = max_measurements
        self.__max_rotate = 10
        self.__min_rotate = -10
        self.__disired_distance = disired_distance
    
    def compute(self, measurement):
        error = measurement - self.__disired_distance
        if np.isinf(error):
            # if front is inf the pid controller get crazy
            error = self.__max_measurements - self.__disired_distance
        self.__intgral += error * self.delta_t
        if self.__intgral > 15:
            self.__intgral /= 10       
        u_t = self.Kp * error + self.Ki * self.intgral + self.Kd * (error - self.last_error) / self.delta_t
        self.last_error = error

        if u_t < self.__min_rotate:
            u_t = self.__min_rotate
        if u_t > self.__max_rotate:
            u_t = self.__max_rotate

        return u_t
    
    def reset(self):
        self.last_error = 0
        self.__intgral = 0

    @property
    def intgral(self):
        return self.__intgral
    
class Algorithms:
    def __init__(self, controller, mode="random", delta_t=0.1):
        self.__mode = mode
        self.__auto = False
        self.__state = None
        
        self.__t_id = None
        
        self.__controller = controller
        self.__delta_t = delta_t
        self.to_draw = []
        self.local_start = None
        self.disable_roll = False

        self.__cum_rotatation = 0 # calc from gyro
        self.__local_pose = np.array([0.0, 0.0]) # calc from opticalflow and gyro


        #PIDs
        self.PID_p = PID(1.5,0.04,0.04, disired_distance=0.3)
        self.PID_r = PID(3.0,3.0,2.0)
        
        #tunnel PIDs
        self.PID_p_t = PID(1.0,0.06,0.6,disired_distance=0.3)
        self.PID_r_t = PID(2.0,2.0,2.0/1.5,disired_distance=0.3)

        # keeps track of itersactions passed
        self.intersections = []
        
        # BAT tresholds
        self.emengercy_tresh = 0.3
        self.tunnel_tresh = 0.75
        self.front_tresh = 1
        self.right_far_tresh = 2.5
        self.left_far_tresh = 2.5

    def draw(self, screen, h, w):
        for p in self.to_draw :
            position = to_screen_coords(h, w, p)
            pygame.draw.circle(screen, color=(0, 255, 0), center=position, radius=10)
        
    def run(self):
        self.__auto = True
        if self.__mode == "random":
            self.__t_id = threading.Thread(target=self.random_walk, args=())
        if self.__mode == "bat":
            self.__t_id = threading.Thread(target=self.BAT, args=())

        self.__t_id.start()

    def stop(self):
        self.__auto = False
        self.__t_id.join()
    
    def sample_data(self):
        self.__data = self.__controller.sensors_data()            
    
    @property
    def state(self):
        return self.__state

    def random_walk(self):
        self.__controller.takeoff()
        while self.__auto:
            self.__controller.yaw(random.choice([-1,1]))
            
            self.__controller.pitch(random.choice([-1,1]))

            self.__controller.roll(random.choice([-1,1]))
 
        self.__controller.land()

    def Emengercy(self):
        self.__state = 'Emengercy'
        self.__controller.pitch(-1)
        
    def Tunnel(self, left, right, wall_align = 'd_right'):
                # print(self.PID_p.intgral)
        self.__state = 'Tunnel'
        u_t_p_t = self.PID_p_t.compute(self.__data['d_front'])
        self.__controller.pitch(u_t_p_t)        
        epsilon = 0.2
        if abs(right - left) > epsilon :
            u_t_r_t = self.PID_r_t.compute(self.__data[wall_align])
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
        angle = 10
        self.__controller.yaw(angle)
        self.__cum_rotatation += angle
    
    def RotateCW(self):
        self.__state = 'Rotate CW'
        angle = -10
        self.__controller.yaw(angle)
        self.__cum_rotatation += angle

    def RotateCW_90(self):
        self.__state = 'Rotate 90CW'
        angle = -90
        self.__controller.yaw(angle)
        self.__cum_rotatation += angle
    
    def RotateCCW_90(self):
        self.__state = 'Rotate 90CCW'
        angle = 90
        self.__controller.yaw(angle)
        self.__cum_rotatation += angle

    def rotate180(self):
        current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])
        current[current == np.inf] = 3.0
        current = norm(current)
        rmses = []
        for deg in range(0, 360, 10):
            self.RotateCCW()
            rotate_current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])[::-1]
            rotate_current[rotate_current == np.inf] = 3.0
            rotate_current = norm(rotate_current)
            rmses.append(rmse(current,rotate_current))
            time.sleep(self.__delta_t)

        min_index_deg = rmses.index(min(rmses))
        for _ in range(min_index_deg):
            self.RotateCCW()
            
    def BAT(self):
        epsilon = 0.28
        
        self.__controller.takeoff()
        self.sample_data()
        current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])
        prev = current.copy()

        home = current.copy()
        home[home == np.inf] = 3.0
        home = norm(home)
        home = [(home[0] + home[3])/2.0, (home[1] + home[2])/2.0, (home[0] + home[3])/2.0, (home[1] + home[2])/2.0] 
        homes = [home, np.array([home[1],home[2],home[3],home[0]]), home[::-1], np.array([home[3],home[0],home[1],home[2]])]
        
        is_turnning = False
        direction = 'front'
        while self.__auto and self.__data["battery"] > 60:
            self.sample_data()
            if is_intersection(current, prev) != None:
                is_turnning = True

            if current[0] < self.emengercy_tresh:
                self.Emengercy()
            elif current[0] < self.front_tresh:
                self.RotateCCW()
            elif (current[3] - prev[3])/self.__delta_t > epsilon: #and front > front_tresh*1.5:
                self.RotateCW()
            elif current[1] < self.tunnel_tresh and current[3] < self.tunnel_tresh:
                self.Tunnel(current[1], current[3])
            elif current[3] > self.right_far_tresh:
                self.RotateCW_90()
            else:
                self.Fly_Forward()
            prev = current.copy()
            current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])
            
            direction = is_intersection(current, prev)
            if is_turnning and direction == None:
                is_turnning = False
                t = turn(prev, current)
                norm_current = current.copy()
                norm_current[norm_current == np.inf] = 3.0
                norm_current = norm(norm_current)
                self.intersections.append((norm_current, t))
                self.to_draw.append(self.__controller.position)

            time.sleep(self.__delta_t)
        
        # drone go home
        self.PID_p.reset()
        self.PID_p_t.reset()
        self.PID_r.reset()
        self.PID_r_t.reset()

        # stop movement
        self.__controller.pitch(0)
        self.__controller.roll(0)
        
        # rotate 180 degrees
        if 80 < self.__cum_rotatation < 100:
            self.rotate180()
        
        # navigate home
        self.GoHome(homes)

   # retuns the mathcing the given intersection
    def match_intersection(self,current):
        inters_dist = [rmse(current, v[0]) for v in self.intersections]
        print(inters_dist)
        return np.argmin(inters_dist)

    def GoHome(self, homes):
        self.sample_data()
        current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])        
        prev = current.copy()

        norm_current = current.copy()
        norm_current[norm_current == np.inf] = 3.0
        norm_current = norm(norm_current)
        rmses = [rmse(norm_current, v) for v in homes]
        
        is_turnning = False
        direction = None
        
        epsilon = 0.3
        while self.__auto and self.__data["battery"] > 0 and min(rmses) > 0.1:
            self.sample_data()
            if current[0] < self.emengercy_tresh:
                self.Emengercy()
            elif current[0] < self.front_tresh:
                self.RotateCW()
            elif (current[1] - prev[1])/self.__delta_t > epsilon:
                self.RotateCCW()
            elif current[1] < self.tunnel_tresh and current[3] < self.tunnel_tresh:
                self.Tunnel(current[1], current[3], wall_align='d_left')
            elif current[1] > self.left_far_tresh:
                self.RotateCCW_90()
            else:
                self.Fly_Forward(wall_align='d_left')

            prev = current.copy()

            current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])
            
            if is_intersection(current,prev) and not is_turnning:
                is_turnning = True
                norm_current = current.copy()
                norm_current[norm_current == np.inf] = 3.0
                norm_current = norm(norm_current)
                direction = self.intersections[self.match_intersection(norm_current)][1]
                
                if direction == 'right':
                    self.RotateCCW_90()
                elif direction == 'left':
                    self.RotateCW_90()
            elif done_turnning(direction, current):                    
                is_turnning = False

            norm_current = current.copy()
            norm_current[norm_current == np.inf] = 3.0
            norm_current = norm(norm_current)
            rmses = [rmse(norm_current, v[0]) for v in homes]
            
            time.sleep(self.__delta_t)

        self.__controller.land()