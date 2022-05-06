from faulthandler import disable
import random
import time
import threading
import numpy as np
import math
import pygame
from playground.utils.transform import to_screen_coords

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

def is_intersection(dis):
    threshold = 2.5
    if dis[0] > threshold and dis[1] > threshold:
        return True
    if dis[0] > threshold and dis[3] > threshold:
        return True

    return False

def turn(prev,current):
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
        self.intgral = 0

        self.delta_t = delta_t
        self.__max_measurements = max_measurements
        self.__max_rotate = 10
        self.__min_rotate = -10
        self.__disired_distance = disired_distance
    
    def compute(self, measurement):
        error = measurement - self.__disired_distance
        if error == np.inf:
            # if front is inf the pid controller get crazy
            error = self.__max_measurements - self.__disired_distance
        self.intgral += error * self.delta_t
        # if self.intgral > 1:
        #     self.intgral = 0      
        u_t = self.Kp * error + self.Ki * self.intgral + self.Kd * (error - self.last_error) / self.delta_t
        self.last_error = error
        # print("self.intgral: ", self.intgral)

        if u_t < self.__min_rotate:
            u_t = self.__min_rotate
        if u_t > self.__max_rotate:
            u_t = self.__max_rotate

        return u_t
    
    def reset(self):
        self.last_error = 0
        self.intgral = 0
    
class Algorithms:
    def __init__(self, controller, mode="random", delta_t=0.1):
        self.__mode = mode
        self.__auto = False
        self.__t_id = None
        self.__controller = controller
        self.__delta_t = delta_t
        self.to_draw = []
        self.local_pos = (0,0)
        self.disable_roll = False

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

    def random_walk(self):
        self.__controller.takeoff()
        while self.__auto:
            self.__controller.yaw(random.choice([-1,1]))
            
            self.__controller.pitch(random.choice([-1,1]))

            self.__controller.roll(random.choice([-1,1]))
 
        self.__controller.land()

    def Emengercy(self):
        print("Emengercy state")
        self.__controller.pitch(-1)
        

    def Tunnel(self, left, right, PID_p_t, PID_r_t, wall_align = 'd_right'):
        print("Tunnel state")

        data = self.__controller.sensors_data()

        u_t_p_t = PID_p_t.compute(data['d_front'])
        self.__controller.pitch(u_t_p_t)
        
        # print("u_t_p_t: ", u_t_p_t)

        
        epsilon = 0.2


        if abs(right - left) > epsilon:
            u_t_r_t = PID_r_t.compute(data[wall_align])
            self.__controller.roll(u_t_r_t)

            # print("u_t_r: ", u_t_r_t)
            # print("right: ", right)
            # print("left: ", left)       

    def Fly_Forward(self,PID_p,PID_r, wall_align = 'd_right'):
        data = self.__controller.sensors_data()

        u_t_p = PID_p.compute(data['d_front'])
        self.__controller.pitch(u_t_p)
        
        # print("u_t_p: ", u_t_p)
        sign = 1
        if wall_align == 'd_left':
            sign = -1
        
        if not self.disable_roll :
            u_t_r = sign * PID_r.compute(data[wall_align])
            self.__controller.roll(u_t_r)

        #print("u_t_r: ", u_t_r)

    def RotateCCW(self):
        self.__controller.yaw(10)
    
    def RotateCW(self):
        self.__controller.yaw(-10)

    def RotateCW_90(self):
        # print("RotateCCW_90")
        self.__controller.yaw(-90)
    
    def RotateCCW_90(self):
        # print("RotateCCW_90")
        self.__controller.yaw(90)

    def BAT(self):
        epsilon = 0.30

        emengercy_tresh = 0.3
        tunnel_tresh = 0.75
        front_tresh = 1
        right_far_tresh = 2.5
        left_far_tresh = 2.5
        
        PID_p = PID(1.5,0.004,0.4, disired_distance=0.3)
        PID_r = PID(6,6,3)

        PID_p_t = PID(1,0.006,0.6, disired_distance=0.2)
        PID_r_t = PID(4,4,2, disired_distance=0.3)

        self.__controller.takeoff()
        self.local_pos = np.array([0.0, 0.0])
        
        data = self.__controller.sensors_data()
        current = np.array([data["d_front"], data["d_left"], data["d_back"], data["d_right"]])
        right_prev = current[3]

        home = current.copy()
        home[home == np.inf] = 3.0
        home = norm(home)
        
        home = [(home[0] + home[3])/2.0, (home[1] + home[2])/2.0, (home[0] + home[3])/2.0, (home[1] + home[2])/2.0] 
        homes = [home, np.array([home[1],home[2],home[3],home[0]]), home[::-1], np.array([home[3],home[0],home[1],home[2]])]
        
        intersections = []
        flag = False
        direction = 'front'
        while self.__auto and data["battery"] > 60:
            if current[3] < right_far_tresh:
                self.disable_roll = False

            if current[0] < emengercy_tresh:
                self.Emengercy()
            elif current[0] < front_tresh:
                self.RotateCCW()
            elif (current[3] - right_prev)/self.__delta_t > epsilon: #and front > front_tresh*1.5:
                #print("fix roll cw")
                self.RotateCW()
            elif current[1] < tunnel_tresh and current[3] < tunnel_tresh:
                self.Tunnel(current[1], current[3], PID_p_t, PID_r_t)
            elif current[3] > right_far_tresh:
                self.disable_roll = True
                self.RotateCW_90()
                direction = 'right'
            elif is_intersection(current):
                self.Fly_Forward(PID_p_t,PID_r_t)
                prev_dis = current.copy()
                if np.isinf(current[1]):
                    direction = 'left'
                is_turnning = True
            else:
                self.Fly_Forward(PID_p,PID_r)

            right_prev = current[3]
            data = self.__controller.sensors_data()
            current = np.array([data["d_front"], data["d_left"], data["d_back"], data["d_right"]])
            
            if done_turnning(direction, current):
                is_turnning = False
                t = turn(prev_dis, current)
                intersections.append((current, t))
                self.to_draw.append(self.__controller.position)

            self.local_pos[0] += data['v_y']
            self.local_pos[1] += data['v_x']

            time.sleep(self.__delta_t)
        
        # drone go home
        PID_p.reset()
        PID_p_t.reset()
        PID_r.reset()
        PID_r_t.reset()

        self.__controller.pitch(0)
        self.__controller.roll(0)
        
        # rotate 180 degrees
        data = self.__controller.sensors_data()
        current = np.array([data["d_front"], data["d_left"], data["d_back"], data["d_right"]])
        
        current[current == np.inf] = 3.0
        current = norm(current)
        
        rmses = []
        for deg in range(0, 360, 10):
            self.RotateCCW()
            data = self.__controller.sensors_data()
            rotate_current = np.array([data["d_front"], data["d_left"], data["d_back"], data["d_right"]])[::-1]
            rotate_current[rotate_current == np.inf] = 3.0
            rotate_current = norm(rotate_current)

            rmses.append(rmse(current,rotate_current))

            time.sleep(self.__delta_t)

        min_index_deg = rmses.index(min(rmses))
        for _ in range(min_index_deg):
            self.RotateCCW()

        left_prev = current[1]

        data = self.__controller.sensors_data()
        current = np.array([data["d_front"], data["d_left"], data["d_back"], data["d_right"]])        
        current[current == np.inf] = 3.0
        current = norm(current)
        rmses = [rmse(current, v) for v in homes]

        is_turnning = False
        direction = None
        while self.__auto and data["battery"] > 0 and min(rmses) > 0.1: #rmse(local_pos, np.array([0.0, 0.0])) > 1:
            if current[0] < emengercy_tresh:
                self.Emengercy()
            elif current[0] < front_tresh:
                self.RotateCW()
            elif (current[1] - left_prev)/self.__delta_t > epsilon:
                self.RotateCCW()
            elif current[1] < tunnel_tresh and current[3] < tunnel_tresh:
                self.Tunnel(current[1], current[3], PID_p_t, PID_r_t, wall_align='d_left')
            elif current[1] > left_far_tresh:
                self.RotateCCW_90()
            else:
                self.Fly_Forward(PID_p,PID_r, wall_align='d_left')

            left_prev = current[1]
            data = self.__controller.sensors_data()
            current = np.array([data["d_front"], data["d_left"], data["d_back"], data["d_right"]])
            if is_intersection(current) and not is_turnning:
                is_turnning = True
                direction = intersections[-1][1]
                if direction == 'right':
                    self.RotateCCW_90()
                elif direction == 'left':
                    self.RotateCW_90()
            elif done_turnning(direction, current):                    
                is_turnning = False
                intersections.remove(-1)

            current[current == np.inf] = 3.0
            current = norm(current)
            rmses = [rmse(current, v) for v in homes]

            print("min(rmses): ", min(rmses))
            print("current: ", current)

            time.sleep(self.__delta_t)
            

        self.__controller.land()