from faulthandler import disable
import random
import time
import threading
import numpy as np
import math
import pygame
from playground.utils.transform import to_screen_coords,create_rotation_matrix_yx, make_direction
np.random.seed(42)


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

def start_intersection(dis, dis_prev, delta_t=0.1):
    threshold = 20
    dis[dis == np.inf] = 3.0
    dis_prev[dis_prev == np.inf] = 3.0
    if (dis[1] - dis_prev[1])/delta_t > threshold or (dis[3] - dis_prev[3])/delta_t > threshold:
        return True
    
    return False

def done_intersection(dis, dis_prev, delta_t=0.1):
    threshold = 20
    dis[dis == np.inf] = 3.0
    dis_prev[dis_prev == np.inf] = 3.0
    if (dis_prev[1] - dis[1])/delta_t > threshold or (dis_prev[3] - dis[3])/delta_t > threshold:
        return True
    
    return False

    
class PID():
    def __init__(self, Kp, Ki, Kd, delta_t=0.1, max_measurements=3, disired_distance=0.5):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.__last_error = 0
        self.__integral = 0

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
        self.__integral += error * self.delta_t
        if error == 0:
            self.__integral = 0
        if self.__integral > 15:
            self.__integral /= 10       
        u_t = self.Kp * error + self.Ki * self.__integral + self.Kd * (error - self.__last_error) / self.delta_t
        self.__last_error = error

        if u_t < self.__min_rotate:
            u_t = self.__min_rotate
        if u_t > self.__max_rotate:
            u_t = self.__max_rotate

        return u_t
    
    def reset(self):
        self.__last_error = 0
        self.__integral = 0
    
class Algorithms:
    def __init__(self, controller, delta_t=0.1):
        
        self.__controller = controller
        self.__state = None
        self.__data = None
        
        # step control indcators  
        self.__first_step = True
        self.__first_go_home = True
        self.__second_go_home = True
        self.__arrive_home = False
        
        # time constant
        self.__delta_t = delta_t
        
        # self.__homes = None
        # BAT and goHome
        self.__time_to_home = None
        self.__current = None
        self.__prev = None
        
        # points to draw
        self.to_draw_home = []
        self.to_draw = []

        self.cum_gyro = 0
        # self.__cum_distance = 0 
        # self.__local_pose =  np.array([0.0, 0.0]) # calc from opticalflow and gyro
        # self.__rotation_matrix = np.matmul(np.identity(3), create_rotation_matrix_yx(180)) 

        #PIDs
        # self.PID_p = PID(1.5,0.004,0.4, disired_distance=0.4)
        # self.PID_r = PID(4,4,2)

        self.PID_p = PID(1.5,0.004,0.4, disired_distance=0.1)
        self.PID_r = PID(6.0,6.0,2.0)

        # keeps track of itersactions passed
        self.intersections = []
        self.__delta_c_t = 0

        # BAT tresholds
        self.emengercy_tresh = 0.3
        self.tunnel_tresh = 0.75
        self.front_tresh = 1
        self.right_far_tresh = 2.5
        self.left_far_tresh = 2.5

        #rotate180
        self.__rmses = []
        self.__rotate_back = 0
        self.__number_of_rotations = 0
        self.__done180 = False

    @property
    def state(self):
        return self.__state

    def draw(self, screen, h, w):
        for p in self.to_draw : #intersection while exploration
            position = to_screen_coords(h, w, p)
            pygame.draw.circle(screen, color=(0, 255, 0), center=position, radius=10)
        
        for p in self.to_draw_home : #intersction on the way back
            position = to_screen_coords(h, w, p)
            pygame.draw.circle(screen, color=(0, 0, 255), center=position, radius=10)
  
    # def follow_rotation(self, angle):
    #     tr_mat = create_rotation_matrix_yx(angle)
    #     self.__rotation_matrix = np.matmul(self.__rotation_matrix, tr_mat)
    #     # self.__rotation_matrix = self.__controller.robot.rotation
        
    # def follow_local_pos(self):
    #     # x axis
    #     direction = make_direction(self.__rotation_matrix)
    #     # print("x_direction: " , direction)
    #     pos = self.__local_pose.copy()
    #     # pos += direction * (self.__data['v_x'] * 100) / 2.5 * self.__delta_t
    #     pos += direction * self.__controller.x
    #     # y axis
    #     direction = make_direction(np.matmul(self.__rotation_matrix, create_rotation_matrix_yx(-90)))
    #     # print("y_direction: " , direction)
    #     pos += direction * self.__controller.y 
    #     # pos += (-1 ) * direction * (self.__data['v_y'] * 100) / 2.5 * self.__delta_t

    #     self.__local_pose = pos
        
    #   #self.__cum_distance += math.sqrt(self.__controller.x**2 + self.__controller.y**2) * self.__delta_t

    def sample_data(self):
        self.__data = self.__controller.sensors_data()
        self.__current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])
        if self.__first_step:
            self.__prev = self.__current.copy()

    # retuns the mathcing the given intersection
    def match_intersection(self,current):
        inters_dist = [rmse(current, v[0]) for v in self.intersections]
        if len(inters_dist) > 0:
            return np.argmin(inters_dist)
        return None

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
        epsilon = 0.2
        if abs(right - left) > epsilon :
            u_t_r_t = self.PID_r.compute(self.__data[wall_align])
            self.__controller.roll(u_t_r_t)
            
        # self.follow_local_pos()

    def Fly_Forward(self, wall_align = 'd_right'):
        self.__state = 'Forward'
        u_t_p = self.PID_p.compute(self.__data['d_front'])
        self.__controller.pitch(u_t_p)
        sign = 1
        if wall_align == 'd_left':
            sign = -1
        u_t_r = sign * self.PID_r.compute(self.__data[wall_align])        
        self.__controller.roll(u_t_r)
        
        # self.follow_local_pos()

    def RotateCCW(self):
        self.__state = 'Rotate CCW'
        self.__controller.yaw(10)
        # self.follow_rotation(10)
    
    def RotateCW(self):
        self.__state = 'Rotate CW'
        self.__controller.yaw(-10)
        # self.follow_rotation(-10)

    def RotateCW_90(self):
        self.__state = 'Rotate 90CW'
        self.__controller.yaw(-90)
        # self.follow_rotation(-90)
    
    def RotateCCW_90(self):
        self.__state = 'Rotate 90CCW'
        self.__controller.yaw(90)
        # self.follow_rotation(90)

    # def rotate180(self):
    #     self.__state = 'Rotate 180'
    #     if self.__number_of_rotations < 36:
    #         self.RotateCCW()
    #         rotate_current = self.__current.copy()[::-1]
    #         rotate_current[rotate_current == np.inf] = 3.0
    #         rotate_current = norm(rotate_current)
    #         self.__rmses.append(rmse(self.__current, rotate_current))
    #         self.__number_of_rotations += 1
    #     else:
    #         min_index_deg = self.__rmses.index(min(self.__rmses))
            
    #         if self.__rotate_back < min_index_deg:
    #             self.RotateCCW()
    #             self.__rotate_back += 1
    #         else :
    #             self.__done180 = True

    def step(self):
        if self.__first_step:
            self.__controller.takeoff()

            home = self.__current.copy()
            home[home == np.inf] = 3.0
            home = norm(home)
            home = [(home[0] + home[3])/2.0, (home[1] + home[2])/2.0, (home[0] + home[3])/2.0, (home[1] + home[2])/2.0] 
            self.__homes = [home, np.array([home[1],home[2],home[3],home[0]]), home[::-1], np.array([home[3],home[0],home[1],home[2]])]
            self.__first_step = False

        epsilon = 0.35
        self.__current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])

        if self.__data["battery"] > 55:
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
            if not self.__done180 :
                # self.rotate180()
                self.RotateCW_90()
                self.RotateCW_90()
                self.__done180 = True
            else:
                self.__second_go_home = False
            
        elif self.__data["battery"] > 0 and not self.__arrive_home:
            self.GoHome(epsilon)

        elif self.__data["battery"] <= 0:
            print("Timeout!")
            self.__controller.pitch(0)
            self.__controller.roll(0)
            self.__controller.land()

        else:
            print("Drone returned home")
            self.__controller.pitch(0)
            self.__controller.roll(0)
            self.__controller.land()
        
        self.__prev = self.__current.copy()


    def BAT(self, epsilon):
        if self.__current[0] < self.emengercy_tresh:
            self.Emengercy()
        elif self.__current[0] < self.front_tresh:
            self.RotateCCW()
        elif (self.__current[3] - self.__prev[3])/self.__delta_t > epsilon: #and front > front_tresh*1.5:
            self.RotateCW()
        elif self.__current[1] < self.tunnel_tresh and self.__current[3] < self.tunnel_tresh:
            self.Tunnel(self.__current[1], self.__current[3])
        elif self.__current[3] > self.right_far_tresh:
            self.RotateCW_90()
        else:
            self.Fly_Forward()
        
        if start_intersection(self.__current, self.__prev):
            self.cum_gyro += self.__data["gyro"]
        elif done_intersection(self.__current, self.__prev):
            norm_current = self.__current.copy()
            norm_current[norm_current == np.inf] = 3.0
            norm_current = norm(norm_current)
            self.intersections.append((norm_current, self.__delta_c_t, self.cum_gyro))
            self.to_draw.append(self.__controller.position)

            # print('delta_c:', self.__delta_c_t)
            self.cum_gyro = 0
            self.__delta_c_t = 0
        self.__delta_c_t += self.__delta_t 

    def GoHome(self, epsilon):
        self.__delta_c_t = 0
        
        # if self.__time_to_home != None:
        #     self.__delta_c_t += self.__delta_t
        #     # if abs(self.__delta_c_t - self.__time_to_home) < epsilon:
        #     if 
        #         self.__arrive_home = True
        #         return
        
        norm_current = self.__current.copy()
        norm_current[norm_current == np.inf] = 3.0
        norm_current = norm(norm_current)
        rmses = [rmse(norm_current, v[0]) for v in self.__homes]
        
        print("min(rmses): ", min(rmses))

        if min(rmses) < 0.1:
                self.__arrive_home = True
                
        if self.__current[0] < self.emengercy_tresh:
            self.Emengercy()
        elif self.__current[0] < self.front_tresh:
            self.RotateCW()
        elif (self.__current[1] - self.__prev[1])/self.__delta_t > epsilon:
            self.RotateCCW()
        elif self.__current[1] < self.tunnel_tresh and self.__current[3] < self.tunnel_tresh:
            self.Tunnel(self.__current[1], self.__current[3], wall_align='d_left')
        elif self.__current[1] > self.left_far_tresh:
            self.RotateCCW_90()
        else:
            self.Fly_Forward(wall_align='d_left')

            
        if start_intersection(self.__current, self.__prev):
            norm_current = self.__current.copy()
            norm_current[norm_current == np.inf] = 3.0
            norm_current = norm(norm_current)
            if abs(self.__delta_c_t - self.intersections[-1][1]) < epsilon:
                print('delta_c:',self.__delta_c_t)
                self.__delta_c_t = 0
                if len(self.intersections) == 1:
                    self.__time_to_home = self.intersections[0][1]
                    print("time_to_home: ", self.__time_to_home)

                self.intersections.pop()
                
            elif self.__delta_c_t > self.intersections[-1][1] - epsilon:
                sum = 0
                count = 0

                for i in reversed(self.intersections):
                    sum += i[1]
                    count += 1
                    if sum - epsilon >= self.__delta_c_t:
                        break

                for i in range(count):
                    self.intersections.pop()
                    if len(self.intersections) == 1:
                        time_to_home = self.intersections[0][1]
                        print("time_to_home: ", time_to_home)

                self.__delta_c_t = 0

            self.to_draw_home.append(self.__controller.position)
                
            self.__delta_c_t += self.__delta_t
            

