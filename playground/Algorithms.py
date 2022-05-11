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
    
    def set_params(self, pid_obj):
        self.__integral = pid_obj.__integral
        self.__last_error = pid_obj.__last_error

    @property
    def integral(self):
        return self.__integral
    
    @property
    def last_error(self):
        return self.__last_error
    
class Algorithms:
    def __init__(self, controller, mode="random", delta_t=0.1):
        self.__mode = mode
        self.__auto = False
        self.__state = None
        
        self.__t_id = None
        self.__data = None
        
        self.__controller = controller
        self.__delta_t = delta_t

        self.to_draw_home = []
        self.to_draw = []

        self.cum_gyro = 0
        # self.__cum_distance = 0 
        # self.__local_pose =  np.array([0.0, 0.0]) # calc from opticalflow and gyro
        # self.__rotation_matrix = np.matmul(np.identity(3), create_rotation_matrix_yx(180)) 

        #PIDs
        self.PID_p = PID(1.5,0.004,0.4, disired_distance=0.4)
        self.PID_r = PID(4,4,2)
        
        #tunnel PIDs
        # self.PID_p_t = PID(1,0.006,0.6, disired_distance=0.3)
        # self.PID_r_t = PID(3,3,1.5, disired_distance=0.3)

        # keeps track of itersactions passed
        self.intersections = []
        self.delta_c = 0

        # BAT tresholds
        self.emengercy_tresh = 0.3
        self.tunnel_tresh = 0.75
        self.front_tresh = 1
        self.right_far_tresh = 2.5
        self.left_far_tresh = 2.5

    def draw(self, screen, h, w):
        for p in self.to_draw : #intersection while exploration
            position = to_screen_coords(h, w, p)
            pygame.draw.circle(screen, color=(0, 255, 0), center=position, radius=10)
        
        for p in self.to_draw_home : #intersction on the way back
            position = to_screen_coords(h, w, p)
            pygame.draw.circle(screen, color=(0, 0, 255), center=position, radius=10)
        
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
        
    # def follow_rotation(self, angle):
    #     # tr_mat = create_rotation_matrix_yx(angle)
    #     # self.__rotation_matrix = np.matmul(self.__rotation_matrix, tr_mat)
    #     self.__rotation_matrix = self.__controller.robot.rotation
        
    # def follow_local_pos(self):
    #     # x axis
    #     direction = make_direction(self.__rotation_matrix)
    #     # print("x_direction: " , direction)
    #     pos = self.__local_pose.copy()
    #     pos += direction * (self.__data['v_x'] * 100) / 2.5 * self.__delta_t
    #     # y axis
    #     direction = make_direction(np.matmul(self.__rotation_matrix, create_rotation_matrix_yx(90)))
    #     # print("y_direction: " , direction)

    #     pos += (-1 ) * direction * (self.__data['v_y'] * 100) / 2.5 * self.__delta_t

    #     self.__local_pose = pos
        
    #     #self.__cum_distance += math.sqrt(self.__controller.x**2 + self.__controller.y**2) * self.__delta_t

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

    def rotate180(self):
        self.__state = 'Rotate 180'
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
        ang = 0
        for _ in range(min_index_deg):
            self.RotateCCW()
            ang += 10
            # time.sleep(self.__delta_t)
                    
    def BAT(self):
        epsilon = 0.28
        delta_c_t = 0

        self.__controller.takeoff()
        if self.__data == None:
            self.sample_data()

        current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])
        prev = current.copy()

        home = current.copy()
        home[home == np.inf] = 3.0
        home = norm(home)
        home = [(home[0] + home[3])/2.0, (home[1] + home[2])/2.0, (home[0] + home[3])/2.0, (home[1] + home[2])/2.0] 
        homes = [home, np.array([home[1],home[2],home[3],home[0]]), home[::-1], np.array([home[3],home[0],home[1],home[2]])]
        
        while self.__auto and self.__data["battery"] > 55:
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
            
            if start_intersection(current, prev):
                self.cum_gyro += self.__data["gyro"]
            elif done_intersection(current, prev):
                norm_current = current.copy()
                norm_current[norm_current == np.inf] = 3.0
                norm_current = norm(norm_current)
                self.intersections.append((norm_current, delta_c_t, self.cum_gyro))
                self.to_draw.append(self.__controller.position)

                print('delta_c:',delta_c_t)
                # print("self.cum_gyro: ", self.cum_gyro)
                self.cum_gyro = 0
                delta_c_t = 0
                
            delta_c_t += self.__delta_t
            # print("local_pos: ", self.__local_pose)
            # print("distance: ", self.__cum_distance)
            # print("cum_rotatation: ", self.__cum_rotatation)

            time.sleep(self.__delta_t)
        
        # drone go home
        self.PID_p.reset()
        # self.PID_p_t.reset()
        self.PID_r.reset()
        # self.PID_r_t.reset()

        # stop movement
        self.__controller.pitch(0)
        self.__controller.roll(0)
        
        opt = [self.__data["v_x"], self.__data["v_y"]]
        while min(opt) > 0:
            opt = [self.__data["v_x"], self.__data["v_y"]]

        # rotate 180 degrees
        self.rotate180()
        
        # navigate home
        self.GoHome(homes)

   # retuns the mathcing the given intersection
    def match_intersection(self,current):
        inters_dist = [rmse(current, v[0]) for v in self.intersections]
        if len(inters_dist) > 0:
            return np.argmin(inters_dist)
        return None

    def GoHome(self, homes):
        delta_c_t = 0

        current = np.array([self.__data["d_front"], self.__data["d_left"], self.__data["d_back"], self.__data["d_right"]])        
        prev = current.copy()

        norm_current = current.copy()
        norm_current[norm_current == np.inf] = 3.0
        norm_current = norm(norm_current)
        rmses = [rmse(norm_current, v) for v in homes]
        
        epsilon = 0.28
        time_to_home = None
        while self.__auto and self.__data["battery"] > 0 : #and min(rmses) > 0.2:
            if time_to_home != None:
                delta_c_t += self.__delta_t
                if abs(delta_c_t - time_to_home) < epsilon:
                    break

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
            
            if start_intersection(current, prev):
                norm_current = current.copy()
                norm_current[norm_current == np.inf] = 3.0
                norm_current = norm(norm_current)
                if abs(delta_c_t - self.intersections[-1][1]) < epsilon:
                    print('delta_c:',delta_c_t)
                    delta_c_t = 0
                    if len(self.intersections) == 1:
                        time_to_home = self.intersections[0][1]
                        print("time_to_home: ", time_to_home)

                    self.intersections.pop()
                 

                elif delta_c_t > self.intersections[-1][1] - epsilon:
                    sum = 0
                    count = 0

                    for i in reversed(self.intersections):
                        sum += i[1]
                        count += 1
                        if sum - epsilon >= delta_c_t:
                            break

                    for i in range(count):
                        self.intersections.pop()
                        if len(self.intersections) == 1:
                            time_to_home = self.intersections[0][1]
                            print("time_to_home: ", time_to_home)

                    delta_c_t = 0

                self.to_draw_home.append(self.__controller.position)
                
            delta_c_t += self.__delta_t

            norm_current = current.copy()
            norm_current[norm_current == np.inf] = 3.0
            norm_current = norm(norm_current)
            rmses = [rmse(norm_current, v[0]) for v in homes]

            #print("local_pos: ", self.__local_pose)
            # print("min(rmses): ", min(rmses))

            time.sleep(self.__delta_t)

        if min(rmses) > 0.2:
            print("Drone return home")
            self.__controller.pitch(0)
            self.__controller.roll(0)

        self.__controller.land()