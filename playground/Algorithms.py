import random
import time
import threading
import numpy as np

    
def cosine_similarity(a, b):
    return np.dot(a, b)/(np.linalg.norm(a)*np.linalg.norm(b))

def rmse(a, b):
    MSE = np.square(np.subtract(a,b)).mean() 
    RMSE = np.sqrt(MSE)
    return RMSE

def norm(a):
    return a / np.sqrt(np.sum(a**2))

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
        

    def Tunnel(self, right, left, PID_p_t, PID_r_t, wall_align = 'd_right'):
        print("Tunnel state")

        data = self.__controller.sensors_data()

        u_t_p_t = PID_p_t.compute(data['d_front'])
        self.__controller.pitch(u_t_p_t)
        
        print("u_t_p_t: ", u_t_p_t)

        
        epsilon = 0.2

        # self.__controller.pitch(3)

        if abs(right - left) > epsilon:
            u_t_r_t = PID_r_t.compute(data[wall_align])
            self.__controller.roll(u_t_r_t)

            print("u_t_r: ", u_t_r_t)

            print("right: ", right)
            print("left: ", left)
        else:
            return False
        #     if right < left:
        #         self.RotateCCW()
        #     elif left < right:
        #         self.RotateCW()
        

    def Fly_Forward(self,PID_p,PID_r, wall_align = 'd_right'):
        data = self.__controller.sensors_data()

        u_t_p = PID_p.compute(data['d_front'])
        self.__controller.pitch(u_t_p)
        
        # print("u_t_p: ", u_t_p)

        u_t_r = PID_r.compute(data[wall_align])
        self.__controller.roll(u_t_r)

        # print("u_t_r: ", u_t_r)

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
        counter = 2

        emengercy_tresh = 0.3
        tunnel_tresh = 0.75
        front_tresh = 1
        right_far_tresh = 2.5
        left_far_tresh = 2.5
        
        data = self.__controller.sensors_data()
        front, right, left = data["d_front"], data["d_right"], data["d_left"]
        right_prev = right
        PID_p = PID(1.5,0.004,0.4, disired_distance=0.3)
        PID_r = PID(6,6,3)

        PID_p_t = PID(1,0.006,0.6, disired_distance=0.2)
        PID_r_t = PID(4,4,2, disired_distance=0.3)

        self.__controller.takeoff()
        while self.__auto and data["battery"] > 50:
            if front < emengercy_tresh:
                self.Emengercy()
            elif front < front_tresh:
                self.RotateCCW()
            elif (right - right_prev)/self.__delta_t > epsilon: #and front > front_tresh*1.5:
                print("fix roll cw")
                self.RotateCW()
            elif left < tunnel_tresh and right < tunnel_tresh:
                self.Tunnel(right, left, PID_p_t, PID_r_t)
            elif right > right_far_tresh:
                self.RotateCW_90()
            else:
                self.Fly_Forward(PID_p,PID_r)

            right_prev = right
            data = self.__controller.sensors_data()
            front, right, left = data["d_front"], data["d_right"], data["d_left"]
            time.sleep(self.__delta_t)
        
        # drone go home
        
        # while not self.Tunnel(right, left, PID_p_t, PID_r_t, wall_align='d_left'):
        #     time.sleep(self.__delta_t)

        PID_p.reset()
        PID_p_t.reset()
        PID_r.reset()
        PID_r_t.reset()

        self.__controller.pitch(0)
        self.__controller.roll(0)
        
        data = self.__controller.sensors_data()
        front, right, left, back = data["d_front"], data["d_right"], data["d_left"], data["d_back"]
        
        a = np.array([front, right, left, back])
        a[a == np.inf] = 3.0
        a = norm(a)
        b = np.ones_like(a)

        while self.__auto and rmse(a,b) > 0.05:
            self.RotateCCW()
            
            data = self.__controller.sensors_data()
            front, right, left, back = data["d_front"], data["d_right"], data["d_left"], data["d_back"]
            b = np.array([front, right, left, back])[::-1]
            b[b == np.inf] = 3.0
            b = norm(b)
            time.sleep(self.__delta_t)

        left_prev = left
        while self.__auto and data["battery"] > 0:
            if front < emengercy_tresh:
                self.Emengercy()
            elif front < front_tresh:
                self.RotateCW()
            elif (left - left_prev)/self.__delta_t > epsilon:
                print("fix roll cw")
                self.RotateCCW()
            elif left < tunnel_tresh and right < tunnel_tresh:
                self.Tunnel(right, left, PID_p_t, PID_r_t, wall_align='d_left')
            elif left > left_far_tresh:
                self.RotateCCW_90()
            else:
                self.Fly_Forward(PID_p,PID_r, wall_align='d_left')

            left_prev = left
            data = self.__controller.sensors_data()
            front, right, left = data["d_front"], data["d_right"], data["d_left"]
            time.sleep(self.__delta_t)

        self.__controller.land()