import random
import time
import threading
import numpy as np

class PID():
    def __init__(self, Kp, Ki, Kd, delta_t=0.01, max_measurements=3, disired_distance=0.3):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.last_error = 0
        self.intgral = 0

        self.delta_t = delta_t
        self.__max_measurements = max_measurements
        self.__disired_distance = disired_distance
    
    def compute(self, measurement):
        error = measurement - self.__disired_distance
        if error == np.inf:
            # if front is inf the pid controller get crazy
            error = self.__max_measurements - self.__disired_distance
        self.intgral = self.intgral + error * self.delta_t        
        u_t = self.Kp * error + self.Ki * self.intgral + self.Kd * (error - self.last_error) / self.delta_t
        self.last_error = error
        return u_t
    
class Algorithms:
    def __init__(self, controller, mode="random"):
        self.__mode = mode
        self.__auto = False
        self.__t_id = None
        self.__controller = controller
        
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

    def Emengercy(self,PID_p,PID_r):
        self.RotateCW()
        self.Fly_Forward(PID_p,PID_r)
        

    def Tunnel(self, right, left):
        print("The drone enter to a tunnel")
        epsilon = 25
        counter = 0
        if abs(right - left) < epsilon:
            if right < left:
                self.__controller.roll(1)
                counter += 1
            elif left < right:
                self.___controller.roll(-1)
                counter -= 1

            # data = self.__controller.sensors_data()
            #right, left = data["d_right"], data["d_left"]
        
        # if counter != 0:
        #     if counter < 0:
        #         self.___controller.yaw(1)
        #     elif counter > 0:
        #         self.___controller.yaw(-1)
        

    def Fly_Forward(self,PID_p,PID_r):
        data = self.__controller.sensors_data()

        u_t_p = PID_p.compute(data['d_front'])
        pitch = self.__controller.sensors_data()['pitch']
        if pitch < u_t_p :
            self.__controller.pitch(1)
        elif pitch > u_t_p :
            self.__controller.pitch(-1)
        
        print("u_t_p: ", u_t_p)

        u_t_r = PID_r.compute(data['d_right'])
        roll = self.__controller.sensors_data()['roll']
        if roll < u_t_r :
            self.__controller.roll(1)
        elif roll > u_t_r :
            self.__controller.roll(-1)

        print("u_t_r: ", u_t_r)

    def RotateCCW(self):
        self.__controller.yaw(1)
        
    def RotateCW(self):
        for _ in range(0,10):
            self.__controller.yaw(-1)

    def BAT(self):
        emengercy_tresh = 0.3
        tunnel_tresh = 0.25
        front_tresh = 1
        right_far_tresh = 2.5

        front = -1 # distance of the front sensor of the drone
        right = -1 # distance of the right sensor of the drone
        left = -1 # distance of the left sensor of the drone
        data = self.__controller.sensors_data()
        front, right, left = data["d_front"], data["d_right"], data["d_left"]
        PID_p = PID(15,0.04,4)
        PID_r = PID(60,60,10)
        self.__controller.takeoff()
        
        while self.__auto:
            if front < emengercy_tresh:
                self.Emengercy(PID_p,PID_r)
            elif front < front_tresh:
                self.RotateCCW()
            elif left < tunnel_tresh and right < tunnel_tresh:
                self.Tunnel(right, left)
            elif right > right_far_tresh:
                self.RotateCW()
            else:
                self.Fly_Forward(PID_p,PID_r)

            data = self.__controller.sensors_data()
            front, right, left = data["d_front"], data["d_right"], data["d_left"]
            time.sleep(0.1)

        self.__controller.land()