import random
import time
import threading
import numpy as np

class Algorithms:
    def __init__(self, controller, mode="random"):
        self.__mode = mode
        self.__auto = False
        self.__t_id = None
        self.__controller = controller
        self.last_error = 0
        self.intgral = 0
        self.pitch_Kp = 15
        self.pitch_Ki = 0.04
        self.pitch_Kd = 4
        
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
        self.RotateCW()
        self.Fly_Forward()
        

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
            data = self.__controller.sensors_data()
            
            right, left = data["d_right"], data["d_left"]
        
        # if counter != 0:
        #     if counter < 0:
        #         self.___controller.yaw(1)
        #     elif counter > 0:
        #         self.___controller.yaw(-1)
        

    def Fly_Forward(self):
        delta_t = 0.01
        data = self.__controller.sensors_data()
        error = data['d_front'] - 0.25
        if error == np.inf:
            # if front is inf the pid controller get crazy
            error = 3 - 0.25
        self.intgral = self.intgral + error * delta_t        
        u_t = self.pitch_Kp * error + self.pitch_Ki * self.intgral + self.pitch_Kd * (error - self.last_error) / delta_t
        
        pitch = self.__controller.sensors_data()['pitch']
        if pitch < u_t :
            self.__controller.pitch(1)
        elif pitch > u_t :
            self.__controller.pitch(-1)
        self.last_error = error

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
        
        if np.isinf(right):
            right = np.inf

        self.__controller.takeoff()


        while self.__auto:

            if front < emengercy_tresh:
                self.Emengercy()
            elif front < front_tresh:
                self.RotateCCW()
            elif left < tunnel_tresh and right < tunnel_tresh:
                self.Tunnel(right, left)
            elif right > right_far_tresh:
                self.RotateCW()
            else:
                self.Fly_Forward()

            data = self.__controller.sensors_data()
            front, right, left = data["d_front"], data["d_right"], data["d_left"]
            time.sleep(0.1)

        self.__controller.land()