from playground.DroneController import DroneController
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
        

    def run(self):
        self.__auto = True
        if self.__mode == "random":
            self.t_id = threading.Thread(target=self.random_walk, args=())
        if self.__mode == "bat":
            self.t_id = threading.Thread(target=self.BAT, args=())
        self.t_id.start()

    def stop(self):
        self.__auto = False
        self.t_id.join()
    
    def random_walk(self):
        self.__controller.takeoff()
        while self.__auto:
            self.__controller.yaw(random.choice([-1,1]))
            
            self.__controller.pitch(random.choice([-1,1]))
            time.sleep(0.8)

            # self.__controller.sensors_data()
 
        self.__controller.land()

    def Emengercy(self):
        self.RotateCW()
        self.Fly_Forward()
        pass

    def Tunnel(self, right, left):
        # epsilon = 0.1
        # while abs(right - left) < epsilon:
        #     self.Fly_Forward()
        #     data = self.__controller.sensors_data()
        #     front, right, leff = data["d_front"], data["d_right"], data["d_left"]
        pass

    def Fly_Forward(self):
        self.__controller.pitch(1)

    def RotateCCW(self):
        self.__controller.yaw(-1)

    def RotateCW(self):
        for _ in range(0,9):
            self.__controller.yaw(1)

    def BAT(self):
        emengercy_tresh = 0.3
        tunnel_tresh = 0.25
        front_tresh = 1
        right_far_tresh = 2.5

        front = -1 # distance of the front sensor of the drone
        right = -1 # distance of the right sensor of the drone
        left = -1 # distance of the left sensor of the drone
        data = self.__controller.sensors_data()
        front, right, leff = data["d_front"], data["d_right"], data["d_left"]
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
            # print("front: ", front, "right: ", right, "left: ", left)
            print(data)

            time.sleep(1)

        self.__controller.land()