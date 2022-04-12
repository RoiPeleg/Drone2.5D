from playground.DroneController import DroneController
import random
import time
import threading

class Algorithms:
    def __init__(self, controller, mode="random"):
        self.__mode = mode
        self.__auto = False
        self.__t_id = None
        self.__controller = controller
        

    def run(self):
        self.__auto = True
        if self.__mode == "random":
            self.t_rd = threading.Thread(target=self.random_walk, args=())
        
        self.t_rd.start()

    def stop(self):
        self.__auto = False
        self.t_rd.join()
    
    def random_walk(self):
        self.__controller.takeoff()
        while self.__auto:
            self.__controller.yaw(random.choice([-1,1]))
            
            self.__controller.pitch(random.choice([-1,1]))
            time.sleep(0.8)

            # self.__controller.sensors_data()
 
        self.__controller.land()

    def Emengercy(self):
        pass

    def Tunnel(self):
        pass

    def Fly_Forward(self):
        pass

    def RotateCCW(self):
        self.__controller.yaw(-1)

    def RotateCW(self):
        for i in range(0,9):
            self.__controller.yaw(1)

    def BAT(self):
        emengercy_tresh = 0
        tunnel_tresh = 0
        front_tresh = 0
        right_far_tresh = 0


        front = -1 # distance of the front sensor of the drone
        right = -1 # distance of the right sensor of the drone
        left = -1 # distance of the left sensor of the drone
        while self.__auto:
            if front < emengercy_tresh:
                self.Emengercy()
            elif front < front_tresh:
                self.RotateCCW()
            elif left < tunnel_tresh and right < tunnel_tresh:
                self.Tunnel()
            elif right > right_far_tresh:
                self.RotateCW()
            else:
                self.Fly_Forward()