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
            d = random.randint(-1,1)
            self.__controller.yaw(d)
            time.sleep(0.8)
        self.__cont__controller.land()