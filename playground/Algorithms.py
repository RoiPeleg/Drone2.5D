import random
import time
import threading
import numpy as np

class Algorithms:
    def __init__(self, controller, mode="random"):
        self.__mode = mode
        self.__auto = False
        self.__t_id = None
        self.__careful_id = None
        self.__controller = controller
        

    def run(self):
        self.__auto = True
        self.__careful_id = threading.Thread(target=self.careful_walk, args=())
        if self.__mode == "random":
            self.__t_id = threading.Thread(target=self.random_walk, args=())
        if self.__mode == "bat":
            self.__t_id = threading.Thread(target=self.BAT, args=())
        self.__t_id.start()
        self.__careful_id.start()

    def stop(self):
        self.__auto = False
        self.__t_id.join()
        self.__careful_id.join()
    
    def random_walk(self):
        self.__controller.takeoff()
        while self.__auto:
            self.__controller.yaw(random.choice([-1,1]))
            
            self.__controller.pitch(random.choice([-1,1]))

            self.__controller.roll(random.choice([-1,1]))
 
        self.__controller.land()

    def careful_walk(self):
        t = 0.1
        pid = self.PID(2, 0.1, 2)        # create pid control
        pid.send(None)

        while self.__auto:
            data = self.__controller.sensors_data()
            SP = 5           # get setpoint
            PV = data["d_right"]               # get measurement
            print("PV: ", PV)

            MV = pid.send([t, PV, SP])   # compute manipulated variable
            # apply
            print("MV: ", MV)

    def PID(self, Kp, Ki, Kd, MV_bar=0):
        # initialize stored data
        e_prev = 0
        t_prev = 0
        I = 0
        
        # initial control
        MV = MV_bar
        
        while True:
            # yield MV, wait for new t, PV, SP
            t, PV, SP = yield MV
            
            # PID calculations
            e = SP - PV
            
            print("in pid: SP: ", SP)
            print("in pid: PV: ", PV)
            print("in pid: e: ", e)

            P = Kp*e
            I = I + Ki*e*(t - t_prev)
            D = Kd*(e - e_prev)/(t - t_prev)
            
            MV = MV_bar + P + I + D
            
            # update stored data for next iteration
            e_prev = e
            t_prev = t + t

    def Emengercy(self):
        self.RotateCW()
        self.Fly_Forward()

        self.__controller.reset_movement()

    def Tunnel(self, right, left):
        # epsilon = 0.1
        # while abs(right - left) < epsilon:
        #     self.Fly_Forward()
        #     data = self.__controller.sensors_data()
        #     front, right, leff = data["d_front"], data["d_right"], data["d_left"]
        pass

    def Fly_Forward(self):
        self.__controller.pitch(1)

        self.__controller.reset_movement()

    def RotateCCW(self):
        self.__controller.yaw(-1)

        self.__controller.reset_movement()

    def RotateCW(self):
        for _ in range(0,10):
            self.__controller.yaw(1)
        
        self.__controller.reset_movement()

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
            # time.sleep(1)

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

        self.__controller.land()