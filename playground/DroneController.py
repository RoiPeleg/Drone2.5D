from playground.robot import Robot
import time
import threading


# Drone API
class DroneController:
    def __init__(self, __robot,__sensor_view):
        self.__robot = __robot
        self.__sensor_view = __sensor_view

        self.__max_velocity = 120 # 3 meter/second
        self.__pitch = 0 
        self.__roll = 0
        self.__yaw = 0
        # self.__acceleration = 1 # m/s^2
        # self.__angular_velocity = 100 # angle

        # self.__time_to_change = self.__angular_velocity / self.__acceleration

        self.t_move = threading.Thread(target=self.move, args=())
        self.__running = True


    def move(self):
        while self.__running:
            velocity = self.__robot.speed

            dist = velocity / 2.5
            # velocity = self.__pitch/100
            self.__robot.move(dist)   
    
    def stop(self):
        self.__running = False
        self.t_move.join()

    def yaw(self,dieraction):
        if dieraction == 1:
            self.__robot.rotate(10)
        else:
            self.__robot.rotate(-10)

    def pitch(self,sign):
        vel_dis = 10
        if sign == 1:
            new_velocity = min(self.__max_velocity, self.__robot.speed + vel_dis)
            self.__robot.c_speed(new_velocity)
        else:
            new_velocity = max(0, self.__robot.speed - vel_dis)
            self.__robot.c_speed(new_velocity)

    def roll(self):
        pass

    def takeoff(self):
        time.sleep(1)
        self.__robot.set_altitude(1)
        self.t_move.start()
      

    def land(self):
        time.sleep(1)
        self.__robot.set_altitude(0)

    def battery_level(self):
        return self.__sensor_view.battery
    
    def sensors_data(self):
        Z = self.__sensor_view.drone_height
        bat = self.__sensor_view.battery
        return [Z,bat]