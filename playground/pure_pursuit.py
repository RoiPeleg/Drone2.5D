import math
import matplotlib.pyplot as plt
import numpy as np


L = 30  # look ahead distance
dt = 0.1  # discrete time

# Vehicle parameters (m)
LENGTH = 4.5
WIDTH = 2.0
BACKTOWHEEL = 1.0
WHEEL_LEN = 0.3
WHEEL_WIDTH = 0.2
TREAD = 0.7
WB = 2.5


def plotVehicle(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):
    """
    The function is to plot the vehicle
    it is copied from https://github.com/AtsushiSakai/PythonRobotics/blob/187b6aa35f3cbdeca587c0abdb177adddefc5c2a/PathTracking/model_predictive_speed_and_steer_control/model_predictive_speed_and_steer_control.py#L109
    """
    outline = np.array(
        [
            [
                -BACKTOWHEEL,
                (LENGTH - BACKTOWHEEL),
                (LENGTH - BACKTOWHEEL),
                -BACKTOWHEEL,
                -BACKTOWHEEL,
            ],
            [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2],
        ]
    )

    fr_wheel = np.array(
        [
            [WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
            [
                -WHEEL_WIDTH - TREAD,
                -WHEEL_WIDTH - TREAD,
                WHEEL_WIDTH - TREAD,
                WHEEL_WIDTH - TREAD,
                -WHEEL_WIDTH - TREAD,
            ],
        ]
    )

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array(
        [[math.cos(steer), math.sin(steer)], [-math.sin(steer), math.cos(steer)]]
    )

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(
        np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten(), truckcolor
    )
    plt.plot(
        np.array(fr_wheel[0, :]).flatten(),
        np.array(fr_wheel[1, :]).flatten(),
        truckcolor,
    )
    plt.plot(
        np.array(rr_wheel[0, :]).flatten(),
        np.array(rr_wheel[1, :]).flatten(),
        truckcolor,
    )
    plt.plot(
        np.array(fl_wheel[0, :]).flatten(),
        np.array(fl_wheel[1, :]).flatten(),
        truckcolor,
    )
    plt.plot(
        np.array(rl_wheel[0, :]).flatten(),
        np.array(rl_wheel[1, :]).flatten(),
        truckcolor,
    )
    plt.plot(x, y, "*")


def getDistance(p1, p2):
    """
    Calculate distance
    :param p1: list, point1
    :param p2: list, point2
    :return: float, distance
    """
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)


class Vehicle:
    def __init__(self, x, y, yaw, vel=0):
        """
        Define a vehicle class
        :param x: float, x position
        :param y: float, y position
        :param yaw: float, vehicle heading
        :param vel: float, velocity
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vel = vel

    def update(self, acc, delta):
        """
        Vehicle motion model, here we are using simple bycicle model
        :param acc: float, acceleration
        :param delta: float, heading control
        """
        self.x += self.vel * math.cos(self.yaw) * dt
        self.y += self.vel * math.sin(self.yaw) * dt
        self.yaw += self.vel * math.tan(delta) / WB * dt
        self.vel += acc * dt


class Trajectory:
    def __init__(self, traj_x, traj_y):
        """
        Define a trajectory class
        :param traj_x: list, list of x position
        :param traj_y: list, list of y position
        """
        self.traj_x = traj_x
        self.traj_y = traj_y
        self.last_idx = 0

    def getPoint(self, idx):
        return [self.traj_x[idx], self.traj_y[idx]]

    def getTargetPoint(self, pos):
        """
        Get the next look ahead point
        :param pos: list, vehicle position
        :return: list, target point
        """
        target_idx = self.last_idx
        target_point = self.getPoint(target_idx)
        curr_dist = getDistance(pos, target_point)

        while curr_dist < L and target_idx < len(self.traj_x) - 1:
            target_idx += 1
            target_point = self.getPoint(target_idx)
            curr_dist = getDistance(pos, target_point)

        self.last_idx = target_idx
        return self.getPoint(target_idx)


class PI:
    def __init__(self, kp=1.0, ki=0.1):
        """
        Define a PID controller class
        :param kp: float, kp coeff
        :param ki: float, ki coeff
        :param kd: float, kd coeff
        """
        self.kp = kp
        self.ki = ki
        self.Pterm = 0.0
        self.Iterm = 0.0
        self.last_error = 0.0

    def control(self, error):
        """
        PID main function, given an input, this function will output a control unit
        :param error: float, error term
        :return: float, output control
        """
        self.Pterm = self.kp * error
        self.Iterm += error * dt

        self.last_error = error
        output = self.Pterm + self.ki * self.Iterm
        return output


def main():
    # create vehicle
    ego = Vehicle(0, 0, 0)
    plotVehicle(ego.x, ego.y, ego.yaw)

    # target velocity
    target_vel = 10

    # target course
    traj = [(73, 19), (73, 20), (73, 21), (73, 22), (73, 23), (73, 24), (73, 25), (73, 26), (73, 27), (73, 28), (73, 29), (73, 30), (73, 31), (73, 32), (73, 33), (73, 34), (73, 35), (73, 36), (73, 37), (73, 38), (73, 39), (72, 40), (71, 41), (70, 42), (69, 43), (68, 44), (67, 45), (66, 46), (65, 47), (64, 48), (63, 49), (62, 50), (61, 51), (60, 52), (60, 53), (60, 54), (59, 55), (58, 56), (57, 57), (57, 58), (57, 59), (56, 60), (56, 61), (55, 62), (54, 63), (54, 64), (54, 65), (53, 66), (52, 67), (52, 68), (51, 69), (50, 70), (50, 71), (50, 72), (49, 73), (49, 74), (49, 75), (48, 76), (47, 77), (47, 78), (47, 79), (47, 80), (47, 81), (46, 82), (46, 83), (46, 84), (45, 85), (45, 86), (45, 87), (45, 88), (44, 89), (44, 90), (44, 91), (44, 92), (43, 93), (43, 94), (42, 95), (42, 96), (42, 97), (42, 98), (42, 99), (42, 100), (42, 101), (42, 102), (42, 103), (42, 104), (42, 105), (42, 106), (42, 107), (42, 108), (42, 109), (42, 110), (42, 111), (42, 112), (42, 113), (42, 114), (42, 115), (42, 116), (42, 117), (42, 118), (42, 119), (42, 120), (42, 121), (42, 122), (42, 123), (42, 124), (42, 125), (42, 126), (42, 127), (42, 128), (42, 129), (42, 130), (42, 131), (42, 132), (42, 133), (42, 134), (42, 135), (42, 136), (42, 137), (42, 138), (42, 139), (42, 140), (42, 141), (42, 142), (42, 143), (42, 144), (42, 145), (42, 146), (42, 147), (42, 148), (42, 149), (42, 150), (42, 151), (42, 152), (42, 153), (42, 154), (42, 155), (42, 156), (42, 157), (42, 158), (42, 159), (42, 160), (42, 161), (42, 162), (42, 163), (42, 164), (42, 165), (42, 166), (42, 167), (43, 168), (43, 169), (44, 170), (45, 171), (45, 172), (45, 173), (45, 174), (45, 175), (45, 176), (45, 177), (45, 178), (45, 179), (45, 180), (45, 181), (45, 182), (45, 183), (45, 184), (45, 185), (45, 186), (45, 187), (45, 188), (45, 189), (45, 190), (45, 191), (45, 192), (45, 193), (45, 194), (45, 195), (45, 196), (45, 197), (45, 198), (45, 199), (45, 200), (44, 201), (43, 202), (42, 203), (41, 204), (40, 205), (39, 206), (38, 207), (37, 208), (36, 209), (35, 210), (34, 211), (33, 212), (32, 213), (31, 214), (30, 215), (29, 216)]
    # traj_x = np.arange(0, 100, 0.5)
    # traj_y = [math.sin(x / 10.0) * x / 2.0 for x in traj_x]
    traj_x = []
    traj_y = []
    for pos in traj:
        traj_x.append(pos[0])
        traj_y.append(pos[1])
    traj_x = np.array(traj_x)
    traj_y = np.array(traj_y)
    traj = Trajectory(traj_x, traj_y)
    goal = traj.getPoint(len(traj_x) - 1)

    # create PI controller
    PI_acc = PI()
    PI_yaw = PI()

    # real trajectory
    traj_ego_x = []
    traj_ego_y = []

    plt.figure(figsize=(12, 8))
    while getDistance([ego.x, ego.y], goal) > 1:
        target_point = traj.getTargetPoint([ego.x, ego.y])

        # use PID to control the vehicle
        vel_err = target_vel - ego.vel
        acc = PI_acc.control(vel_err)

        yaw_err = math.atan2(target_point[1] - ego.y, target_point[0] - ego.x) - ego.yaw
        delta = PI_yaw.control(yaw_err)

        # move the vehicle
        ego.update(acc, delta)

        # store the trajectory
        traj_ego_x.append(ego.x)
        traj_ego_y.append(ego.y)

        # plots
        plt.cla()
        plt.plot(traj_x, traj_y, "-r", linewidth=5, label="course")
        plt.plot(traj_ego_x, traj_ego_y, "-b", linewidth=2, label="trajectory")
        plt.plot(target_point[0], target_point[1], "og", ms=5, label="target point")
        plotVehicle(ego.x, ego.y, ego.yaw, delta)
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.legend()
        plt.grid(True)
        plt.pause(0.1)


if __name__ == "__main__":
    main()