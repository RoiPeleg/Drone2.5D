import numpy as np

from playground.environment.body import Body


class Odometry(Body):
    def __init__(self, mu, sigma):
        super().__init__()
        self.__mu = mu
        self.__sigma = sigma
        self.__optical = 10
        self.__gyro = 0

    def track_rotate(self, angle):
        noise = np.random.normal(self.__mu, self.__sigma)
        self.rotate(angle + noise)
        self.__gyro += angle + noise

    def track_move(self, dist):
        noise = np.random.normal(self.__mu, self.__sigma)
        self.move(dist + noise)

    @property
    def optical(self):
        return self.__optical

    @property
    def gyro(self):
        return self.__gyro