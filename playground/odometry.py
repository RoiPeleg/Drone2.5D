import numpy as np

from playground.environment.body import Body

np.random.seed(42)
import random 
random.seed(42)

class Odometry(Body):
    def __init__(self, filename, mu, sigma, delta_t=0.1):
        super().__init__(filename)
        self.__mu = mu
        self.__sigma = sigma
        self.__angle = 0
        self.__gyro = 0
        self.__delta_t = delta_t

    def track_rotate(self, angle):
        noise = np.random.normal(self.__mu, self.__sigma)
        self.rotate(angle + noise)
        self.__gyro = (angle + noise) * self.__delta_t
        self.__angle += angle + noise

    def track_move(self, dist_x, dist_y):
        noise = np.random.normal(self.__mu, self.__sigma)
        self.move(dist_x + noise, dist_y + noise)

    def track_altitude(self, alt):
        noise = np.random.normal(self.__mu, self.__sigma/100)
        self.set_altitude(alt + noise)

    @property
    def gyro(self):
        return self.__gyro 