import numpy as np

from playground.environment.body import Body


class Odometry(Body):
    def __init__(self, filename, mu, sigma):
        super().__init__(filename)
        self.__mu = mu
        self.__sigma = sigma
        self.__angle = 0
        self.__gyro = 0

    def track_rotate(self, angle):
        noise = np.random.normal(self.__mu, self.__sigma)
        self.rotate(angle + noise)
        self.__gyro = abs(self.__angle - (angle + noise))
        self.__angle += angle + noise

    def track_move(self, dist):
        noise = np.random.normal(self.__mu, self.__sigma)
        self.move(dist + noise)

    def track_altitude(self, alt):
        noise = np.random.normal(self.__mu, self.__sigma/100)
        self.set_altitude(alt + noise)

    @property
    def gyro(self):
        #noise = np.random.normal(self.__mu, self.__sigma)
        return self.__gyro 