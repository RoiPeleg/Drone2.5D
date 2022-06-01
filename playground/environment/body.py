import numpy as np
from playground.utils.transform import create_rotation_matrix_yx, make_direction
np.random.seed(42)


class Body:
    """
        Physical environment simulation element
    """

    def __init__(self):        
        self.__pos = np.zeros(2)
        self.__altitude = 0
        self.__rotation_matrix = np.identity(3)

    def rotate(self, angle):
        tr_mat = create_rotation_matrix_yx(angle)
        self.__rotation_matrix = np.matmul(self.__rotation_matrix, tr_mat)

    def move(self, dist_x, dist_y):
        self.__pos = self.try_move(dist_x, dist_y)

    def try_move(self, dist_x, dist_y):
        direction = make_direction(self.__rotation_matrix)
        pos = self.__pos.copy()
        pos += direction * dist_x
        direction = make_direction(np.matmul(self.__rotation_matrix, create_rotation_matrix_yx(-90)))
        pos += direction * dist_y
        return pos

    def set_altitude(self, alt):
        self.__altitude = alt

    @property
    def size(self):
        return 1

    @property
    def altitude(self):
        return self.__altitude

    @property
    def position(self):
        return self.__pos

    @property
    def rotation(self):
        return self.__rotation_matrix

