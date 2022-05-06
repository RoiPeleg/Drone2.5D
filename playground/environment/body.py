import numpy as np
from playground.utils.transform import create_rotation_matrix_yx, make_direction


class Body:
    """
        Physical environment simulation element
    """

    def __init__(self, filename):

        self.filename = filename
        
        self.__pos = np.zeros(2)
        self.start_drone_positions = {"assets/p15.png": np.array([120.0, 610.0]),
                                        "assets/p11.png": np.array([250.0, -520.0]),
                                        "assets/map.png": np.array([160.0, 0.0]),
                                         "assets/p14.png": np.array([0.0, 100.0])
                                        }
        self.__pos = self.start_drone_positions[filename]

        self.__altitude = 0
        self.__rotation_matrix = np.identity(3)

    def rotate(self, angle):
        tr_mat = create_rotation_matrix_yx(angle)
        self.__rotation_matrix = np.matmul(self.__rotation_matrix, tr_mat)

    # def move(self, dist):
    #     self.__pos = self.try_move(dist)
    
    # def try_move(self, dist):
    #     direction = make_direction(self.__rotation_matrix)
    #     new_pos = self.__pos.copy()
    #     new_pos += direction * dist
    #     return new_pos
    
    def move(self, dist_x, dist_y):
        self.__pos = self.try_move(dist_x, dist_y)

    def try_move(self, dist_x, dist_y):
        # x axis
        direction = make_direction(self.__rotation_matrix)
        pos = self.__pos.copy()
        pos += direction * dist_x

        # y axis
        direction = make_direction(np.matmul(self.__rotation_matrix, create_rotation_matrix_yx(90)))
        pos += (-1) * direction * dist_y
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

