from sqlite3 import Timestamp
import pygame
import argparse
from enum import Enum
import threading

import warnings
warnings.filterwarnings(action='ignore', message='Mean of empty slice')
warnings.filterwarnings(action='ignore', message='invalid value encountered in double_scalars')

import playground.slam.frontend
from playground.rawsensorsview import RawSensorsView
from playground.robot import Robot
from playground.odometry import Odometry
from playground.Lidar import Lidar
from playground.environment.world import World
from playground.DroneController import DroneController
from playground.Algorithms import Algorithms

import time
import numpy as np
np.random.seed(42)

class SimulationMode(Enum):
    RAW_SENSORS = 1,
    ICP_ADJUSTMENT = 2        

class Clock:
        def __init__(self, maximum_time_to_live, current_time_to_live):
            self.__maximum_time_to_live = maximum_time_to_live
            self.__current_time_to_live = current_time_to_live
        
        def decay(self):
            self.__current_time_to_live -= 0.1

        @property
        def current_time_to_live(self):
            return self.__current_time_to_live
        
        @property
        def maximum_time_to_live(self):
            return self.__maximum_time_to_live

def main():
    pygame.init()
    pygame.display.set_caption('BAT drone simulation')

    parser = argparse.ArgumentParser()
    parser.add_argument('filename', help='Environmental map filename', nargs='?', const=1, default='assets/p11.png')
    args = parser.parse_args()

    # Create simulation objects
    delta_t = 0.1
    world = World(args.filename, max_z= 3)
    odometry = Odometry(args.filename, mu=0, sigma=0.02, delta_t=delta_t)  # noised measurements
    sensor = Lidar(dist_range=120, fov=90, mu=0, sigma=0.02)  # noised measurements
    robot = Robot(odometry, sensor, world, args.filename)
    sensors_view = RawSensorsView(world.height, world.width, world.max_z)
    slam_front_end = playground.slam.frontend.FrontEnd(world.height, world.width)
    controller = DroneController(robot, sensors_view, delta_t=delta_t)
    algo = Algorithms(controller, mode="bat")

    clock = Clock(maximum_time_to_live = 8*60.0, current_time_to_live = 8*60.0)
    # clock = Clock(maximum_time_to_live = 1*60.0, current_time_to_live = 1*60.0)
    
    # Initialize rendering
    screen = pygame.display.set_mode([world.width, world.height])
    font = pygame.font.Font(pygame.font.get_default_font(), 18)
    
    # Robot movement configuration
    rotation_step = 10  # degrees
    moving_step = 10  # points

    # make first initialization
    def clock_fun():
        while threading.main_thread().isAlive():
            
            # lidar sensor:
            sensor.scan(robot.position, robot.rotation, world)
            sensors_view.take_measurements(odometry, sensor)
            slam_front_end.add_key_frame(sensor)
            robot.sensor.scan(robot.position, robot.rotation, world)
            
            # battery sensor:
            clock.decay()
            sensors_view.take_measurements_battery(round(clock.current_time_to_live/clock.maximum_time_to_live*100, 2))
            
            # optical flow sensor:
            sensors_view.take_measurements_optical(odometry)

            # barometer sensor:
            sensors_view.take_measurements_barometer(odometry)

            # gyro sensor:
            sensors_view.take_measurements_gyro(odometry)



            controller.move()
            algo.sample_data()

            time.sleep(0.1)

            if clock.current_time_to_live <= 0:
                break

    t_clock = threading.Thread(target=clock_fun, args=())
    
    # start simulation loop
    running = True
    t_clock.start()
    algo.run()
    robot.rotate(180)
    while running:
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_LEFT:
                        robot.rotate(rotation_step)
                    if event.key == pygame.K_RIGHT:
                        robot.rotate(-rotation_step)
                    if event.key == pygame.K_UP:
                        robot.move(moving_step)
                    if event.key == pygame.K_DOWN:
                        robot.move(-moving_step)

            world.draw(screen, sensors_view.map)
            robot.draw(screen, world.height, world.width)
            algo.draw(screen, world.height, world.width)
            # sensors_view.draw(screen, offset=world.width)
            
            data_sensors = controller.sensors_data()
            
            text_surface = font.render(f'Battery: {data_sensors["battery"]}%', True, (255, 0, 0))
            screen.blit(text_surface, dest=(550, 15))
            
            text_surface = font.render(f'State: {algo.state}', True, (255, 0, 0))
            screen.blit(text_surface, dest=(300, 15))
            
            text_surface = font.render(f'pitch: {data_sensors["pitch"]}', True, (255, 0, 0))
            screen.blit(text_surface, dest=(570, 570))

            text_surface = font.render(f'roll: {data_sensors["roll"]}', True, (255, 0, 0))
            screen.blit(text_surface, dest=(480, 570))

            text_surface = font.render(f'yaw: {data_sensors["yaw"]}', True, (255, 0, 0))
            screen.blit(text_surface, dest=(570, 600))

            text_surface = font.render(f'left: {data_sensors["d_left"]}', True, (255, 0, 0))
            screen.blit(text_surface, dest=(700, 570))

            text_surface = font.render(f'right: {data_sensors["d_right"]}', True, (255, 0, 0))
            screen.blit(text_surface, dest=(700, 600))

            text_surface = font.render(f'front: {data_sensors["d_front"]}', True, (255, 0, 0))
            screen.blit(text_surface, dest=(815, 570))

            text_surface = font.render(f'back: {data_sensors["d_back"]}', True, (255, 0, 0))
            screen.blit(text_surface, dest=(815, 600))

            text_surface = font.render(f'up: {data_sensors["d_up"]}', True, (255, 0, 0))
            screen.blit(text_surface, dest=(930, 570))

            text_surface = font.render(f'down: {data_sensors["d_down"]}', True, (255, 0, 0))
            screen.blit(text_surface, dest=(930, 600))
        

            text_surface = font.render(f'optical: [{data_sensors["v_x"]}, {data_sensors["v_y"]}]', True, (255, 0, 0))
            screen.blit(text_surface, dest=(1100, 575))
            
            text_surface = font.render(f'gyro: {data_sensors["gyro"]}', True, (255, 0, 0))
            screen.blit(text_surface, dest=(1100, 600))
            
            
            pygame.display.flip()

        except KeyboardInterrupt:
            # User interrupt the program with ctrl+c
            running = False
    
    controller.stop()
    algo.stop()
    pygame.quit()
    t_clock.join(0.1)
    exit()

if __name__ == '__main__':
    main()
