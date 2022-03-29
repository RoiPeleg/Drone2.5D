from sqlite3 import Timestamp
import pygame
import argparse
from enum import Enum
import threading

import playground.slam.frontend
# import playground.slam.backend
# import playground.slam.gtsambackend
from playground.rawsensorsview import RawSensorsView
from playground.robot import Robot
from playground.odometry import Odometry
from playground.Lidar import Lidar
from playground.environment.world import World

import time

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
    pygame.display.set_caption('SLAM playground')
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', help='Environmental map filename')
    args = parser.parse_args()

    # Create simulation objects

    world = World(args.filename, max_z= 3)
    odometry = Odometry(mu=0, sigma=1)  # noised measurements
    sensor = Lidar(dist_range=100, fov=90, mu=0, sigma=1)  # noised measurements
    robot = Robot(odometry, sensor)
    sensors_view = RawSensorsView(world.height, world.width, world.max_z)
    slam_front_end = playground.slam.frontend.FrontEnd(world.height, world.width)
    # gtsam_slam_back_end = playground.slam.gtsambackend.GTSAMBackEnd(edge_sigma=0.5, angle_sigma=0.1)
    # slam_back_end = playground.slam.backend.BackEnd(edge_sigma=0.5, angle_sigma=0.1)

    clock = Clock(maximum_time_to_live = 8*60.0, current_time_to_live = 8*60.0)
    
    # Initialize rendering
    screen = pygame.display.set_mode([world.width * 2, world.height])
    font = pygame.font.Font(pygame.font.get_default_font(), 24)
    sensors_text_surface = font.render('Sensors', True, (255, 0, 0))
    icp_text_surface = font.render('ICP', True, (255, 0, 0))
    
    # Robot movement configuration
    rotation_step = 10  # degrees
    moving_step = 10  # points

    # make first initialization
    def clock_fun():
        while True:
            # lidar sensor:
            sensors_view.take_measurements(odometry, sensor)
            slam_front_end.add_key_frame(sensor)
            robot.sensor.scan(robot.position, robot.rotation, world)
            
            # battery sensor:
            clock.decay()
            
            # optical flow sensor:
            sensors_view.take_measurements_optical(odometry)

            # barometer sensor:
            sensors_view.take_measurements_barometer(odometry)

            # gyro sensor:
            time.sleep(0.1)

    t_clock = threading.Thread(target=clock_fun, args=())

    robot.move(0, world)
    sensors_view.take_measurements(odometry, sensor)
    slam_front_end.add_key_frame(sensor)
    
    # start simulation loop
    simulation_mode = SimulationMode.RAW_SENSORS
    running = True
    rd = False
    t_clock.start()
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                robot.stop_rd()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    simulation_mode = SimulationMode.RAW_SENSORS
                if event.key == pygame.K_i:
                    simulation_mode = SimulationMode.ICP_ADJUSTMENT
                if event.key == pygame.K_s:
                    # we assume that we detect a loop so can try to optimize pose graph
                    # loop_frame = slam_front_end.create_loop_closure(sensor)
                    # slam_back_end.update_frames(slam_front_end.get_frames(), loop_frame)
                    break
                if event.key == pygame.K_g:
                    # we assume that we detect a loop so can try to optimize pose graph
                    # loop_frame = slam_front_end.create_loop_closure(sensor)
                    # gtsam_slam_back_end.update_frames(slam_front_end.get_frames(), loop_frame)
                    break
                if event.key == pygame.K_KP_PLUS:
                    moving_step += 1
                    robot.c_speed(1)
                if event.key == pygame.K_KP_MINUS:
                    moving_step -= 1
                    robot.c_speed(-1)
                if event.key == pygame.K_p:
                    if not rd:
                        robot.init_rd(moving_step,world)
                    else:
                        robot.stop_rd()
                if event.key == pygame.K_LEFT:
                    robot.rotate(rotation_step, world)
                if event.key == pygame.K_RIGHT:
                    robot.rotate(-rotation_step, world)
                if event.key == pygame.K_UP:
                    robot.move(moving_step, world)
                if event.key == pygame.K_DOWN:
                    robot.move(-moving_step, world)

        world.draw(screen)
        robot.draw(screen, world.height, world.width)
        
        if simulation_mode == SimulationMode.RAW_SENSORS:
            sensors_view.draw(screen, offset=world.width)
            screen.blit(sensors_text_surface, dest=(15, 15))
        if simulation_mode == SimulationMode.ICP_ADJUSTMENT:
            slam_front_end.draw(screen, offset=world.width)
            screen.blit(icp_text_surface, dest=(30, 15))
        
        opticalflow_text_surface = font.render(f'optical: {sensors_view.opticalflow}', True, (255, 0, 0))
        screen.blit(opticalflow_text_surface, dest=(500, 570))
        
        barometer_text_surface = font.render(f'z: {round(sensors_view.drone_height, 2)}', True, (255, 0, 0))
        screen.blit(barometer_text_surface, dest=(400, 570))
        
        battery_text_surface = font.render(f'Battery: {round(clock.current_time_to_live/clock.maximum_time_to_live*100, 2)}%', True, (255, 0, 0))
        screen.blit(battery_text_surface, dest=(550, 15))
        
        gyro_text_surface = font.render(f'gyro: {round(robot.odomentry.gyro, 2)}', True, (255, 0, 0))
        screen.blit(gyro_text_surface, dest=(700, 570))
        
        pygame.display.flip()

    t_clock.join()
    pygame.quit()

if __name__ == '__main__':
    main()
