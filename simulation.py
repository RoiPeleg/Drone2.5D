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

def main():
    pygame.init()
    pygame.display.set_caption('SLAM playground')
    parser = argparse.ArgumentParser()
    parser.add_argument('filename', help='Environmental map filename')
    args = parser.parse_args()

    # Create simulation objects

    world = World(args.filename)
    odometry = Odometry(mu=0, sigma=3)  # noised measurements
    sensor = Lidar(dist_range=100, fov=90, mu=0, sigma=1)  # noised measurements
    robot = Robot(odometry, sensor)
    sensors_view = RawSensorsView(world.height, world.width)
    slam_front_end = playground.slam.frontend.FrontEnd(world.height, world.width)
    # gtsam_slam_back_end = playground.slam.gtsambackend.GTSAMBackEnd(edge_sigma=0.5, angle_sigma=0.1)
    # slam_back_end = playground.slam.backend.BackEnd(edge_sigma=0.5, angle_sigma=0.1)

    maximum_time_to_live = 8*60.0 #seconds
    current_time_to_live = maximum_time_to_live

    # Initialize rendering
    screen = pygame.display.set_mode([world.width * 2, world.height])
    font = pygame.font.Font(pygame.font.get_default_font(), 24)
    sensors_text_surface = font.render('Sensors', True, (255, 0, 0))
    icp_text_surface = font.render('ICP', True, (255, 0, 0))
    battery_text_surface = font.render(f'Battery: {current_time_to_live/maximum_time_to_live*100}%', True, (255, 0, 0))
    opticalflow_text_surface = font.render(f'optical: {sensors_view.opticalflow}', True, (255, 0, 0))
    # Robot movement configuration
    rotation_step = 10  # degrees
    moving_step = 10  # points

    # make first initialization
    def clock():
        while True:
            # lidar sensor:
            sensors_view.take_measurements(odometry, sensor)
            slam_front_end.add_key_frame(sensor)
            robot.sensor.scan(robot.position, robot.rotation, world)
            
            # battery sensor:
            current_time_to_live = current_time_to_live - 0.1
            
            # optical flow sensor:
            sensors_view.take_measurements_optical(odometry)

            # gyro sensor:
            time.sleep(0.1)

    t_clock = threading.Thread(target=clock, args=())

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
        screen.blit(opticalflow_text_surface, dest=(550, 500))
        battery_text_surface = font.render(f'Battery: {current_time_to_live/maximum_time_to_live*100}%', True, (255, 0, 0))
        screen.blit(battery_text_surface, dest=(550, 15))
        gyro_text_surface = font.render(f'gyro: {robot.odomentry.gyro}', True, (255, 0, 0))
        screen.blit(gyro_text_surface, dest=(700, 570))
        pygame.display.flip()

    t_clock.join()
    pygame.quit()

if __name__ == '__main__':
    main()
