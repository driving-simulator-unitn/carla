import glob
import os
import sys

try:
    sys.path.append(glob.glob(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import time
import pygame
import random
import numpy as np

def game_loop(spectator, vehicle, spectator2, vehicle2):

    while True:
        # Move the vehicle
        transform = vehicle.get_transform()
        transform.location.x += 0.1

        vehicle.set_transform(transform)
        vehicle2.set_transform(vehicle.get_transform())

        # Move the spectator to follow the vehicle from behind
        transform = vehicle.get_transform()
        location = transform.transform(carla.Location(x=-4.5, z=2.4))
        transform.location = location

        spectator.set_transform(transform)
        spectator2.set_transform(transform)

        time.sleep(1/120)

def main():
    # Connect to Carla
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    # Get the world
    world = client.get_world()

    # Get a random blueprint
    blueprint_library = world.get_blueprint_library()
    vehicle_blueprint = blueprint_library.filter('model3')[0]

    # Spawn the vehicle in a random place
    transform = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(vehicle_blueprint, transform)

    # Create a spectator
    spectator = world.get_spectator()

    # Connect to Carla
    client2 = carla.Client('localhost', 3000)
    client2.set_timeout(2.0)

    # Get the world
    world2 = client2.get_world()

    # Get a random blueprint
    blueprint_library = world2.get_blueprint_library()
    vehicle_blueprint = blueprint_library.filter('model3')[0]

    # Spawn the vehicle in a random place
    vehicle2 = world2.spawn_actor(vehicle_blueprint, transform)

    # Create a spectator
    spectator2 = world2.get_spectator()

    # Game loop
    game_loop(spectator, vehicle, spectator2, vehicle2)

    # Destroy the vehicle
    vehicle.destroy()
    vehicle2.destroy()

if __name__ == '__main__':
    main()