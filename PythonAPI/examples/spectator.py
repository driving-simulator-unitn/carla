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

def game_loop(spectator, vehicle, world):
    pygame.init()
    pygame.font.init()
    display = pygame.display.set_mode((100, 100))
    pygame.display.set_caption('Spectator')
    clock = pygame.time.Clock()

    # Attach a clock to the world
    world_clock = pygame.time.Clock()
    world.on_tick(lambda timestamp: world_clock.tick())

    # Add a listener to the camera that displays the image on the screen
    # def camera_callback(image):
    #     array = image.raw_data
    #     array = np.frombuffer(array, dtype=np.uint8)
    #     array = array.reshape((image.height, image.width, 4))
    #     array = array[:, :, :3]
    #     surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    #     display.blit(surface, (0, 0))

    # camera.listen(camera_callback)

    while True:
        clock.tick_busy_loop(120)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return

        # Move the vehicle
        pressed_keys = pygame.key.get_pressed()
        if pressed_keys[pygame.K_UP]:
            vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
        elif pressed_keys[pygame.K_DOWN]:
            vehicle.apply_control(carla.VehicleControl(brake=1.0, steer=0.0))
        elif pressed_keys[pygame.K_LEFT]:
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=-1.0))
        elif pressed_keys[pygame.K_RIGHT]:
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=1.0))
        else:
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0))

        # Move the spectator to follow the vehicle from behind
        transform = vehicle.get_transform()
        location = transform.transform(carla.Location(x=-4.5, z=2.4))
        transform.location = location
        rotation = transform.transform_vector(carla.Rotation(pitch=-15).get_forward_vector())
        # transform.rotation = carla.Rotation(rotation.y, rotation.z, rotation.x)
        spectator.set_transform(transform)

        # Print the server fps on the screen
        fps = world_clock.get_fps()
        # font = pygame.font.SysFont('Arial', 30)
        # text = font.render(f'FPS: {fps:.2f}', True, (255, 255, 255))
        # display.blit(text, (0, 0))
        # print(fps)

        pygame.display.flip()

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

    # Attach a camera to the vehicle
    camera_blueprint = blueprint_library.find('sensor.camera.rgb')
    camera_blueprint.set_attribute('image_size_x', '1920')
    camera_blueprint.set_attribute('image_size_y', '1080')
    camera_blueprint.set_attribute('fov', '90')
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle)

    # Create a spectator
    spectator = world.get_spectator()

    # Move the spectator to follow the vehicle from behind
    # spectator_location = carla.Location(vehicle.get_location() + carla.Location(z=1))
    # spectator_rotation = carla.Rotation(vehicle.get_transform().rotation.pitch - 15, vehicle.get_transform().rotation.yaw, vehicle.get_transform().rotation.roll)
    # spectator.set_transform(carla.Transform(spectator_location, spectator_rotation))

    # Game loop
    game_loop(spectator, vehicle, world)

    # Destroy the vehicle
    vehicle.destroy()

if __name__ == '__main__':
    main()