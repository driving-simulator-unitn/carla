'''
This file reads data from the dummy_send.py file, and applies it to the current
vehicle in the simulation. It is expected to be run on a the same machine as the
as the dummy_send.py file, and apply the changes to all the different carla
servers.
'''

#1# IMPORTS

# Standard imports
import os
import sys
import glob
import time
import argparse
import math

# Add carla to the path
try:
    sys.path.append(
        glob.glob(
            os.path.dirname(
                os.path.dirname(os.path.abspath(__file__))
            ) + (f'/carla/dist/carla-*{sys.version_info.major:d}.'
                 f'{sys.version_info.minor:d}-'
                 f'{"win-amd64" if os.name == "nt" else "linux-x86_64":s}.egg')
        )[0]
    )
except IndexError:
    pass

# Third-party imports
import carla
import pygame
import zmq
import numpy as np

#1# CLASSES

# Carla client instance
class CarlaClient:
    '''
    CarlaClient carla client instance.
    '''

    # Constructor
    def __init__(self, host, port, fps, camera_transform, pull_endpoint=False):
        '''
        __init__ constructor.

        :param host: carla client host
        :type host: str
        :param port: carla client port
        :type port: int
        :param fps: FPS of the simluation
        :type fps: int
        :param camera_transform: The transform of the camera
        :type camera_transform: carla.Transform
        :param pull_endpoint: The endpoint for the pull socket, default False
        :type pull_endpoint: str
        '''

        # Initialise carla
        self.client = carla.Client(host, port)
        self.client.set_timeout(120)

        # Get the world
        self.world = self.client.get_world()

        # Reset the traffic lights
        self.world.reset_all_traffic_lights()

        # Set the FPS
        self.fps = fps
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / self.fps
        settings.substepping = True
        settings.max_substep_delta_time = 1.0 / self.fps
        self.world.apply_settings(settings)

        # Spawn a vehicle
        blueprint_library = self.world.get_blueprint_library()
        bp                = blueprint_library.filter('model3')[0]
        spawn_point       = self.world.get_map().get_spawn_points()[2]
        self.vehicle      = self.world.spawn_actor(bp, spawn_point)

        # Create a dummy actor to attach the camera to
        dummy_bp = blueprint_library.find('sensor.other.collision')
        self.dummy      = self.world.spawn_actor(dummy_bp, camera_transform, attach_to=self.vehicle)

        # Get the spectator
        self.spectator        = self.world.get_spectator()
        self.spectator_origin = self.spectator.get_transform()
        self.transform        = camera_transform

        if pull_endpoint:
            # Initialise ZMQ
            self.context     = zmq.Context()
            self.pull_socket = self.context.socket(zmq.PULL)
            self.pull_socket.setsockopt(zmq.CONFLATE, 1)
            self.pull_socket.connect(pull_endpoint)
        else:
            # Remove physics from the vehicle
            self.vehicle.set_simulate_physics(False)

    # Destructor
    def __del__(self):
        '''
        __del__ destructor.
        '''

        # Destroy actors
        self.vehicle.destroy()

        # Reset the spectator
        self.spectator.set_transform(self.spectator_origin)

        # Update the world
        self.tick()

        # Destroy ZMQ sockets if the parameter is set
        if hasattr(self, 'context'):
            self.context.destroy()

    # Get the controls
    def get_controls(self):
        '''
        get_controls get the controls from the ZMQ socket.
        '''

        # Read data, controls are [throttle, brake, steer]
        try:
            data     = self.pull_socket.recv(zmq.DONTWAIT).decode('utf-8')
            controls = [float(x) for x in data.split(',')]
        except zmq.error.Again:
            controls = [0.0, 0.0, 0.0]

        # Return the controls
        return controls

    # Tick the world
    def tick(self):
        '''
        tick ticks the world.
        '''

        self.world.tick()

        return self.world.get_snapshot()

    # Follow the vehicle with the spectator
    def follow(self, vehicle_transform):
        '''
        follow follow the vehicle with the spectator.
        '''

        # # Get the vehicle yaw
        # vehicle_yaw = np.deg2rad(vehicle_transform.rotation.yaw)

        # # Project the spectator transform onto the vehicle transform
        # location_x = self.transform.location.x*np.cos(vehicle_yaw) - self.transform.location.y*np.sin(vehicle_yaw)
        # location_y = self.transform.location.x*np.sin(vehicle_yaw) + self.transform.location.y*np.cos(vehicle_yaw)

        # # Compute the spectator transform
        # spectator_location = carla.Location(
        #     vehicle_transform.location.x + location_x,
        #     vehicle_transform.location.y + location_y,
        #     vehicle_transform.location.z + self.transform.location.z
        # )
        # spectator_rotation = carla.Rotation(
        #     vehicle_transform.rotation.pitch + self.transform.rotation.pitch,
        #     vehicle_transform.rotation.yaw   + self.transform.rotation.yaw,
        #     vehicle_transform.rotation.roll  + self.transform.rotation.roll
        # )

        # # Set the spectator to follow the vehicle
        # self.spectator.set_transform(
        #     carla.Transform(
        #         spectator_location,
        #         spectator_rotation
        #     )
        # )

        # Use the dummy actor to follow the vehicle
        transform = self.dummy.get_transform()

        # Get the vehicle velocity, angular velocity, and acceleration
        vehicle_velocity         = self.vehicle.get_velocity()
        vehicle_angular_velocity = self.vehicle.get_angular_velocity()
        vehicle_acceleration     = self.vehicle.get_acceleration()

        # Print them
        # print(f'vehicle_velocity:         {vehicle_velocity}')
        # print(f'vehicle_angular_velocity: {vehicle_angular_velocity}')
        # print(f'vehicle_acceleration:     {vehicle_acceleration}')

        # Project forward using the velocity, angular velocity, acceleration, and FPS
        transform.location.x     += vehicle_velocity.x / self.fps + 0.5*vehicle_acceleration.x / self.fps**2
        transform.location.y     += vehicle_velocity.y / self.fps + 0.5*vehicle_acceleration.y / self.fps**2
        transform.location.z     += vehicle_velocity.z / self.fps + 0.5*vehicle_acceleration.z / self.fps**2
        transform.rotation.yaw   += vehicle_angular_velocity.z / self.fps
        transform.rotation.pitch += vehicle_angular_velocity.y / self.fps
        transform.rotation.roll  += vehicle_angular_velocity.x / self.fps

        self.transform = transform

        # Set the transform
        self.spectator.set_transform(transform)

    def test(self, controls, vehicle_transform):
        '''
        test computes the position of the vehicle by converting the controls to
        a position and orientation.
        '''

        # Compute the position
        position = carla.Location(
            vehicle_transform.location.x + 0.5*controls[0]*np.cos(np.deg2rad(vehicle_transform.rotation.yaw)),
            vehicle_transform.location.y + 0.5*controls[0]*np.sin(np.deg2rad(vehicle_transform.rotation.yaw)),
            vehicle_transform.location.z
        )

        # Compute the orientation
        orientation = carla.Rotation(
            vehicle_transform.rotation.pitch,
            vehicle_transform.rotation.yaw + 0.5*controls[2],
            vehicle_transform.rotation.roll
        )

        # Return the transform
        return carla.Transform(position, orientation)

    # Advance
    def advance(self, transform=None):
        '''
        advance advance the simulation by applying the controls to the vehicle.

        :param transform: The transform to send to the secondary servers,
           default None
        :type transform: carla.Transform
        '''

        # # If we are the primary server, get and apply the controls
        # if transform is None:
        #     # Get the vehicle transform
        #     vehicle_transform = self.vehicle.get_transform()

        #     # Get the controls
        #     controls = self.get_controls()

        #     # Apply controls
        #     # vehicle_transform = self.test(controls, vehicle_transform)
        #     self.vehicle.apply_control(
        #         carla.VehicleControl(
        #             throttle = controls[0],
        #             brake    = controls[1],
        #             steer    = controls[2]
        #         )
        #     )
        # else:
        #     vehicle_transform = transform

        # # Set the transform
        # # self.vehicle.set_transform(vehicle_transform)

        # # Follow the vehicle
        # self.follow(vehicle_transform)

        # # Return the transform to use for the secondary servers
        # return vehicle_transform

        vehicle_transform = self.dummy.get_transform()
        err_x = self.transform.location.x - vehicle_transform.location.x
        err_y = self.transform.location.y - vehicle_transform.location.y
        err_z = self.transform.location.z - vehicle_transform.location.z
        err_yaw = self.transform.rotation.yaw - vehicle_transform.rotation.yaw
        err_pitch = self.transform.rotation.pitch - vehicle_transform.rotation.pitch
        err_roll = self.transform.rotation.roll - vehicle_transform.rotation.roll
        err_loc = np.sqrt(err_x**2 + err_y**2 + err_z**2)
        err_rot = np.sqrt(err_yaw**2 + err_pitch**2 + err_roll**2)
        # print(f'err_loc: {err_loc:.2e}')
        # print(f'err_rot: {err_rot:.2e}')

        # Get the controls
        controls = self.get_controls()

        # Apply controls
        # vehicle_transform = self.test(controls, vehicle_transform)
        self.vehicle.apply_control(
            carla.VehicleControl(
                throttle = controls[0],
                brake    = controls[1],
                steer    = controls[2]
            )
        )

        self.follow(vehicle_transform)

#1# FUNCTIONS

# Get the camera transform for the spectator (left, center, right)
def get_camera_transform(camera_position):
    '''
    get_camera_transform get the camera transform for the spectator ((l)eft,
    (c)entre, (r)ight).

    :param camera_position: The position of the camera ((l)eft, (c)entre,
       (r)ight)
    :type camera_position: str
    :raises ValueError: If the camera position is invalid
    :return: The camera transform for the spectator
    :rtype: carla.Transform
    '''

    # Camera base location is always the same
    location = carla.Location(x=0.0, y=-0.4, z=1.15)

    # Apply the three rotations around the z-, y-, and x-axis respectively
    if camera_position == 'l':
        # The left camera has:
        # - -1.057744 radians around the z-axis (yaw)
        # -  0.019855 radians around the y-axis (pitch)
        # -  0.104814 radians around the x-axis (roll)
        rotation = carla.Rotation(
            pitch = np.rad2deg(0.019855),
            yaw   = np.rad2deg(-1.057744),
            roll  = np.rad2deg(0.104814)
        )
    elif camera_position == 'c':
        # The centre camera has:
        # -  0.000000 radians around the z-axis (yaw)
        # - -0.041460 radians around the y-axis (pitch)
        # -  0.000000 radians around the x-axis (roll)
        rotation = carla.Rotation(
            pitch = np.rad2deg(-0.041460),
            yaw   = np.rad2deg(0.000000),
            roll  = np.rad2deg(0.000000)
        )
    elif camera_position == 'r':
        # The right camera has:
        # -  1.057744 radians around the z-axis (yaw)
        # -  0.019855 radians around the y-axis (pitch)
        # - -0.104814 radians around the x-axis (roll)
        rotation = carla.Rotation(
            pitch = np.rad2deg(0.019855),
            yaw   = np.rad2deg(1.057744),
            roll  = np.rad2deg(-0.104814)
        )
    else:
        raise ValueError(
            'Invalid camera position, must be (l)eft, (c)entre, or (r)ight.'
        )


    # Return the transform
    return carla.Transform(location, rotation)

# Main loop
def main_loop(clients):
    '''
    main_loop main loop of the program.

    :param clients: The clients to advance
    :type clients: list
    '''

    # Divide the clients
    main_client  = clients[0]
    side_clients = clients[1:]

    print('Reading from simplatform, CTRL-C to exit.')
    iteration        = 0
    num_side_clients = len(side_clients)
    while True:
        try:
            # Start timer
            start = time.perf_counter()

            # Tick the simulation of the main server first
            snapshot = main_client.tick()

            # Get the transform of the vehicle from the snapshot
            vehicle_transform = snapshot.find(main_client.vehicle.id).get_transform()

            # Tick the other servers, in a fair manner
            for i in range(iteration, num_side_clients + iteration):
                side_clients[i % num_side_clients].tick()

            # Advance the simulation of the main server first
            transform = main_client.advance(vehicle_transform)

            # Use the result to advance the other servers, in a fair manner
            for i in range(iteration, num_side_clients + iteration):
                side_clients[i % num_side_clients].advance(transform)

            iteration += 1

            # Stop timer
            end = time.perf_counter()

            # Sleep for the remainder of the frame to reach the target fps
            time.sleep(max(1.0 / main_client.fps - (end - start), 0))

            # Print the FPS
            print(f'FPS: {math.floor(1.0 / (time.perf_counter() - start)):4d}', end='\r')

        except KeyboardInterrupt:
            return

# Main function
def main():
    '''
    main main function of the program.
    '''

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--fps',
        type=int,
        default=60,
        help='Frames per second of the simulation (default: 60)'
    )
    parser.add_argument(
        '-chps', '--carla-host-port-screen',
        type=str,
        default='127.0.0.1:2000:c',
        help=('The carla client host, port, and screen '
              '(default: 127.0.0.1:2000:c) (l)eft, (c)entre, (r)ight. '
              'Multiple can be specified by separating them with a comma.')
    )
    parser.add_argument(
        '-pe', '--pull_endpoint',
        type=str,
        default='tcp://127.0.0.1:5555',
        help='The endpoint to pull data from (default: tcp://127.0.0.1:5555)'
    )
    parser.add_argument(
        '--res',
        type=str,
        default='1920x1080',
        help='The resolution of the camera (default: 1920x1080)'
    )
    parser.add_argument(
        '--fov',
        type=float,
        default=90.0,
        help='The field of view of the camera (default: 90.0)'
    )
    args = parser.parse_args()

    # Split the carla host, port, and screen
    servers = args.carla_host_port_screen.split(',')

    # Initialise the client
    clients = []
    for server in servers:
        host, port, screen = server.split(':')

        if 'c' == screen:
            client = CarlaClient(
                host,
                int(port),
                args.fps,
                get_camera_transform(screen),
                args.pull_endpoint
            )
        else:
            client = CarlaClient(host, int(port), args.fps, get_camera_transform(screen))

        clients.append(client)

    # Main loop
    main_loop(clients)

#1# MAIN

if __name__ == '__main__':
    main()
