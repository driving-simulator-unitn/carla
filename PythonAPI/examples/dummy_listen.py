'''

This file reads data from the dummy_send.py file, and applies it to the current
vehicle in the simulation. It is expected to be run on a different machine than
the one running the dummy_send.py file, and it is expected to be run on the same
machine as the carla server.
'''

#1# IMPORTS

# Standard imports
import os
import sys
import glob
import time
import argparse

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
import zmq

#1# FUNCTIONS

# Main loop of the program
def main_loop(socket, fps, vehicle, spectator):
    '''
    main_loop main loop of the program.

    :param socket: ZMQ socket to broadcast data to
    :type socket: zmq.Socket
    :param fps: Frames per second of the simulation
    :type fps: int
    :param vehicle: The vehicle to apply the data to
    :type vehicle: carla.Vehicle
    :param spectator: The spectator to follow the vehicle
    :type spectator: carla.Spectator
    '''

    print('Reading from dummy_send, CTRL-C to exit.')
    while True:
        try:
            # Read data
            try:
                increment = float(socket.recv(zmq.DONTWAIT).decode('utf-8'))
            except zmq.error.Again:
                increment = 0

            # Get current vehicle transform
            transform = vehicle.get_transform()

            # Apply increment to the x-axis of the position of the vehicle
            transform.location.x += increment

            # Apply transform to the vehicle
            vehicle.set_transform(transform)

            # Apply transform to the spectator
            location = transform.transform(carla.Location(x=-4.5, z=2.4))
            transform.location = location
            spectator.set_transform(transform)

            # Since the receive is non-blocking and fast, we can sleep for the
            # desired amount of time to achieve the desired fps
            time.sleep(1 / fps)
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
        default=120,
        help='Frames per second of the simulation'
    )
    parser.add_argument(
        '-ch', '--carla-host',
        type=str,
        default='127.0.0.1',
        help='The host of the carla server'
    )
    parser.add_argument(
        '-cp', '--carla-port',
        type=int,
        default=2000,
        help='The port of the carla server'
    )
    parser.add_argument(
        '-e', '--endpoint',
        type=str,
        default='tcp://127.0.0.1:5555',
        help='The endpoint to pull data from'
    )
    args = parser.parse_args()

    # Initialise ZMQ
    context = zmq.Context()

    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.connect(args.endpoint)

    # Initialise carla
    client = carla.Client(args.carla_host, args.carla_port)
    client.set_timeout(2.0)

    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    bp                = blueprint_library.filter('model3')[0]
    spawn_point       = world.get_map().get_spawn_points()[0]
    vehicle           = world.spawn_actor(bp, spawn_point)

    spectator = world.get_spectator()

    # Main loop
    main_loop(socket, args.fps, vehicle, spectator)

    # Destroy actors
    vehicle.destroy()

    # Close ZMQ
    context.destroy()

#1# MAIN

if __name__ == '__main__':
    main()
