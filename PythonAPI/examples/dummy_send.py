'''

This file sends dummy data to three different instances of carla servers, in an
attempt to achieve enough perfomance to run the simulation in real time on our
three projectors.

It simply broadcasts the same data to three different endpoints, which are
expected to be the three different instances of carla servers. The data is
broadcasted using the ZeroMQ library. The data is simply an increment in the
x-axis of the position of the vehicle, happening only if the W key is pressed.
'''

#1# IMPORTS

# Standard imports
import time
import argparse

# Third-party imports
import zmq
import pygame

#1# FUNCTIONS

# Main loop of the program
def main_loop(socket, fps):
    '''
    main_loop main loop of the program.

    :param socket: ZMQ socket to broadcast data to
    :type socket: zmq.Socket
    :param fps: Frames per second of the simulation
    :type fps: int
    '''

    print('Press W to move the vehicle forward, ESC to exit.')
    while True:
        # Read keyboard input
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return

        # Get pressed keys
        pressed = pygame.key.get_pressed()

        # If W is pressed, increment the x-axis of the position of the vehicle
        if pressed[pygame.K_w]:
            throttle = 1
        else:
            throttle = 0

        if pressed[pygame.K_s]:
            brake = 1
        else:
            brake = 0

        if pressed[pygame.K_a]:
            steer = -1
        elif pressed[pygame.K_d]:
            steer = 1
        else:
            steer = 0

        msg = f'{throttle},{brake},{steer}'

        # Send data
        try:
            socket.send(bytes(msg, 'utf-8'), zmq.DONTWAIT)
        except zmq.error.Again:
            pass

        # Since the send is non-blocking and fast, we can sleep for the desired
        # amount of time to achieve the desired fps
        time.sleep(1 / fps)

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
        '-e', '--endpoint',
        type=str,
        default='tcp://*:5555',
        help='The endpoint to broadcast data to'
    )
    args = parser.parse_args()

    # Initialise ZMQ
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.bind(args.endpoint)

    # Initialise pygame to read keyboard input without a window
    pygame.init()
    pygame.display.set_mode((100, 100))

    # Main loop
    main_loop(socket, args.fps)

    # Close ZMQ
    context.destroy()

#1# MAIN

if __name__ == '__main__':
    main()
