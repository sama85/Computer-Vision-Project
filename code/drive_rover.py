# Do the necessary imports
import argparse
import shutil
import base64
from datetime import datetime
import os
import cv2
import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from PIL import Image
from flask import Flask
from io import BytesIO, StringIO
import json
import pickle
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import time

debug_mode = False

# Import functions for perception and decision making
from perception import perception_step
from decision import decision_step
from supporting_functions import update_rover, create_output_images

# Initialize socketio server and Flask application
# (learn more at: https://python-socketio.readthedocs.io/en/latest/)
sio = socketio.Server()
app = Flask(__name__)

# Read in ground truth map and create 3-channel green version for overplotting
# NOTE: images are read in by default with the origin (0, 0) in the upper left
# and y-axis increasing downward.
ground_truth = mpimg.imread('../calibration_images/map_bw.png')
# This next line creates arrays of zeros in the red and blue channels
# and puts the map into the green channel.  This is why the underlying
# map output looks green in the display image
ground_truth_3d = np.dstack((ground_truth * 0, ground_truth * 255, ground_truth * 0)).astype(float)


# Define RoverState() class to retain rover state parameters
class RoverState():
    def __init__(self):
        self.start_time = None  # To record the start time of navigation
        self.total_time = None  # To record total duration of naviagation
        self.sample_seen = False  # If a sample is detected, change to True
        self.img = np.zeros((160, 320, 3), dtype=float)  # Current camera image
        self.pos = None  # Current position (x, y)
        self.yaw = None  # Current yaw angle
        self.pitch = None  # Current pitch angle
        self.roll = None  # Current roll angle
        self.vel = None  # Current velocity
        self.steer = 0  # Current steering angle
        self.throttle = 0  # Current throttle value
        self.brake = 0  # Current brake value
        self.nav_angles = None  # Angles of navigable terrain pixels
        self.nav_dists = None  # Distances of navigable terrain pixels
        self.ground_truth = ground_truth_3d  # Ground truth worldmap
        self.mode = 'forward'  # Current mode (can be forward or stop)
        self.throttle_set = 0.2  # Throttle setting when accelerating
        self.brake_set = 10  # Brake setting when braking
        # The stop_forward and go_forward fields below represent total count
        # of navigable terrain pixels.  This is a very crude form of knowing
        # when you can keep going and when you should stop.  Feel free to
        # get creative in adding new fields or modifying these!
        self.stop_forward = 50  # Threshold to initiate stopping
        self.go_forward = 500  # Threshold to go forward again
        self.max_vel = 2  # Maximum velocity (meters/second)
        # Image output from perception step
        # Update this image to display your intermediate analysis steps
        # on screen in autonomous mode
        self.vision_image = np.zeros((160, 320, 3), dtype=float)
        self.vision_warped = np.zeros((160, 320, 3), dtype=float)
        self.vision_threshed = np.zeros((160, 320), dtype=float)
        # Update this image with the positions of navigable terrain
        # obstacles and rock samples
        self.worldmap = np.zeros((200, 200, 3), dtype=float)
        self.samples_pos = None  # To store the actual sample positions
        self.samples_to_find = 0  # To store the initial count of samples
        self.samples_located = 0  # To store number of samples located on map
        self.samples_collected = 0  # To count the number of samples collected
        self.nessar_sample = 0  # Will be set to telemetry value data["near_sample"]
        self.picking_up = 0  # Will be set to telemetry value data["picking_up"]
        self.send_pickup = False  # Set to True to trigger rock pickup
        
    #if the rover's absolute pitch or roll is greater than 0.75
    #then the rover is not stable
    #mapping the world while the rover is not stable can cause poor fidelity
    def is_Stable(self) -> bool:
        if (self.pitch < 0.75 or self.pitch > 359.25)\
            and (self.roll < 0.75 or self.roll > 359.25):
            return True
        return False


# Initialize our rover
Rover = RoverState()

# Variables to track frames per second (FPS)
# Intitialize frame counter
frame_counter = 0
# Initalize second counter
second_counter = time.time()
fps = None

if debug_mode:
    # fig = plt.figure()
    plt.ion()
    plt.gcf().canvas.manager.set_window_title('Debugging Window')
    ax1 = plt.subplot(221)
    ax1.set_title('Rover View')
    im1 = ax1.imshow(Rover.img)
    ax2 = plt.subplot(222)
    ax2.set_title('Bird Eye View')
    im2 = ax2.imshow(Rover.vision_warped)
    ax3 = plt.subplot(223)
    ax3.set_title('Thresholded image')
    im3 = ax3.imshow(Rover.vision_threshed, cmap='gray')
    

# Define telemetry function for what to do with incoming data
@sio.on('telemetry')
def telemetry(sid, data):
    global frame_counter, second_counter, fps
    frame_counter += 1
    # Do a rough calculation of frames per second (FPS)
    if (time.time() - second_counter) > 1:
        fps = frame_counter
        frame_counter = 0
        second_counter = time.time()
    print("Current FPS: {}".format(fps))

    if data:
        global Rover
        # Initialize / update Rover with current telemetry
        Rover, image = update_rover(Rover, data)

        if np.isfinite(Rover.vel):

            # Execute the perception and decision steps to update the Rover's state
            Rover = perception_step(Rover)
            Rover = decision_step(Rover)

            if debug_mode:
                im1.set_data(Rover.img)
                im2.set_data(Rover.vision_warped.astype(np.uint8))
                im3.set_data(Rover.vision_threshed)
                im3.autoscale()                
                # plt.draw()
                plt.gcf().canvas.draw_idle()
                plt.gcf().canvas.start_event_loop(0.01)
                # plt.pause(0.1)


            # Create output images to send to server
            out_image_string1, out_image_string2 = create_output_images(Rover)

            # The action step!  Send commands to the rover!

            # Don't send both of these, they both trigger the simulator
            # to send back new telemetry so we must only send one
            # back in respose to the current telemetry data.

            # If in a state where want to pickup a rock send pickup command
            if Rover.send_pickup and not Rover.picking_up:
                send_pickup()
                # Reset Rover flags
                Rover.send_pickup = False
            else:
                # Send commands to the rover!
                commands = (Rover.throttle, Rover.brake, Rover.steer)
                send_control(commands, out_image_string1, out_image_string2)

        # In case of invalid telemetry, send null commands
        else:

            # Send zeros for throttle, brake and steer and empty images
            send_control((0, 0, 0), '', '')

        # If you want to save camera images from autonomous driving specify a path
        # Example: $ python drive_rover.py image_folder_path
        # Conditional to save image frame if folder was specified
        if args.image_folder != '':
            timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
            image_filename = os.path.join(args.image_folder, timestamp)
            image.save('{}.jpg'.format(image_filename))

    else:
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control((0, 0, 0), '', '')
    sample_data = {}
    sio.emit(
        "get_samples",
        sample_data,
        skip_sid=True)


def send_control(commands, image_string1, image_string2):
    # Define commands to be sent to the rover
    data = {
        'throttle': commands[0].__str__(),
        'brake': commands[1].__str__(),
        'steering_angle': commands[2].__str__(),
        'inset_image1': image_string1,
        'inset_image2': image_string2,
    }
    # Send commands via socketIO server
    sio.emit(
        "data",
        data,
        skip_sid=True)
    eventlet.sleep(0)


# Define a function to send the "pickup" command
def send_pickup():
    print("Picking up")
    pickup = {}
    sio.emit(
        "pickup",
        pickup,
        skip_sid=True)
    eventlet.sleep(0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument(
        'image_folder',
        type=str,
        nargs='?',
        default='',
        help='Path to image folder. This is where the images from the run will be saved.'
    )
    args = parser.parse_args()

    # os.system('rm -rf IMG_stream/*')
    if args.image_folder != '':
        print("Creating image folder at {}".format(args.image_folder))
        if not os.path.exists(args.image_folder):
            os.makedirs(args.image_folder)
        else:
            shutil.rmtree(args.image_folder)
            os.makedirs(args.image_folder)
        print("Recording this run ...")
    else:
        print("NOT recording this run ...")

    # wrap Flask application with socketio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)