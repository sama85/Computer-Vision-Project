"""
Module for rover state actuation.

"""


import numpy as np
import math 
from perception import pix_to_rover, to_polar_coords



def FollowingLeftWall(Decider, Rover):
    """Create a class to represent FollowingLeftWall state."""
    THROTTLE_SET = 0.8
    LARGE_WALL_OFFSET = -9

    # Add negative bias to nav angles left of rover to follow wall and not hit wall
    wall_heading = np.mean(Rover.nav_angles_left) + LARGE_WALL_OFFSET
    
    # If drive below max velocity, accelerate
    if Rover.vel < Rover.MAX_VEL:
        Rover.throttle = THROTTLE_SET
    else:
        Rover.throttle = 0
    
    # Steer to keep at heading that follows wall
    Rover.brake = 0
    #clip angle to be in steering range
    Rover.steer = np.clip(wall_heading,
                              Rover.MAX_STEER_RIGHT, Rover.MAX_STEER_LEFT)

    
     
                              


def TurningToLeftWall(Decider, Rover):
    """Create a class to represent TurningToLeftWall state."""

        # Stopping before turning
    if Rover.vel > Rover.MIN_VEL:
        Rover.throttle = 0
        Rover.brake = Rover.MAX_BRAKE
        Rover.steer = 0
    # Turn left towards wall
    elif Rover.vel <= Rover.MIN_VEL:
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = Rover.MAX_STEER_LEFT
    
    

def AvoidingLeftWall(Decider, Rover):
    """Create a class to represent AvoidingLeftWall state."""

    
        # Stopping before turning
    if Rover.vel > Rover.MIN_VEL:
        Rover.throttle = 0
        Rover.brake = Rover.MAX_BRAKE
        Rover.steer = 0
        # Turn right to avoid left wall
    elif Rover.vel <= Rover.MIN_VEL:
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = Rover.MAX_STEER_RIGHT
    
    # """Handle switching from AvoidingLeftWall state."""
    # if pointed_along_wall(Rover=Rover):
    #     Decider.switch_to_state(Rover, Decider.state[0])  # FollowingLeftWall
    # else:
    #     Decider.switch_to_state(Rover, Decider.curr_state)



def AvoidingObstacles(Decider, Rover):
    """Create a class to represent AvoidingObstacles state."""


    nav_heading = np.mean(Rover.nav_angles)
        # Stopping before avoiding obstacles
    if Rover.vel > Rover.MIN_VEL:
        Rover.throttle = 0
        Rover.brake = Rover.MAX_BRAKE
        Rover.steer = 0
        # Turn left or right depending on where nav terrain is
    elif Rover.vel <= Rover.MIN_VEL:
        Rover.throttle = 0
        Rover.brake = 0
            # Turn right if nav terrain is more than 15 deg to the right
        if nav_heading < Rover.MIN_ANGLE_RIGHT:
            Rover.steer = Rover.MAX_STEER_RIGHT
            # Turn left if nav terrain is more than 15 deg to the left
        elif nav_heading > Rover.MIN_ANGLE_LEFT:
            Rover.steer = Rover.MAX_STEER_LEFT
            # Back up e.g. if nav_angles are NaN
        else:
            Rover.steer = Rover.MAX_STEER_RIGHT

    # """Handle switching from AvoidingObstacles state."""
    # stucktime = 2.0
    # if completed_mission(Rover=Rover):
    #     Decider.switch_to_state(Rover, Decider.state[7])  # ReturningHome
    # elif pointed_along_wall(Rover=Rover):
    #     Decider.switch_to_state(Rover, Decider.state[0])  # FollowingLeftWall
    # else:
    #     Decider.switch_to_state(Rover, Decider.curr_state)



def GoingToSample(Decider, Rover):
    """Create a class to represent GoingToSample state."""

    THROTTLE_SET = 0.39
    APPROACH_VEL = 1.0
    SMALL_WALL_OFFSET = -3.6

    rock_pixs = len(Rover.rock_angles)
        # Stopping before going to sample
    if Rover.vel > APPROACH_VEL:
        Rover.throttle = 0
        Rover.brake = Rover.MAX_BRAKE
        Rover.steer = 0
        # Drive to sample
    elif(Rover.vel <= APPROACH_VEL):
            # If sample in view
        if rock_pixs >= 1:
                # Add a right bias to heading so as not to bump in left wall
            rock_heading = np.mean(Rover.rock_angles) + SMALL_WALL_OFFSET
                #rock_heading = np.mean(Rover.rock_angles)
                # Yaw left if rock sample to left more than 23 deg
            if rock_heading >= Rover.MIN_ANGLE_LEFT:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = Rover.MAX_STEER_LEFT
                # Yaw right if rock sample to right more than -23 deg
            elif rock_heading <= Rover.MIN_ANGLE_RIGHT:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = Rover.MAX_STEER_RIGHT
                # Otherwise drive at average rock sample heading
            elif (Rover.MIN_ANGLE_RIGHT < rock_heading < Rover.MIN_ANGLE_LEFT) or math.isnan(rock_heading):
                Rover.brake = 0
                Rover.throttle = THROTTLE_SET
                Rover.steer = np.clip(rock_heading,
                                          Rover.MAX_STEER_RIGHT,
                                          Rover.MAX_STEER_LEFT)
        else:  # rock not in view
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = Rover.MAX_STEER_LEFT

    # """Handle switching from GoingToSample state."""
    # # Time in seconds allowed to remain stuck in this state
    # stucktime = 4.0
    # if Rover.near_sample:
    #     Rover.timer_on = False
    #     Decider.switch_to_state(Rover, Decider.state[5])  # Stopping

    # elif Decider.is_stuck(Rover=Rover):
    #     Rover.timer_on = False
    #     Decider.switch_to_state(Rover, Decider.state[6])  # GettingUnstuck
    # else:
    #     Decider.switch_to_state(Rover, Decider.curr_state)



def GettingUnstuck(Decider, Rover):

    # THROTTLE_SET = 1.0
    # OBS_OFFSET_YAW = 35
   

    # # Yaw value measured from either
    # # right or left of the obstacle
    # obs_offset_yaw = np.absolute(Rover.yaw - Rover.stuck_heading)
    # if obs_offset_yaw < OBS_OFFSET_YAW:
    #     Rover.throttle = 0
    #     Rover.brake = 0
    #     Rover.steer = Rover.MAX_STEER_RIGHT
    #     # ..at witch point drive straight
    # else:
    #     Rover.brake = 0
    #     Rover.steer = 0
    #     Rover.throttle = THROTTLE_SET

    nav_heading = np.mean(Rover.nav_angles)
        # Stopping before avoiding obstacles
    if Rover.vel > Rover.MIN_VEL:
            Rover.throttle = 0
            Rover.brake = Rover.MAX_BRAKE
            Rover.steer = 0
        # Turn left or right depending on where nav terrain is
    elif Rover.vel <= Rover.MIN_VEL:
            Rover.throttle = 0
            Rover.brake = 0
            # Turn right if nav terrain is more than 20 deg to the right
            if nav_heading < Rover.MAX_STEER_RIGHT:
                Rover.steer = Rover.MAX_STEER_RIGHT
            # Turn left if nav terrain is more than 20 deg to the left
            elif nav_heading > Rover.MAX_STEER_LEFT:
                Rover.steer = Rover.MAX_STEER_LEFT
            # Back up e.g. if nav_angles are NaN
            else:
                Rover.steer = Rover.MAX_STEER_RIGHT
    
    # stucktime = 2.3
    # if Rover.vel >= 1.0:
    #     if Rover.going_home:
    #         Decider.switch_to_state(Rover, Decider.state[7])  # ReturningHome
    #     else:
    #         Decider.switch_to_state(Rover, Decider.state[0])  # FollowingLeftWall

    # elif Decider.is_stuck(Rover=Rover):
    #     Rover.timer_on = False
    #     if Rover.going_home:
    #         Decider.switch_to_state(Rover, Decider.state[7])  # ReturningHome
    #     else:
    #         Decider.switch_to_state(Rover, Decider.state[0])  # FollowingLeftWall
    # else:
    #     Decider.switch_to_state(Rover, Decider.curr_state)




def ReturningHome(Decider, Rover):
    """Create a class to represent ReturningHome state."""
    
    SLOW_VEL = 1.0
    PARK_VEL = 0.5
    MAX_THROTTLE_SET = 0.8
    SLOW_THROTTLE_SET = 0.2
    PARK_THROTTLE_SET = 0.3

    home_pixpts_rf = pix_to_rover(Rover.home_coords_world,
                                        Rover.pos, Rover.yaw)
    xpix_pts, ypix_pts = home_pixpts_rf
    home_distances, home_headings = to_polar_coords(xpix_pts, ypix_pts)
        # Update Rover home polar coordinates
    Rover.home_distance = np.mean(home_distances)
    Rover.home_heading = np.mean(home_headings)

        # Drive at a weighted average of home and nav headings with a 3:7 ratio
    nav_heading = np.mean(Rover.nav_angles)
    homenav_heading = 0.3*Rover.home_heading + (1 - 0.3)*nav_heading

        # Keep within max velocity
    if Rover.vel < Rover.MAX_VEL:
            Rover.throttle = MAX_THROTTLE_SET
    else:
            Rover.throttle = 0

        # Approach at pure nav heading
    if Rover.home_distance > 450:
            Rover.brake = 0
            Rover.steer = np.clip(nav_heading,
                                  Rover.MAX_STEER_RIGHT, Rover.MAX_STEER_LEFT)
        # Approach at the weighted average home and nav headings
    elif 200 < Rover.home_distance <= 450:
            Rover.brake = 0
            Rover.steer = np.clip(homenav_heading,
                                  Rover.MAX_STEER_RIGHT, Rover.MAX_STEER_LEFT)
        # Slow down while keeping current heading
    elif 100 < Rover.home_distance <= 200:
            if Rover.vel < SLOW_VEL:
                Rover.throttle = SLOW_THROTTLE_SET
            else:
                Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = np.clip(homenav_heading,
                                  Rover.MAX_STEER_RIGHT, Rover.MAX_STEER_LEFT)
        # Precisely approach at pure home heading and slow down for parking
    elif Rover.home_distance <= 100:
            if Rover.vel > PARK_VEL:
                Rover.throttle = 0
                Rover.brake = Rover.MAX_BRAKE
                Rover.steer = 0
            elif Rover.vel <= PARK_VEL:
                Rover.brake = 0
                # yaw left if home to the left more than 23 deg
                if Rover.home_heading >= 23:
                    Rover.throttle = 0
                    Rover.steer = Rover.MAX_STEER_LEFT
                # yaw right if home to the right more than 23 deg
                elif Rover.home_heading <= -23:
                    Rover.throttle = 0
                    Rover.steer = Rover.MAX_STEER_RIGHT
                # otherwise tread slowly at pure home heading
                elif -23 < Rover.home_heading < 23:
                    Rover.throttle = PARK_THROTTLE_SET
                    Rover.steer = np.clip(Rover.home_heading,
                                          Rover.MAX_STEER_RIGHT,
                                          Rover.MAX_STEER_LEFT)
    
    # """Handle switching from ReturningHome state."""
    # # Time in seconds allowed to remain stuck in this state
    # stucktime = 2.5
    # if obstacle_at_front(Rover=Rover):
    #     Rover.timer_on = False
    #     Decider.switch_to_state(Rover, Decider.state[3])  # AvoidingObstacles

    # elif reached_home(Rover=Rover):        
    #     Rover.timer_on = False
    #     Decider.switch_to_state(Rover, Decider.state[8])  # ParkingAtHome

    # elif Decider.is_stuck(Rover=Rover):
    #     Rover.timer_on = False
    #     Decider.switch_to_state(Rover, Decider.state[6])  # GettingUnstuck
    # else:
    #     Decider.switch_to_state(Rover, Decider.curr_state)



def Stopping(Decider, Rover):
    """Create a class to represent Stopping state."""

   
    Rover.throttle = 0
    Rover.brake = Rover.MAX_BRAKE
    Rover.steer = 0
    # """Handle switching from Stopping state."""
    # Rover.send_pickup = True
    # if Rover.picking_up == 0:
    #     Decider.switch_to_state(Rover, Decider.state[2])  # AvoidingLeftWall
    # else:
    #     Decider.switch_to_state(Rover, Decider.curr_state)

    

def ParkingAtHome(Decider, Rover):
    """Create a class to represent ParkingAtHome state."""
    
        # Brake if still moving
    if Rover.vel > Rover.MIN_VEL:
            Rover.throttle = 0
            Rover.brake = Rover.MAX_BRAKE
            Rover.steer = 0
    # """Handle switching from ParkingAtHome state."""
    # Decider.switch_to_state(Rover, Decider.state[8])  # Remain in ParkingAtHome