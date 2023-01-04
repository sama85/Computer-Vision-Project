
import numpy as np
import math 
from perception import pix_to_rover, to_polar_coords



def FollowingLeftWall(Rover):

    THROTTLE_SET = 0.8
    LARGE_WALL_OFFSET = -9

    # Add negative bias to nav angles left of rover to fnot hit wall
    angle_to_wall = np.mean(Rover.nav_angles_left) + LARGE_WALL_OFFSET
    
    if Rover.vel < Rover.MAX_VEL:
        Rover.throttle = THROTTLE_SET
    else:
        Rover.throttle = 0
    
    Rover.brake = 0

    #clip angle to be in steering range
    Rover.steer = np.clip(angle_to_wall,
                              Rover.MAX_STEER_RIGHT, Rover.MAX_STEER_LEFT)



def TurningToLeftWall( Rover):

    # Stop before turning left
    if Rover.vel > Rover.MIN_VEL:
        Rover.throttle = 0
        Rover.brake = Rover.MAX_BRAKE
        Rover.steer = 0
    
    # Turn left towards wall
    elif Rover.vel <= Rover.MIN_VEL:
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = Rover.MAX_STEER_LEFT
    

def AvoidingLeftWall(Rover):

    # Stop before turning right
    if Rover.vel > Rover.MIN_VEL:
        Rover.throttle = 0
        Rover.brake = Rover.MAX_BRAKE
        Rover.steer = 0
    
    # Turn right to avoid left wall
    elif Rover.vel <= Rover.MIN_VEL:
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = Rover.MAX_STEER_RIGHT
    


def AvoidingObstacles(Rover):

    nav_angle = np.mean(Rover.nav_angles)
    
    # Stop before avoiding obstacles
    if Rover.vel > Rover.MIN_VEL:
        Rover.throttle = 0
        Rover.brake = Rover.MAX_BRAKE
        Rover.steer = 0

    # check where navigable terrain is and turn left or right
    elif Rover.vel <= Rover.MIN_VEL:
        Rover.throttle = 0
        Rover.brake = 0
        
        # Turn right if nav terrain is more than 15 deg to the right
        if nav_angle < Rover.MIN_ANGLE_RIGHT:
            Rover.steer = Rover.MAX_STEER_RIGHT
        
        # Turn left if nav terrain is more than 15 deg to the left
        elif nav_angle > Rover.MIN_ANGLE_LEFT:
            Rover.steer = Rover.MAX_STEER_LEFT
        
        # if nav angles are undefined, back up
        else:
            Rover.steer = Rover.MAX_STEER_RIGHT


def GoingToSample(Rover):

    THROTTLE_SET = 0.39
    APPROACH_VEL = 1.0
    SMALL_WALL_OFFSET = -3.6

    rock_pixs = len(Rover.rock_angles)
    
    # Stop before going to sample
    if Rover.vel > APPROACH_VEL:
        Rover.throttle = 0
        Rover.brake = Rover.MAX_BRAKE
        Rover.steer = 0
    
    # Drive to sample
    elif(Rover.vel <= APPROACH_VEL):
            
        # If sample in view
        if rock_pixs >= 1:
            
            # Add a right bias to angle so as not to bump in left wall
            angle_to_rock = np.mean(Rover.rock_angles) + SMALL_WALL_OFFSET
            
            # Yaw left if rock sample to left more than 23 deg
            if angle_to_rock >= Rover.MIN_ANGLE_LEFT:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = Rover.MAX_STEER_LEFT
            
            # Yaw right if rock sample to right more than -23 deg
            elif angle_to_rock <= Rover.MIN_ANGLE_RIGHT:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = Rover.MAX_STEER_RIGHT
            
            # Otherwise drive at average rock sample angle
            elif (Rover.MIN_ANGLE_RIGHT < angle_to_rock < Rover.MIN_ANGLE_LEFT) or math.isnan(angle_to_rock):
                Rover.brake = 0
                Rover.throttle = THROTTLE_SET
                Rover.steer = np.clip(angle_to_rock,
                                          Rover.MAX_STEER_RIGHT,
                                          Rover.MAX_STEER_LEFT)
        # rock not in view
        else:  
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = Rover.MAX_STEER_LEFT



def GettingUnstuck(Rover):

    nav_angle = np.mean(Rover.nav_angles)
    
    # Stopping before avoiding obstacles
    if Rover.vel > Rover.MIN_VEL:
            Rover.throttle = 0
            Rover.brake = Rover.MAX_BRAKE
            Rover.steer = 0
    
    # check where navigable terrain is and turn left or right
    elif Rover.vel <= Rover.MIN_VEL:
            Rover.throttle = 0
            Rover.brake = 0

            # Turn right if nav terrain is more than 20 deg to the right
            if nav_angle < Rover.MAX_STEER_RIGHT:
                Rover.steer = Rover.MAX_STEER_RIGHT
            
            # Turn left if nav terrain is more than 20 deg to the left
            elif nav_angle > Rover.MAX_STEER_LEFT:
                Rover.steer = Rover.MAX_STEER_LEFT
            # if nav angles are undefined, back up
            else:
                Rover.steer = Rover.MAX_STEER_RIGHT

def StoppingAtSample(Rover):
   
    Rover.throttle = 0
    Rover.brake = Rover.MAX_BRAKE
    Rover.steer = 0


def ReturningHome(Rover):

    SLOW_VEL = 1.0
    PARK_VEL = 0.5
    MAX_THROTTLE_SET = 0.8
    SLOW_THROTTLE_SET = 0.2
    PARK_THROTTLE_SET = 0.3

    home_pixpts_rf = pix_to_rover(Rover.home_coords_world,
                                        Rover.pos, Rover.yaw)
    xpix_pts, ypix_pts = home_pixpts_rf
    distances_from_home, angles_from_home = to_polar_coords(xpix_pts, ypix_pts)
    
    # Update Rover home polar coordinates
    Rover.distance_from_home = np.mean(distances_from_home)
    Rover.angle_from_home = np.mean(angles_from_home)

    # Drive at a weighted average of home and nav headings with a 3:7 ratio
    nav_angle = np.mean(Rover.nav_angles)
    homenav_heading = 0.3*Rover.angle_from_home + (1 - 0.3)*nav_angle

    # Keep within max velocity
    if Rover.vel < Rover.MAX_VEL:
            Rover.throttle = MAX_THROTTLE_SET
    else:
            Rover.throttle = 0

    # Approach at pure nav angle
    if Rover.distance_from_home > 450:
            Rover.brake = 0
            Rover.steer = np.clip(nav_angle,
                                  Rover.MAX_STEER_RIGHT, Rover.MAX_STEER_LEFT)
    
    # Approach at the weighted average home and nav headings
    elif 200 < Rover.distance_from_home <= 450:
            Rover.brake = 0
            Rover.steer = np.clip(homenav_heading,
                                  Rover.MAX_STEER_RIGHT, Rover.MAX_STEER_LEFT)
    
    # Slow down while keeping current angle
    elif 100 < Rover.distance_from_home <= 200:
            if Rover.vel < SLOW_VEL:
                Rover.throttle = SLOW_THROTTLE_SET
            else:
                Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = np.clip(homenav_heading,
                                  Rover.MAX_STEER_RIGHT, Rover.MAX_STEER_LEFT)
    
    # Precisely approach at pure home angle and slow down for parking
    elif Rover.distance_from_home <= 100:
            if Rover.vel > PARK_VEL:
                Rover.throttle = 0
                Rover.brake = Rover.MAX_BRAKE
                Rover.steer = 0
            elif Rover.vel <= PARK_VEL:
                Rover.brake = 0
                # yaw left if home to the left more than 23 deg
                if Rover.angle_from_home >= 23:
                    Rover.throttle = 0
                    Rover.steer = Rover.MAX_STEER_LEFT
                # yaw right if home to the right more than 23 deg
                elif Rover.angle_from_home <= -23:
                    Rover.throttle = 0
                    Rover.steer = Rover.MAX_STEER_RIGHT
                # otherwise tread slowly at pure home angle
                elif -23 < Rover.angle_from_home < 23:
                    Rover.throttle = PARK_THROTTLE_SET
                    Rover.steer = np.clip(Rover.angle_from_home,
                                          Rover.MAX_STEER_RIGHT,
                                          Rover.MAX_STEER_LEFT)
    

    

def ParkingAtHome(Rover):
    
    # Brake if still moving
    if Rover.vel > Rover.MIN_VEL:
            Rover.throttle = 0
            Rover.brake = Rover.MAX_BRAKE
            Rover.steer = 0
