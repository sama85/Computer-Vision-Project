
import numpy as np
import time

def is_stuck(Rover):

        exceeded_stucktime = False
        # If not moving then check since when
        if Rover.vel < 0.1:
            # if the timer is not on, then the rover just got stuck now. Initialize timer
            if not Rover.timer_on:
                Rover.start_stuck_time = time.time()
                Rover.timer_on = True
            else:
                endtime = time.time()
                exceeded_stucktime = (endtime - Rover.start_stuck_time) > Rover.MAX_STUCK_TIME
        # no longer stuck. turn off timer
        else:  
            Rover.timer_on = False
            
        return exceeded_stucktime



def pointed_along_wall(Rover, safe_pixs=500, wall_angle_offset=-10):

    left_nav_pixs = len(Rover.nav_angles_left)
    angle_from_left_wall = np.mean(Rover.nav_angles_left)

    return (left_nav_pixs >= safe_pixs
            and angle_from_left_wall > wall_angle_offset)


def deviated_from_left_wall(Rover, max_angle_wall=25):

    angle_from_left_wall = np.mean(Rover.nav_angles_left)
    return angle_from_left_wall > max_angle_wall


def obstacle_at_front(Rover, safe_pixs=600):

    nav_pixs_front = len(Rover.nav_angles)
    return nav_pixs_front < safe_pixs


def obstacle_on_left(Rover, safe_pixs=50):

    left_nav_pixs = len(Rover.nav_angles_left)
    return left_nav_pixs < safe_pixs


def sample_located(Rover, rock_dist_limit=71, min_left_angle=0.0, max_right_angle=-17):

    rock_angle = np.mean(Rover.rock_angles)
    rock_distance = np.mean(Rover.rock_dists)

    return ((rock_angle >= min_left_angle or rock_angle > max_right_angle)
            and rock_distance < rock_dist_limit)


# def sample_on_right_close(Rover, rock_dist_limit=75, max_right_angle=17):

#     rock_angle = np.mean(Rover.rock_angles)
#     rock_distance = np.mean(Rover.rock_dists)

#     return (rock_angle > -max_right_angle
#             and rock_distance < rock_dist_limit)


def completed_mission(Rover, min_samples=5, min_mapped=95):

    return (Rover.samples_collected >= min_samples
            and Rover.perc_mapped >= min_mapped)


def reached_home(Rover, max_dist=3):
    
    return Rover.going_home and Rover.distance_from_home < max_dist