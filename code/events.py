
import numpy as np
import time

def is_stuck(Rover):
        """Check if rover is stuck for stucktime."""
        exceeded_stucktime = False
        # If not moving then check since when
        if Rover.vel < 0.1:
            if not Rover.timer_on:
                Rover.start_stuck_time = time.time()  # start timer
                Rover.stuck_heading = Rover.yaw
                Rover.timer_on = True
            else:
                endtime = time.time()
                exceeded_stucktime = (endtime - Rover.start_stuck_time) > Rover.MAX_STUCK_TIME
        else:  # if started to move then switch OFF/Reset timer
            Rover.timer_on = False
            Rover.stuck_heading = 0.0
        return exceeded_stucktime



def pointed_along_wall(Rover, safe_pixs=500, wall_angle_bias=-10):
    """
    Check if rover is pointing along left wall at some safe distance.

    """
    nav_pixs_left = len(Rover.nav_angles_left)
    nav_heading_left = np.mean(Rover.nav_angles_left) + wall_angle_bias

    return (nav_pixs_left >= safe_pixs
            and nav_heading_left > 0)


def deviated_from_wall(Rover, max_angle_wall=25, safe_pixs = 500):
    """
    Check if rover has deviated from left wall beyond specified angle limit.
    """
    nav_pixs_left = len(Rover.nav_angles_left)
    nav_heading_left = np.mean(Rover.nav_angles_left)
    return nav_heading_left > max_angle_wall and nav_pixs_left >= safe_pixs


def obstacle_at_front(Rover, safe_pixs=600):
    """
    Check if rover is up against some obstacle on the front.

    """
    nav_pixs_front = len(Rover.nav_angles)
    return nav_pixs_front < safe_pixs


def obstacle_on_left(Rover, safe_pixs=50):
    """
    Check if obstacle is to the left of rover.
    """

    nav_pixs_left = len(Rover.nav_angles_left)
    return nav_pixs_left < safe_pixs


def sample_on_left(Rover, rock_dist_limit=71, min_left_angle=0.0):
    """Check if a sample is spotted on the left.
    """

    rock_heading = np.mean(Rover.rock_angles)
    rock_distance = np.mean(Rover.rock_dists)

    return (rock_heading >= min_left_angle
            and rock_distance < rock_dist_limit)


def sample_on_right_close(Rover, rock_dist_limit=75, max_right_angle=17):
    """
    Check if a nearby sample is spotted on the right.

    """

    rock_heading = np.mean(Rover.rock_angles)
    rock_distance = np.mean(Rover.rock_dists)

    return (rock_heading > -max_right_angle
            and rock_distance < rock_dist_limit)


def completed_mission(Rover, min_samples=1, min_mapped=95, max_time=200):
    """Check if rover has completed mission criteria."""
    return (Rover.samples_collected >= min_samples
            and Rover.perc_mapped >= min_mapped)
            # or Rover.total_time >= max_time


def reached_home(Rover, max_dist=3):
    """Check if rover has reached home after completing mission."""
    return Rover.going_home and Rover.home_distance < max_dist