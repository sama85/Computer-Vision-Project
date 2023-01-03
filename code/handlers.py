"""
Module for handling switching from states.

"""
import numpy as np

def front_path_clear(Rover, safe_pixs=500):
    """
    Check if sufficient room ahead.

    Keyword arguments:
    safe_pixs -- minimum number of pixels in front to deem front path clear
    """
    nav_pixs_front = len(Rover.nav_angles)
    return nav_pixs_front >= safe_pixs


def left_path_clear(Rover, safe_pixs=1500):
    """
    Check if sufficient room on left.

    Keyword arguments:
    safe_pixs -- minimum number of pixels on left to deem left path clear
    """
    nav_pixs_left = len(Rover.nav_angles_left)
    return nav_pixs_left >= safe_pixs


def pointed_at_nav(Rover, angle_limit=17):
    """
    Check if rover is pointed within some range of average nav heading.

    Keyword arguments:
    angle_limit -- angle range limit for nav angles (degrees)
    """
    nav_heading = np.mean(Rover.nav_angles)
    return -angle_limit <= nav_heading <= angle_limit


def pointed_along_wall(Rover, safe_pixs=500, wall_angle_bias=-10):
    """
    Check if rover is pointing along left wall at some safe distance.

    Keyword arguments:
    safe_pixs --  minimum number of pixels to keep from wall
    wall_angle_bias -- to bias rover heading for pointing along wall (degrees)
    """
    nav_pixs_left = len(Rover.nav_angles_left)
    nav_heading_left = np.mean(Rover.nav_angles_left) + wall_angle_bias

    return (nav_pixs_left >= safe_pixs
            and nav_heading_left > 0)


def deviated_from_wall(Rover, max_angle_wall=25):
    """
    Check if rover has deviated from left wall beyond specified angle limit.

    Keyword arguments:
    max_angle_wall --  maximum allowed angle from left wall (degrees)
    """
    nav_heading_left = np.mean(Rover.nav_angles_left)
    return nav_heading_left > max_angle_wall


def at_front_obstacle(Rover, safe_pixs=600):
    """
    Check if rover is up against some obstacle on the front.

    Keyword arguments:
    safe_pixs -- minimum number of pixels to keep from front obstacles
    """
    nav_pixs_front = len(Rover.nav_angles)
    return nav_pixs_front < safe_pixs


def at_left_obstacle(Rover, safe_pixs=50):
    """
    Check if obstacle is to the left of rover.

    Keyword arguments:
    safe_pixs -- minimum number of pixels to keep from left obstacles
    """

    nav_pixs_left = len(Rover.nav_angles_left)
    return nav_pixs_left < safe_pixs


def sample_on_left(Rover, rock_dist_limit=71, min_left_angle=0.0):
    """Check if a sample is spotted on the left.

    Keyword arguments:
    rock_dist_limit -- rocks only detected when under this dist from rover
    min_left_angle -- rocks only detected when left of this angle from rover
    """

    rock_heading = np.mean(Rover.rock_angles)
    rock_distance = np.mean(Rover.rock_dists)

    return (rock_heading >= min_left_angle
            and rock_distance < rock_dist_limit)


def sample_right_close(Rover, rock_dist_limit=75, max_right_angle=17):
    """
    Check if a nearby sample is spotted on the right.

    Keyword arguments:
    rock_dist_limit -- rocks only detected when under this limit
    max_right_angle -- only rocks to left/above of this are considered
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



def following_wall(Decider, Rover):
    """Handle switching from FollowWall state."""
    # Time in seconds allowed to remain stuck in this state
    stucktime = 2.0

    #if deviating 
    if deviated_from_wall(Rover=Rover) and left_path_clear(Rover=Rover):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[1])  # TurnToWall

    elif at_left_obstacle(Rover=Rover):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[2])  # AvoidWall

    elif sample_on_left(Rover=Rover) or sample_right_close(Rover=Rover):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[4])  # GoToSample

    elif completed_mission(Rover=Rover):
        Rover.going_home = True
        Decider.switch_to_state(Rover, Decider.state[7])  # ReturnHome

    elif Decider.is_stuck_for(Rover, stucktime):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[6])  # GetUnstuck
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def turning_to_wall(Decider, Rover):
    """Handle switching from TurnToWall state."""
    if pointed_along_wall(Rover=Rover):
        Decider.switch_to_state(Rover, Decider.state[0])  # FollowWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)

def avoiding_wall(Decider, Rover):
    """Handle switching from AvoidWall state."""
    if pointed_along_wall(Rover=Rover):
        Decider.switch_to_state(Rover, Decider.state[0])  # FollowWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)

def avoiding_obstacles(Decider, Rover):
    """Handle switching from AvoidObstacles state."""
    stucktime = 2.0
    if completed_mission(Rover=Rover):
        Decider.switch_to_state(Rover, Decider.state[7])  # ReturnHome
    elif pointed_along_wall(Rover=Rover):
        Decider.switch_to_state(Rover, Decider.state[0])  # FollowWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)

def going_to_sample(Decider, Rover):
    """Handle switching from GoToSample state."""
    # Time in seconds allowed to remain stuck in this state
    stucktime = 4.0
    if Rover.near_sample:
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[5])  # Stop

    elif Decider.is_stuck_for(Rover, stucktime):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[6])  # GetUnstuck
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def stopped_at_sample(Decider, Rover):
    """Handle switching from Stop state."""
    Rover.send_pickup = True
    if Rover.picking_up == 0:
        Decider.switch_to_state(Rover, Decider.state[2])  # AvoidWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def getting_unstuck(Decider, Rover):
    """Handle switching from GetUnstuck state."""
    # If reached sufficient velocity or stuck while in GetUnstuck state
    # then get out of GetUnstuck state
    stucktime = 2.3
    if Rover.vel >= 1.0:
        if Rover.going_home:
            Decider.switch_to_state(Rover, Decider.state[7])  # ReturnHome
        else:
            Decider.switch_to_state(Rover, Decider.state[0])  # FollowWall

    elif Decider.is_stuck_for(Rover, stucktime):
        Rover.timer_on = False
        if Rover.going_home:
            Decider.switch_to_state(Rover, Decider.state[7])  # ReturnHome
        else:
            Decider.switch_to_state(Rover, Decider.state[0])  # FollowWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def returning_home(Decider, Rover):
    """Handle switching from ReturnHome state."""
    # Time in seconds allowed to remain stuck in this state
    stucktime = 2.5
    if at_front_obstacle(Rover=Rover):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[3])  # AvoidObstacles

    elif reached_home(Rover=Rover):        
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[8])  # Park

    elif Decider.is_stuck_for(Rover, stucktime):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[6])  # GetUnstuck
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def parking(Decider, Rover):
    """Handle switching from Park state."""
    Decider.switch_to_state(Rover, Decider.state[8])  # Remain in Park
