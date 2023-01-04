"""
Module for handling switching from states.

"""
import events



def following_left_wall_transitions(Decider, Rover):
    """Handle switching from FollowingLeftWall state."""
 
    if events.deviated_from_left_wall(Rover=Rover):        
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[1])  # TurningToLeftWall

    elif events.obstacle_on_left(Rover=Rover):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[2])  # AvoidingLeftWall

    elif events.sample_on_left(Rover=Rover) or events.sample_on_right_close(Rover=Rover):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[4])  # GoingToSample

    elif events.completed_mission(Rover=Rover):
        Rover.going_home = True
        Decider.switch_to_state(Rover, Decider.state[7])  # ReturningHome

    elif events.is_stuck(Rover=Rover):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[6])  # GettingUnstuck
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def turning_to_left_wall_transitions(Decider, Rover):
    """Handle switching from TurningToLeftWall state."""
    if events.pointed_along_wall(Rover=Rover):
        Decider.switch_to_state(Rover, Decider.state[0])  # FollowingLeftWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)

def avoiding_left_wall_transitions(Decider, Rover):
    """Handle switching from AvoidingLeftWall state."""
    if events.pointed_along_wall(Rover=Rover):
        Decider.switch_to_state(Rover, Decider.state[0])  # FollowingLeftWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)

def avoiding_obstacles_transitions(Decider, Rover):
    """Handle switching from AvoidingObstacles state."""
    if events.completed_mission(Rover=Rover):
        Decider.switch_to_state(Rover, Decider.state[7])  # ReturningHome
    elif events.pointed_along_wall(Rover=Rover):
        Decider.switch_to_state(Rover, Decider.state[0])  # FollowingLeftWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)

def going_to_sample_transitions(Decider, Rover):
    """Handle switching from GoingToSample state."""
    # Time in seconds allowed to remain stuck in this state
    if Rover.near_sample:
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[5])  # StoppingAtSample

    elif events.is_stuck(Rover=Rover):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[6])  # GettingUnstuck
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)

def getting_unstuck_transitions(Decider, Rover):
    """Handle switching from GettingUnstuck state."""
    # If reached sufficient velocity or stuck while in GettingUnstuck state
    # then get out of GettingUnstuck state
    if Rover.vel >= 1.0:
        if Rover.going_home:
            Decider.switch_to_state(Rover, Decider.state[7])  # ReturningHome
        else:
            Decider.switch_to_state(Rover, Decider.state[0])  # FollowingLeftWall

    elif events.is_stuck(Rover=Rover):
        Rover.timer_on = False
        if Rover.going_home:
            Decider.switch_to_state(Rover, Decider.state[7])  # ReturningHome
        else:
            Decider.switch_to_state(Rover, Decider.state[0])  # FollowingLeftWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)

def stopping_at_sample_transitions(Decider, Rover):
    """Handle switching from StoppingAtSample state."""
    Rover.send_pickup = True
    if Rover.picking_up == 0:
        Decider.switch_to_state(Rover, Decider.state[2])  # AvoidingLeftWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)




def returning_home_transitions(Decider, Rover):
    """Handle switching from ReturningHome state."""
    # Time in seconds allowed to remain stuck in this state
    if events.obstacle_at_front(Rover=Rover):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[3])  # AvoidingObstacles

    elif events.reached_home(Rover=Rover):        
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[8])  # ParkingAtHome

    elif events.is_stuck(Rover=Rover):
        Rover.timer_on = False
        Decider.switch_to_state(Rover, Decider.state[6])  # GettingUnstuck
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def parking_at_home_transitions(Decider, Rover):
    """Handle switching from ParkingAtHome state."""
    Decider.switch_to_state(Rover, Decider.state[8])  # Remain in ParkingAtHome
