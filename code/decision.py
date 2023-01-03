"""
Module for rover decision-handling.

Used to build a decision tree for determining throttle, brake and
steer commands based on the output of the perception_step() function
in the perception module.

"""



import time

import numpy as np

import events
import states
import handlers


class DecisionSupervisor():
    """Handle events and switch between states."""

    def __init__(self):
        """Initialize a DecisionSupervisor instance."""
        self.state = [
            states.FollowWall, 
            states.TurnToWall,
            states.AvoidWall,
            states.AvoidObstacles,
            states.GoToSample,
            states.Stop,
            states.GetUnstuck,
            states.ReturnHome,
            states.Park
        ]     
        # Default state
        self.curr_state = self.state[1]  # TurnToWall
        self.starttime = 0.0  # for timer

    def switch_to_state(self, Rover, name):
        """Update current state to the next state."""
        name(Rover)
        self.curr_state = name

    def is_stuck_for(self, Rover, stucktime):
        """Check if rover is stuck for stucktime."""
        exceeded_stucktime = False
        # If not moving then check since when
        if Rover.vel < 0.1:
            if not Rover.timer_on:
                self.starttime = time.time()  # start timer
                Rover.stuck_heading = Rover.yaw
                Rover.timer_on = True
            else:
                endtime = time.time()
                exceeded_stucktime = (endtime - self.starttime) > stucktime
        else:  # if started to move then switch OFF/Reset timer
            Rover.timer_on = False
            Rover.stuck_heading = 0.0
        return exceeded_stucktime

    def execute(self, Rover):
        """Select and call the handler for the current state."""
        # Ensure Rover telemetry data is coming in
        if Rover.nav_angles is not None:
            # State identifiers and corresponding handlers
            select = {
                self.state[0]: handlers.following_wall,
                self.state[1]: handlers.turning_to_wall,
                self.state[2]: handlers.avoiding_wall,
                self.state[3]: handlers.avoiding_obstacles,
                self.state[4]: handlers.going_to_sample,
                self.state[5]: handlers.stopped_at_sample,
                self.state[6]: handlers.getting_unstuck,
                self.state[7]: handlers.returning_home,
                self.state[8]: handlers.parking
            }
            # Select and call the handler function for the current state
            func = select.get(self.curr_state, lambda: "nothing")
            func(self, Rover)
        return Rover
