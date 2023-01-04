"""
Module for rover decision-handling.

Used to build a decision tree for determining throttle, brake and
steer commands based on the output of the perception_step() function
in the perception module.

"""



import numpy as np

import events
import states
import handlers


class DecisionMaker():
    """Handle events and switch between states."""

    def __init__(self):
        """Initialize a DecisionMaker instance."""
        self.state = [
            states.FollowingLeftWall, 
            states.TurningToLeftWall,
            states.AvoidingLeftWall,
            states.AvoidingObstacles,
            states.GoingToSample,
            states.Stopping,
            states.GettingUnstuck,
            states.ReturningHome,
            states.ParkingAtHome
        ]     
        # Default state
        self.curr_state = self.state[1]  # TurningToLeftWall
        self.starttime = 0.0  # for timer

    def switch_to_state(self, Rover, name):
        """Update current state to the next state."""
        self.curr_state = name
        name(self,Rover)

    

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
            func = select.get(self.curr_state)

            if func is not None:
                func(self, Rover)
        return Rover
