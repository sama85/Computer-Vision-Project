"""
Module for rover decision-handling.

Used to build a decision tree for determining throttle, brake and
steer commands based on the output of the perception_step() function
in the perception module.

"""
import states
import transition_handlers


"""Handles switching between states"""
class DecisionMaker():

    def __init__(self):

        self.state = [
            states.FollowingLeftWall, 
            states.TurningToLeftWall,
            states.AvoidingLeftWall,
            states.AvoidingObstacles,
            states.GoingToSample,
            states.StoppingAtSample,
            states.GettingUnstuck,
            states.ReturningHome,
            states.ParkingAtHome
        ]     


        self.curr_state = self.state[1]  # TurningToLeftWall

    def switch_to_state(self, Rover, name):

        self.curr_state = name
        name(self,Rover)

    def run(self, Rover):
        """Select and call the handler for the current state."""

        # Ensure Rover telemetry data is coming in
        if Rover.nav_angles is not None:
            # Defining the handlers of each state
            select = {
                self.state[0]: transition_handlers.following_left_wall_transitions,
                self.state[1]: transition_handlers.turning_to_left_wall_transitions,
                self.state[2]: transition_handlers.avoiding_left_wall_transitions,
                self.state[3]: transition_handlers.avoiding_obstacles_transitions,
                self.state[4]: transition_handlers.going_to_sample_transitions,
                self.state[5]: transition_handlers.stopping_at_sample_transitions,
                self.state[6]: transition_handlers.getting_unstuck_transitions,
                self.state[7]: transition_handlers.returning_home_transitions,
                self.state[8]: transition_handlers.parking_at_home_transitions
            }
            # Select and call the handler function for the current state
            func = select.get(self.curr_state)

            if func is not None:
                func(self, Rover)
        return Rover
