"""
Main module for decision handling

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
        print(self.curr_state)

    def switch_to_state(self, Rover, name):

        self.curr_state = name
        name(self,Rover)

    def run(self, Rover):

        if Rover.nav_angles is not None:
            # Mapping each handler to its corresponding state
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
            func = select.get(self.curr_state)

            if func is not None:
                func(self, Rover)
        return Rover
