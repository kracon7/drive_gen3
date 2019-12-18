import numpy as np

# Class that describes our action space.
# We can get an action by index
class ActionSpace():

    def __init__(self, dp):
        self.actions = {'left':    [-dp, 0,  0 ],
                        'right':   [ dp, 0,  0 ],
                        'forward': [ 0,  dp, 0 ],
                        'backward':[ 0, -dp, 0 ],
                        'up':      [ 0,  0,  dp],
                        'down':    [ 0,  0, -dp]}

    def get_action(self, action_key):
        assert(action_key in self.actions.keys())
        return self.actions[action_key]