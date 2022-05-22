
class MarkovDecisionProcess:
    """A Markov Decision Process"""
    def __init__(self):
        self.states = []
        self.transitions = {}
        self.costs = {}
        self.actions = {}


# States: a list of tuples, (-1, -1) for dead-end state

# Actions: a tuple of two tuples, denoting moving from one state to another
# transition: a dictionary of...
#   keys: a tuple denoting the current state
#   values: a dictionary with...
#       keys: a tuple of two tuples, denoting actions
#       values: a dictionary with...
#           keys: a tuple denoting the next state
#           values: a double denoting the probability of transitioning to that state
# Costs: A dictionary with...
#       keys: a tuple denoting the state
#       values: a dictionary with....
#           keys: a tuple of two tuples denoting actions
#           values: a dictionary with...
#               keys: a tuple denoting the state ending up at
#               values: a tuple denoting two costs: the first is the primary objective: probability of reaching
#                       a goal state(we want to maximize this); and the second is the secondary objective:
#                       the cost of in the form of distance covered(we want to minimize this)
# TODO: consult the original paper about how to calculate the probability of reaching a goal
# TODO: introduce more environment so that it may be impossible to reach a goal.

# Cost: this j