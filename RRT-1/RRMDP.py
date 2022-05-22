class MarkovDecisionProcess:
    """A Markov Decision Process"""
    def __init__(self):
        self.states = []
        self.transition = {}
        self.cost = {}

# States: a list of tuples
# Actions: a tuple of two tuples, denoting moving from one state to another
# transition: a dictionary of...
#   key: tuple indicating the xy coordinate
#   values: