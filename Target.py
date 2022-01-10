
from constants import Constants


class Target(object):
    """
        Class to define Point properties
    """
    def __init__(self,x,y,r=10,color='red'):
        """ Init function """

        # X coordinates
        self.x = x

        # Y coordinates
        self.y = y 

        # Radius size
        self.r = r 

        # Id of the agent assigned to that target
        self.agent = Constants.UNASSIGNED_TARGET

        # Type of point = ['red','green']
        self.color = color

    # Override hash method
    def __hash__(self):
        return int(str(self.x)+str(self.y))

    # Override eq method for comparing targets
    def __eq__(self, other):
        return (self.x == other.x and self.y == other.y)

    # Override print method for pretty printing object properties
    def __str__(self):
        return f"Target: ({self.x},{self.y}); agent:{self.agent}"