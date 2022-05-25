import sys
import time
import pickle
import numpy as np
import math
import random
import cv2


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

# return dist and angle b/w new point and nearest node
def dist_and_angle(x1,y1,x2,y2):
    dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
    angle = math.atan2(y2-y1, x2-x1)
    return dist, angle


class Nodes:
    """Class to store the RRT graph"""
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.parent_x = []
        self.parent_y = []


class RRMDP:
    def __init__(self, img, img2):
        self.mdp = MarkovDecisionProcess()
        self.img = img
        self.img2 = img2

    def collision(self, x1, y1, x2, y2):
        color = []

        # x = list(np.arange(x1,x2,(x2-x1)/100))

        # print("Type of x: ", type(x))
        # print("Type of x1: ", type(x1))
        # print("Type of x-x1:", type(x-x1))
        # print("Type of y1: ", type(y1))
        try:
            x = list(np.arange(x1, x2, (x2 - x1) / 100))

            y = list(((y2 - y1) / (x2 - x1)) * (x - x1) + y1)

            # y = list(((y2-y1)/(x2-x1))*([xx-x1 for xx in x]) + y1)
            # print("collision",x,y)
            for i in range(len(x)):
                # print(int(x[i]),int(y[i]))
                color.append(self.img[int(y[i]), int(x[i])])
            if 0 in color:
                return True  # collision
            else:
                return False  # no-collision
        except ValueError:
            print("Value error thrown...")
            print("When it is thrown, the value of x1: ", x1)
            print("When it is thrown, the value of x2: ", x2)
            print("When it is thrown: the value of (x2-x1)/100: ", (x2 - x1) / 100)

    def check_collision(self, tx, ty, nearest_x, nearest_y, end):

        # TODO: trim the branch if its going out of image area
        # print("Image shape",img.shape)
        hy, hx = self.img.shape
        # if y<0 or y>hy or x<0 or x>hx:
        if ty < 0 or ty >= hy or tx < 0 or tx >= hx:
            print("Point out of image bound")
            directCon = False
            nodeCon = False
        else:
            # check direct connection
            # if collision(x,y,end[0],end[1]):
            if self.collision(tx, ty, end[0], end[1]):
                directCon = False
            else:
                # d, _ = dist_and_angle(x1, y1, x2, y2)
                d, _ = dist_and_angle(tx, ty, nearest_x, nearest_y)
                if d < 5:
                    directCon = True
                else:
                    directCon = False

            # check connection between two nodes
            # if collision(x,y,x2,y2):
            if self.collision(tx, ty, nearest_x, nearest_y):
                nodeCon = False
            else:
                nodeCon = True

        # return(x,y,directCon,nodeCon)
        return directCon, nodeCon

    def nearest_state(self, x, y):
        temp_dist = []
        states = self.mdp.states
        for i in range(len(states)):
            dist, _ = dist_and_angle(x, y, states[i].x, states[i].y)
            temp_dist.append(dist)
        return temp_dist.index(min(temp_dist))

    def rnd_point(self0,h, l):
        new_y = random.randint(0, h)
        new_x = random.randint(0, l)
        return new_x, new_y

    def force_generation(self):
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0),
                      (1, 1), (1, -1), (-1, 1), (-1, -1)]
        x_magnitude = random.randint(1, 10)
        y_magnitude = random.randint(1, 10)
        magnitudes = (x_magnitude, y_magnitude)

        direct_idx = random.randint(0, 7)
        direction_sampled = directions[direct_idx]
        return direction_sampled, magnitudes

    def steer_towards(self, nearest_x, nearest_y, nx, ny, step_size):
        dis, angle = dist_and_angle(nx, ny, nearest_x, nearest_y)
        direction_sampled, magnitudes = self.force_generation()
        x_dir, y_dir = direction_sampled
        x_magnitude, y_magnitude = magnitudes

        dis, theta = dist_and_angle(nearest_x, nearest_y, nx, ny)
        if dis <= step_size:
            tx = nx
            ty = ny
        else:
            tx = nearest_x + step_size * math.cos(theta)
            ty = nearest_y + step_size * math.sin(theta)
        print("tx, ty before applying the force: ", tx, ty)
        tx += x_dir * x_magnitude
        ty += y_dir * y_magnitude
        print("x_dir, y_dir: ", x_dir, y_dir)
        print("x_magnitude, y_magnitude: ", x_magnitude, y_magnitude)
        print("tx, ty after applying the force: ", tx, ty)
        return tx, ty


    def extend_mdp(self, start, end, step_size):
        # TODO: the main extension algorithm starts here
# return the neaerst node index
# def nearest_node(x,y):
#     temp_dist=[]
#     for i in range(len(node_list)):
#         dist,_ = dist_and_angle(x,y,node_list[i].x,node_list[i].y)
#         temp_dist.append(dist)
#     return temp_dist.index(min(temp_dist))


# generate a random point in the image space
