"""

Path planning with Rapidly-Exploring Random Trees (RRT)

author: Aakash(@nimrobotics)
web: nimrobotics.github.io

"""

import cv2
import numpy as np
import math
import random
import argparse
import os

class Nodes:
    """Class to store the RRT graph"""
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.parent_x = []
        self.parent_y = []

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y


# check collision
def collision(x1,y1,x2,y2):
    color=[]

    # x = list(np.arange(x1,x2,(x2-x1)/100))

    # print("Type of x: ", type(x))
    # print("Type of x1: ", type(x1))
    # print("Type of x-x1:", type(x-x1))
    # print("Type of y1: ", type(y1))
    try:
        x = list(np.arange(x1,x2,(x2-x1)/100))

        y = list(((y2-y1)/(x2-x1))*(x-x1) + y1)


        # y = list(((y2-y1)/(x2-x1))*([xx-x1 for xx in x]) + y1)
        # print("collision",x,y)
        for i in range(len(x)):
            # print(int(x[i]),int(y[i]))
            color.append(img[int(y[i]),int(x[i])])
        if (0 in color):
            return True #collision
        else:
            return False #no-collision
    except ValueError:
        print("Value error thrown...")
        print("When it is thrown, the value of x1: ", x1)
        print("When it is thrown, the value of x2: ", x2)
        print("When it is thrown: the value of (x2-x1)/100: ", (x2-x1)/100)

# check the  collision with obstacle and trim
# def check_collision(x1,y1,x2,y2):
def check_collision(tx, ty, nearest_x, nearest_y):
    # _,theta = dist_and_angle(x2,y2,x1,y1)
    # x=x2 + stepSize*np.cos(theta)
    # y=y2 + stepSize*np.sin(theta)
    # print(x2,y2,x1,y1)
    # print("theta",theta)
    # print("check_collision",x,y)

    # TODO: trim the branch if its going out of image area
    # print("Image shape",img.shape)
    hy,hx=img.shape
    # if y<0 or y>hy or x<0 or x>hx:
    if ty < 0 or ty >= hy or tx < 0 or tx >= hx:
        print("Point out of image bound")
        directCon = False
        nodeCon = False
    else:
        # check direct connection
        # if collision(x,y,end[0],end[1]):
        if collision(tx, ty, end[0], end[1]):
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
        if collision(tx, ty, nearest_x, nearest_y):
            nodeCon = False
        else:
            nodeCon = True

    # return(x,y,directCon,nodeCon)
    return directCon, nodeCon


# return dist and angle b/w new point and nearest node
def dist_and_angle(x1,y1,x2,y2):
    dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
    angle = math.atan2(y2-y1, x2-x1)
    return dist, angle


# return the neaerst node index
def nearest_node(x,y):
    temp_dist=[]
    for i in range(len(node_list)):
        dist,_ = dist_and_angle(x,y,node_list[i].x,node_list[i].y)
        temp_dist.append(dist)
    return temp_dist.index(min(temp_dist))

# generate a random point in the image space
def rnd_point(h,l):
    new_y = random.randint(0, h)
    new_x = random.randint(0, l)
    return (new_x,new_y)

def force_generation():
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0),
                  (1, 1), (1, -1), (-1, 1), (-1, -1)]
    x_magnitude = random.randint(1, 10)
    y_magnitude = random.randint(1, 10)
    magnitudes = (x_magnitude, y_magnitude)

    direct_idx = random.randint(0, 7)
    direction_sampled = directions[direct_idx]
    return direction_sampled, magnitudes

def steer_towards(nearest_x, nearest_y, nx, ny, step_size):
    dis, angle = dist_and_angle(nx, ny, nearest_x, nearest_y)
    direction_sampled, magnitudes = force_generation()
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


def sample_neighbours_and_add_edge(x_new: Nodes, radius):
    """
    Samples all the neighbour nodes within a neighbourhood circle of radius 'radius'.
    """
    neighbours = []
    for i in range(len(node_list)):
        temp_node = node_list[i]
        dist, _ = dist_and_angle(x_new.x, x_new.y, temp_node.x, temp_node.y)
        if dist <= radius and temp_node != x_new:
            if not collision(x_new.x, x_new.y, temp_node.x, temp_node.y):
                x_new.parent_x.append(temp_node.x)
                x_new.parent_y.append(temp_node.y)
                temp_node.parent_x.append(x_new.x)
                temp_node.parent_y.append(x_new.y)


def RRG(img, img2, start, end, stepSize):
    h,l= img.shape # dim of the loaded image
    # print(img.shape) # (384, 683)
    # print(h,l)

    # neighbouring radius used in RRG algorithm
    neighbour_radius = 65


    # insert the starting point in the node class
    # node_list = [0] # list to store all the node points         
    node_list[0] = Nodes(start[0],start[1])
    node_list[0].parent_x.append(start[0])
    node_list[0].parent_y.append(start[1])

    # display start and end
    cv2.circle(img2, (start[0],start[1]), 5,(0,0,255),thickness=3, lineType=8)
    cv2.circle(img2, (end[0],end[1]), 5,(0,0,255),thickness=3, lineType=8)

    i=1
    pathFound = False

    while not pathFound:
        nx, ny = rnd_point(h, l)
        print("Random points:", nx, ny)

        # Uncertainty here
        # direction_sampled, magnitudes = force_generation()
        # x_force, y_force = direction_sampled
        # x_magnitude, y_magnitude = magnitudes
        # x_diff, y_diff = x_force * x_magnitude, y_force * y_magnitude
        # nx += x_force
        # ny += y_force

        nearest_ind = nearest_node(nx,ny)

        nearest = node_list[nearest_ind]

        nearest_x = nearest.x
        nearest_y = nearest.y
        print("Nearest node coordinates:",nearest_x,nearest_y)

        # Here we do the steering
        # tx, ty = steer_towards(nx, ny, nearest_x, nearest_y, stepSize)
        tx, ty = steer_towards(nearest_x, nearest_y, nx, ny, stepSize)
        print("type of tx: ", type(tx))
        print("type of ty: ", type(ty))
        node_to_be_added = Nodes(tx, ty)
        # check direct connection
        # tx,ty,directCon,nodeCon = check_collision(nx,ny,nearest_x,nearest_y)
        directCon, nodeCon = check_collision(np.float64(tx), np.float64(ty), nearest_x, nearest_y)
        print("Check collision:",tx,ty,directCon,nodeCon)

        if directCon and nodeCon:
            print("Node can connect directly with end")
            node_list.append(i)
            node_list[i] = node_to_be_added
            node_to_be_added.parent_x = nearest.parent_x.copy()
            node_to_be_added.parent_y = nearest.parent_y.copy()
            node_to_be_added.parent_x.append(nearest_x)
            node_to_be_added.parent_y.append(nearest_y)

            # To make it into RRG: extension needs to add a couple of more edges
            nearest.parent_x.append(tx)
            nearest.parent_y.append(ty)

            sample_neighbours_and_add_edge(node_to_be_added, neighbour_radius)
            # node_list[i].parent_x = node_list[nearest_ind].parent_x.copy()
            # node_list[i].parent_y = node_list[nearest_ind].parent_y.copy()
            # node_list[i].parent_x.append(tx)
            # node_list[i].parent_y.append(ty)

            cv2.circle(img2, (int(tx),int(ty)), 2,(0,0,255),thickness=3, lineType=8)
            cv2.line(img2, (int(tx),int(ty)), (int(node_list[nearest_ind].x),int(node_list[nearest_ind].y)), (0,255,0), thickness=1, lineType=8)
            cv2.line(img2, (int(tx),int(ty)), (end[0],end[1]), (255,0,0), thickness=2, lineType=8)

            print("Path has been found")
            #print("parent_x",node_list[i].parent_x)
            for j in range(len(node_list[i].parent_x)-1):
                cv2.line(img2, (int(node_list[i].parent_x[j]),int(node_list[i].parent_y[j])), (int(node_list[i].parent_x[j+1]),int(node_list[i].parent_y[j+1])), (255,0,0), thickness=2, lineType=8)
            # cv2.waitKey(1)
            cv2.imwrite("media/"+str(i)+".jpg",img2)
            cv2.imwrite("out.jpg",img2)
            break

        elif nodeCon:

            node_list.append(node_to_be_added)
            node_to_be_added.parent_x = nearest.parent_x.copy()
            node_to_be_added.parent_y = nearest.parent_y.copy()
            node_to_be_added.parent_x.append(nearest_x)
            node_to_be_added.parent_y.append(nearest_y)

            # Again: doing the RRG extension
            # (The original code is really design in a weird way...)
            nearest.parent_x.append(nearest_x)
            nearest.parent_y.append(nearest_y)

            sample_neighbours_and_add_edge(node_to_be_added, neighbour_radius)

            i=i+1
            # display
            cv2.circle(img2, (int(tx), int(ty)), 2, (0, 0, 255), thickness=3, lineType=8)
            cv2.line(img2, (int(tx), int(ty)), (int(node_list[nearest_ind].x), int(node_list[nearest_ind].y)), (0, 255, 0), thickness=1, lineType=8)
            cv2.imwrite("media/"+str(i)+".jpg", img2)
            cv2.imshow("sdc", img2)
            cv2.waitKey(1)
            continue

        else:
            print("No direct con. and no node con. :( Generating new rnd numbers")
            continue


def draw_circle(event,x,y,flags,param):
    global coordinates
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img2,(x,y),5,(255,0,0),-1)
        coordinates.append(x)
        coordinates.append(y)


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description = 'Below are the params:')
    parser.add_argument('-p', type=str, default='world2.png',metavar='ImagePath', action='store', dest='imagePath',
                    help='Path of the image containing mazes')
    parser.add_argument('-s', type=int, default=10,metavar='Stepsize', action='store', dest='stepSize',
                    help='Step-size to be used for RRT branches')
    parser.add_argument('-start', type=int, default=[20,20], metavar='startCoord', dest='start', nargs='+',
                    help='Starting position in the maze')
    parser.add_argument('-stop', type=int, default=[450,250], metavar='stopCoord', dest='stop', nargs='+',
                    help='End position in the maze')
    parser.add_argument('-selectPoint', help='Select start and end points from figure', action='store_true')

    args = parser.parse_args()

    # remove previously stored data
    try:
      os.system("rm -rf media")
    except:
      print("Dir already clean")
    os.mkdir("media")

    img = cv2.imread(args.imagePath,0) # load grayscale maze image
    img2 = cv2.imread(args.imagePath) # load colored maze image
    start = tuple(args.start) #(20,20) # starting coordinate
    end = tuple(args.stop) #(450,250) # target coordinate
    stepSize = args.stepSize # stepsize for RRT
    node_list = [0] # list to store all the node points

    coordinates=[]

    # run the RRT algorithm 
    RRG(img, img2, start, end, stepSize)