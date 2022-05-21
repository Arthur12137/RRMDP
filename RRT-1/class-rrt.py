import cv2
import numpy as np
import math
import random
import argparse
import os

class Node:
	def __init__(self,x,y):
		self.x = x
		self.y = y 
		self.parent_x = []
		self.parent_y = []


class RRTPlanner:
	def __init__(self, img_, img2_, start_, end_):
		self.img = img_ 
		self.img2 = img2_
		self.start = start_
		self.end = end_
		self.node_list = [0]

	def collision(self, n1: Node, n2: Node) -> bool:
		x1, y1 = n1.x, n1.y 
		x2, y2 = n2.x, n2.y 
		color = []
		try:
			x = list(np.arange(x1, x2, (x2-x1)/100))
			y = list(((y2-y1)/(x2-x1))*(x-x1) + y1)

			for i in range(len(x)):
				color.append(self.img[int(y[i]), int(x[i])])
			if (0 in color):
				return True
			else:
				return False
		except ValueError:
		    print("Value error thrown...")
	        print("When it is thrown, the value of x1: ", x1)
	        print("When it is thrown, the value of x2: ", x2)
	        print("When it is thrown: the value of (x2-x1)/100: ", (x2-x1)/100)

	def check_sollision(self, t: Node, nearest: Node):
		hy, hx = self.img.shape
		tx, ty = t.x, t.y 
		nearest_x, nearest_y = nearest.x, nearest.y 
		if ty < 0 or ty >= hy or tx < 0 or tx >= hx:
			directCon = False
			nodeCon = False
		else:
			if self.collision(t, Node(end[0], end[1])):
				directCon = False
			else:
				d, _ = dist_and_angle(tx, ty, nearest_x, nearest_y)
				if d < 5:
					directCon = True
				else:
					directCon = False

			# if self.collision(t, nearest):
			nodeCon = (not collision(t, nearest))
		return (directCon, nodeCon)


# return dist and angle b/w new point and nearest node
def dist_and_angle(x1,y1,x2,y2):
    dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
    angle = math.atan2(y2-y1, x2-x1)
    return(dist,angle)











