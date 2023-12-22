import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool
from time import sleep
from math import dist
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt
import os

class _node:
    def __init__(self,pos, parent, g, h, f):
        self.pos = pos
        self.parent = parent
        self.g = g
        self.h = h
        self.f = f


class pathplanning(Node):
    def __init__(self):
        super().__init__("pathplanning")
        self.pub = self.create_publisher(Quaternion, 'map', 10)
        self.reset=self.create_publisher(Bool, 'reset', 10)
        path=os.path.abspath('')+'/src/path_planning/path_planning/2dmaps/map'
        while rclpy.ok():
            input_map=int(input("Enter map Number: "))
            msg=Bool()
            msg.data=True
            self.reset.publish(msg)
            self.map=np.array(plt.imread(path+str(input_map)+'.png'))   #<-------------------------------------
            self.n=(self.map.shape[0]-1)/10
            self.find()
            self.map_to_coppeliasim()
            self.astar()

    def find(self):
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if np.array_equal(self.map[i, j], [1.0, 0.0, 0.0]):# find start
                    self.start=_node([i,j],None,0,0,0)
                if np.array_equal(self.map[i, j], [0.0, 1.0, 0.0]): # find end
                    self.end=_node([i,j],None,0,0,0)

    def path_to_coppeliasim(self,path):
        msg = Quaternion()
        for i in range(len(path)):
            msg.x = ((path[i][0]+self.n)/(self.n*2))-3
            msg.y = ((path[i][1]+self.n)/(self.n*2))-3
            msg.w = 4.0
            self.pub.publish(msg)
            sleep(0.02)

    def map_to_coppeliasim(self):
        msg = Quaternion()
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if np.array_equal(self.map[i, j], [0.0, 0.0, 0.0]):  #send boundaries to coppeliasim
                    msg.x = ((i+self.n)/(self.n*2))-3
                    msg.y = ((j+self.n)/(self.n*2))-3
                    msg.w = 0.0
                    self.pub.publish(msg)
                    sleep(0.02)

                elif np.array_equal(self.map[i, j], [1.0, 0.0, 0.0]):  # send start to coppeliasim
                    msg.x = ((i+self.n)/(self.n*2))-3
                    msg.y = ((j+self.n)/(self.n*2))-3
                    msg.w = 2.0
                    self.pub.publish(msg)
                    sleep(0.02)

                elif np.array_equal(self.map[i, j], [0.0, 1.0, 0.0]): # send goal to coppeliasim
                    msg.x = ((i+self.n)/(self.n*2))-3
                    msg.y = ((j+self.n)/(self.n*2))-3
                    msg.w = 3.0
                    self.pub.publish(msg)
                    sleep(0.02)

    def successors(self, node):
        successors = []
        for i in range(-1,2):
            for j in range(-1,2):
                    if i == 0 and j == 0:
                        continue
                    if node.pos[0] + i < 0 or node.pos[0] + i >= self.map.shape[0]:  # check boundaries
                        continue
                    if node.pos[1] + j < 0 or node.pos[1] + j >= self.map.shape[1]:  # check boundaries
                        continue
                    if np.array_equal(self.map[node.pos[0] +i, node.pos[1] +j], [0.0, 0.0, 0.0]):  # check obstacles
                        continue
                    successors.append(_node([node.pos[0] + i, node.pos[1] + j], node,None,None,None))
        return successors
    
    def spline(self,path):
        path=np.array(path)
        x = path[:, 0]
        y = path[:, 1]
        # Fit a spline to the data
        tck, u = splprep([x, y], s=0, per=False)
        # Define new range for the spline parameter
        u_new = np.linspace(u.min(), u.max(), 100)
        # Evaluate the spline at the new parameter values
        x_spline, y_spline = splev(u_new, tck)
        path=np.array([x_spline,y_spline]).T
        return path

    def astar(self):
        open_list = []
        closed_list = []
        open_list.append(self.start)

        while len(open_list) > 0:
            open_list.sort(key=lambda x: x.f, reverse=False)
            parent = open_list.pop(0)
            closed_list.append(parent)

            if parent.pos == self.end.pos:
                path = []
                while parent.parent is not None:
                    path.append(parent.pos)
                    parent = parent.parent
                path.append(self.start.pos)
                path.reverse()
                path=self.spline(path)    #<-----------------------------------------
                self.path_to_coppeliasim(path)
                return path


            childern = self.successors(parent)


            for child in childern:

                child.h = dist(child.pos, self.end.pos)
                child.g = parent.g + dist(child.pos, parent.pos)
                child.f = child.g + child.h
                flag=0

                for i, closed_child in enumerate(closed_list):
                    if child.pos == closed_child.pos:
                        flag=1
                        if child.f < closed_child.f:
                            closed_list[i] = child

                        break

                if flag==1:
                    continue

                for i, open_child in enumerate(open_list):
                    if child.pos == open_child.pos:
                        flag=1
                        if child.f < open_child.f:
                            open_list[i] = child

                        break

                if flag==1:
                    continue
                
                open_list.append(child)

        print("Goal is unreachable")
                        

def main(args = None):
    try:
        rclpy.init(args=args)
        node = pathplanning()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
