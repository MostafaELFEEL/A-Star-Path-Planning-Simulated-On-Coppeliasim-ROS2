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
        path=os.path.abspath('')+'/src/path_planning/path_planning/3dmaps/map'
        while rclpy.ok():
            input_map=int(input("Enter map Number: "))
            msg=Bool()
            msg.data=True
            self.reset.publish(msg)
            self.map=np.array(plt.imread(path+str(input_map)+'.png'))   #<-------------------------------------
            non_white_pixels = (self.map[:, :, :3] != 1.0).any(axis=-1)
            self.nxy=(self.map.shape[0]-1)/10
            self.nz=int((self.map.shape[0]-1)/4)
            self.scale=4
            self.map=np.ones((self.map.shape[0],self.map.shape[1],self.nz))
            self.map[non_white_pixels] = 0

            if input_map==1 or input_map==5:
                self.start=_node([18,2,0],None,0,0,0)
                self.end=_node([2,18,4],None,0,0,0)
            elif input_map==2:
                self.start=_node([18,2,0],None,0,0,0)
                self.end=_node([18,18,4],None,0,0,0)
            elif input_map==3 or input_map==4:
                self.start=_node([2,18,0],None,0,0,0)
                self.end=_node([10,10,4],None,0,0,0)

            elif input_map==6:
                self.start=_node([1,18,0],None,0,0,0)
                self.end=_node([18,18,4],None,0,0,0)

            elif input_map==7:
                self.start=_node([1,18,0],None,0,0,0)
                self.end=_node([10,10,4],None,0,0,0)

            self.map[self.start.pos[0],self.start.pos[1],self.start.pos[2]]=2
            self.map[self.end.pos[0],self.end.pos[1],self.end.pos[2]]=3            

            self.map_to_coppeliasim()
            self.astar()

    

    def path_to_coppeliasim(self,path):
        msg = Quaternion()
        for i in range(len(path)):
            sleep(0.02)
            msg.x = ((path[i][0]+self.nxy)/(self.nxy*2))-3
            msg.y = ((path[i][1]+self.nxy)/(self.nxy*2))-3
            msg.z=    path[i][2]/self.scale
            msg.w = 4.0
            self.pub.publish(msg)
            

    def map_to_coppeliasim(self):
        msg = Quaternion()
        sleep(0.02)
        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                for k in range(self.nz): 
                    
                    if self.map[i,j,k]==0:  #send boundaries to coppeliasim
                        msg.x = ((i+self.nxy)/(self.nxy*2))-3
                        msg.y = ((j+self.nxy)/(self.nxy*2))-3
                        msg.z = k/self.scale
                        msg.w = 0.0
                        self.pub.publish(msg)
                        sleep(0.02)

                    elif self.map[i,j,k]==2:  # send start to coppeliasim
                        msg.x = ((i+self.nxy)/(self.nxy*2))-3
                        msg.y = ((j+self.nxy)/(self.nxy*2))-3
                        msg.z = k/self.scale
                        msg.w = 2.0
                        self.pub.publish(msg)
                        sleep(0.02)

                    elif self.map[i,j,k]==3: # send goal to coppeliasim
                        msg.x = ((i+self.nxy)/(self.nxy*2))-3
                        msg.y = ((j+self.nxy)/(self.nxy*2))-3
                        msg.z = k/self.scale
                        msg.w = 3.0
                        self.pub.publish(msg)
                        sleep(0.02)

    def successors(self, node):
        successors = []
        for i in range(-1,2):
            for j in range(-1,2):
                for k in range(-1,2):
                    if i == 0 and j == 0 and k==0:
                        continue
                    if node.pos[0] + i < 0 or node.pos[0] + i >= self.map.shape[0]:  # check boundaries
                        continue
                    if node.pos[1] + j < 0 or node.pos[1] + j >= self.map.shape[1]:  # check boundaries
                        continue
                    if node.pos[2] + k < 0 or node.pos[2] + k >= self.nz:  # check boundaries
                        continue
                    if self.map[node.pos[0] + i,node.pos[1] + j,node.pos[2] + k]==0:  # check obstacles
                        continue
                    successors.append(_node([node.pos[0] + i, node.pos[1] + j,node.pos[2] + k], node,None,None,None))
        return successors
    
    def spline(self,path):
        path=np.array(path)
        x = path[:, 0]
        y = path[:, 1]
        z = path[:, 2]
        # Fit a spline to the data
        tck, u = splprep([x, y, z], s=0, per=False)
        # Define new range for the spline parameter
        u_new = np.linspace(u.min(), u.max(), 100)
        # Evaluate the spline at the new parameter values
        x_spline, y_spline, z_spline = splev(u_new, tck)
        path=np.array([x_spline,y_spline, z_spline]).T
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
                path=self.spline(path)  #  <-----------------------------------------
                self.path_to_coppeliasim(path)
                return path


            childern = self.successors(parent)


            for child in childern:

                child.h = dist(child.pos, self.end.pos)
                child.g = parent.g + dist(child.pos, parent.pos)
                child.f = child.g + child.h #+ (abs(child.pos[2]-self.end.pos[2])*10)
                flag=False

                for i, closed_child in enumerate(closed_list):
                    if child.pos == closed_child.pos:
                        flag=True
                        if child.f < closed_child.f:
                            closed_list[i] = child

                        break

                if flag:
                    continue

                for i, open_child in enumerate(open_list):
                    if child.pos == open_child.pos:
                        flag=True
                        if child.f < open_child.f:
                            open_list[i] = child

                        break

                if flag:
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
