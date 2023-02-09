#!/usr/bin/env python3
#Standard Libraries
from importlib.resources import path
from pathlib import Path
from xxlimited import foo
import numpy as np
import yaml
import pygame
import time
import pygame_utils
import matplotlib.image as mpimg
from skimage.draw import circle_perimeter
from scipy.linalg import block_diag
from math import sin, cos, atan2

#Map Handling Functions
def load_map(filename):
    im = mpimg.imread("../maps/" + filename)
    im_np = np.array(im)  #Whitespace is true, black is false
    #im_np = np.logical_not(im_np)    
    return im_np

def load_map_yaml(filename):
    with open("../maps/" + filename, "r") as stream:
            map_settings_dict = yaml.safe_load(stream)
    return map_settings_dict

#Node for building a graph
class Node:
    def __init__(self, point, parent_id, cost):
        self.point = point # A 3 by 1 vector [x, y, theta]
        self.parent_id = parent_id # The parent node id that leads to this node (There should only every be one parent in RRT)
        self.cost = cost # The cost to come to this node
        self.children_ids = [] # The children node ids of this node
        return

#Path Planner 
class PathPlanner: 
    #A path planner capable of perfomring RRT and RRT*
    def __init__(self, map_filename, map_setings_filename, goal_point, stopping_dist):
        #Get map information
        self.occupancy_map = load_map(map_filename)
        self.map_shape = self.occupancy_map.shape
        self.map_settings_dict = load_map_yaml(map_setings_filename)

        #Get the metric bounds of the map
        self.bounds = np.zeros([2,2]) #m
        self.bounds[0, 0] = self.map_settings_dict["origin"][0]
        self.bounds[1, 0] = self.map_settings_dict["origin"][1]
        self.bounds[0, 1] = self.map_settings_dict["origin"][0] + self.map_shape[1] * self.map_settings_dict["resolution"]
        self.bounds[1, 1] = self.map_settings_dict["origin"][1] + self.map_shape[0] * self.map_settings_dict["resolution"]

        #Robot information
        self.robot_radius = 0.22 #m
        self.vel_max = 0.5 #m/s (Feel free to change!)
        self.rot_vel_max = 0.2 #rad/s (Feel free to change!)

        #Goal Parameters
        self.goal_point = goal_point #m
        self.stopping_dist = stopping_dist #m

        #Trajectory Simulation Parameters
        self.timestep = 1.0 #s
        self.num_substeps = 10

        #Planning storage
        self.nodes = [Node(np.zeros((3,1)), -1, 0)]

        #RRT* Specific Parameters
        self.lebesgue_free = np.sum(self.occupancy_map) * self.map_settings_dict["resolution"] **2
        self.zeta_d = np.pi
        self.gamma_RRT_star = 2 * (1 + 1/2) ** (1/2) * (self.lebesgue_free / self.zeta_d) ** (1/2)
        self.gamma_RRT = self.gamma_RRT_star + .1
        self.epsilon = 2.5
        
        #Pygame window for visualization
        self.window = pygame_utils.PygameWindow(
            "Path Planner", (1000, 1000), self.occupancy_map.shape, self.map_settings_dict, self.goal_point, self.stopping_dist)
        return

    #Functions required for RRT
    def sample_map_space(self):
        #Return an [x,y] coordinate to drive the robot towards
        print("TO DO: Sample point to drive towards")
        return np.zeros((2, 1))
    
    def check_if_duplicate(self, point):
        
        #Check if point is a duplicate of an already existing node
        print("TO DO: Check that nodes are not duplicates")
        return False
    
    def closest_node(self, point):
        #Returns the index of the closest node
        print("TO DO: Implement a method to get the closest node to a sapled point")
        return 0
    
    def simulate_trajectory(self, node_i, point_s):
        #Simulates the non-holonomic motion of the robot.
        #This function drives the robot from node_i towards point_s. This function does has many solutions!
        #node_i is a 3 by 1 vector [x;y;theta] this can be used to construct the SE(2) matrix T_{OI} in course notation
        #point_s is the sampled point vector [x; y]
        print("TO DO: Implment a method to simulate a trajectory given a sampled point")
        vel, rot_vel = self.robot_controller(node_i, point_s)

        robot_traj = self.trajectory_rollout(vel, rot_vel)
        return robot_traj
    
    def robot_controller(self, node_i, point_s):
        #This controller determines the velocities that will nominally move the robot from node i to node s
        #Max velocities should be enforced

        print("TO DO: Implement a control scheme to drive you towards the sampled point")
        return 0, 0
    
    def trajectory_rollout(self, vel, rot_vel, theta_i):
        # Given your chosen velocities determine the trajectory of the robot for your given timestep
        # The returned trajectory should be a series of points to check for collisions

        trajectory = np.array([[],[],[]])                          # initialize array
        t = np.array(range(self.num_substeps))/self.num_substeps
        print(t)

        if rot_vel == 0:
            x_I = [np.around((vel*t*np.cos(theta_i)),2)]
            y_I = [np.around((vel*t*np.sin(theta_i)),2)]
            theta_I = [np.zeros(self.num_substeps)]
        else:
            x_I = [np.around((vel/rot_vel)*(np.sin(rot_vel*t + theta_i)-np.sin(theta_i)), 2)]       # position in {V} frame
            y_I = [np.around((vel/rot_vel)*(np.cos(theta_i)-np.cos(rot_vel*t + theta_i)),2)]
            print("\nx_components: vel/rot_vel", vel/rot_vel, "np.sin(rot_vel*t)", np.sin(rot_vel*t), "-np.sin(theta):", -np.sin(theta_i))
            print("y_components: vel/rot_vel", vel/rot_vel, "np.cos(theta)", np.cos(theta_i), "-np.sin(theta):", -np.cos(rot_vel*t))

            theta_I = [np.around(rot_vel*t,2)]                          # orientation in {V}

        trajectory = np.vstack((x_I, y_I, theta_I))
        return trajectory
        

        #the trajectory can be approximated using the unicycle robot velocity model 
        # trajectory = np.zeros((3, self.num_substeps))
        # t = np.linspace(0, timestep, num_substeps+1) 

        # print("TO DO: Implement a way to rollout the controls chosen")
        # return np.zeros((3, self.num_substeps))

# Task 1 A: Collision Detection 
    def point_to_cell(self, point):
        #Convert a series of [x,y] points in the map to the indices for the corresponding cell in the occupancy map
        #point is a 2 by N matrix of points of interest

        x_B = point[0] - self.map_settings_dict["origin"][0] # both in meters 
        y_B = point[1] - self.map_settings_dict["origin"][1] 
        print('the x coordinate for B is: ', x_B)
        print('the y coordinate for B is: ', y_B)

        # need to convert to index by dividing by resolution (*1/0.05 = *20)
        height = self.map_shape[1]*self.map_settings_dict["resolution"]          # map height in meters
        width = self.map_shape[0]*self.map_settings_dict['resolution']
        print('the map size is: ', height, 'meters in height', width, 'meters in width')
        print('the grid size is', self.map_shape[1], 'number of grids in height', self.map_shape[0], 'number of grids in width')

        print('the resolution of the map is: ', self.map_settings_dict['resolution'])
        print('the maps shape is: ', self.map_shape)

        x_idx = (x_B/self.map_settings_dict["resolution"]).astype(int)
        y_idx = ((height-y_B)/self.map_settings_dict["resolution"]).astype(int)  # y_B is wrt bottom left, while y_idx is wrt top left
        
        print('x index is: ', x_idx)
        print('y index is: ', y_idx)
        
        point_idx = np.vstack((x_idx,y_idx))

        print("Implement a method to get the map cell the robot is currently occupying")
        return point_idx

# Task 1 B: Collision Detection 
    def points_to_robot_circle(self, points):
        #Convert a series of [x,y] points to robot map footprints for collision detection
        #Hint: The disk function is included to help you with this function

        # start by converting into occupancy grid: 
        points_idx = self.point_to_cell(points) 
        pixel_radius = self.robot_radius/self.map_settings_dict["resolution"]  # robot radius in pixels
        footprint = [[],[]]

        for i in range(len(points_idx[0])):
            # rr, cc indicate the pixel coordinate of the points, can be directly used into an array like: img[rr,cc]
            rr, cc = circle_perimeter(points_idx[0, i], points_idx[1, i], int(pixel_radius))
            footprint = np.hstack((footprint,np.vstack((rr,cc))))

        print("Implement a method to get the pixel locations of the robot path")
        return footprint
    #Note: If you have correctly completed all previous functions, then you should be able to create a working RRT function

    #RRT* specific functions
    def ball_radius(self):
        #Close neighbor distance
        card_V = len(self.nodes)
        return min(self.gamma_RRT * (np.log(card_V) / card_V ) ** (1.0/2.0), self.epsilon)
    
    def connect_node_to_point(self, node_i, point_f):
        #Given two nodes find the non-holonomic path that connects them
        #Settings
        #node is a 3 by 1 node
        #point is a 2 by 1 point
        print("TO DO: Implement a way to connect two already existing nodes (for rewiring).")
        return np.zeros((3, self.num_substeps))
    
    def cost_to_come(self, trajectory_o):
        #The cost to get to a node from lavalle 
        print("TO DO: Implement a cost to come metric")
        return 0
    
    def update_children(self, node_id):
        #Given a node_id with a changed cost, update all connected nodes with the new cost
        print("TO DO: Update the costs of connected nodes after rewiring.")
        return

    #Planner Functions
    def rrt_planning(self):
        #This function performs RRT on the given map and robot
        #You do not need to demonstrate this function to the TAs, but it is left in for you to check your work
        for i in range(1): #Most likely need more iterations than this to complete the map!
            #Sample map space
            point = self.sample_map_space()

            #Get the closest point
            closest_node_id = self.closest_node(point)

            #Simulate driving the robot towards the closest point
            trajectory_o = self.simulate_trajectory(self.nodes[closest_node_id].point, point)

            #Check for collisions
            print("TO DO: Check for collisions and add safe points to list of nodes.")
            
            #Check if goal has been reached
            print("TO DO: Check if at goal point.")
        return self.nodes
    
    def rrt_star_planning(self):
        #This function performs RRT* for the given map and robot        
        for i in range(1): #Most likely need more iterations than this to complete the map!
            #Sample
            point = self.sample_map_space()

            #Closest Node
            closest_node_id = self.closest_node(point)

            #Simulate trajectory
            trajectory_o = self.simulate_trajectory(self.nodes[closest_node_id].point, point)

            #Check for Collision
            print("TO DO: Check for collision.")

            #Last node rewire
            print("TO DO: Last node rewiring")

            #Close node rewire
            print("TO DO: Near point rewiring")

            #Check for early end
            print("TO DO: Check for early end") 
        return self.nodes
    
    def recover_path(self, node_id = -1):
        path = [self.nodes[node_id].point]
        current_node_id = self.nodes[node_id].parent_id
        while current_node_id > -1:
            path.append(self.nodes[current_node_id].point)
            current_node_id = self.nodes[current_node_id].parent_id
        path.reverse()
        return path

def main():
    #Set map information
    map_filename = "willowgarageworld_05res.png"
    map_setings_filename = "willowgarageworld_05res.yaml"

    #robot information
    goal_point = np.array([[10], [10]]) #m
    stopping_dist = 0.5 #m

    #RRT precursor
    path_planner = PathPlanner(map_filename, map_setings_filename, goal_point, stopping_dist)
    nodes = path_planner.rrt_star_planning()
    node_path_metric = np.hstack(path_planner.recover_path())


    #Task 1A test: point_to_cell function 
    print("Task 1A test: point_to_cell function")
    point = np.array([[10, 5, -10, 0, 10, -5],
                      [10, 5, -10, 0, -5, 10]])
    print(path_planner.point_to_cell(point))
    
    for i in point.T:
        # print(point.T)
        path_planner.window.add_point(i, radius=10, color=(100, 0, 255))

    #Task 1B test: point_to_robot_circle function 
    print("/n Task 1B test: points_to_robot_circle function")
    footprint = path_planner.points_to_robot_circle(point)
    
    print(footprint)

    for i in footprint.T:
        path_planner.window.add_point(i, radius=10, color=(100, 0, 255))


    # Task 2A test: trajectory_rollout function
    print("\nTask 2A test: trajectory_rollout function")
    traj_rollout = path_planner.trajectory_rollout(8,0.4)
    print("trajectory_rollout:", traj_rollout)
    for i, val in enumerate(traj_rollout.T):
        if i%2==0:
            continue
        path_planner.window.add_se2_pose(val, length=8, color=(0,0,255))


    #Leftover test functions
    np.save("shortest_path.npy", node_path_metric)

    # open the file for visilization to check the correntness of the map: 
    while 1:
        data = np.load('shortest_path.npy')




if __name__ == '__main__':
    main()