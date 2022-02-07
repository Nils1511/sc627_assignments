#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib

#import other helper files if any
from helper import *

rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

#read input file
#path_name = open("~/catkin_ws/src/assignment_1/input.txt","r")
#k=[]


step_size=0.1
#setting result as initial location
result = MoveXYResult()

result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0 #in radians (0 to 2pi)


goal_pos = [5,3]

obstacle1=[[1,2],[1,0],[3,0]]
obstacle2=[[2,3],[4,1],[5,2]]
#node1.close()
#node2 = open("output_base.txt","w")#write mode

while step_size > calc_dist_points(goal,[result.pose_final.x,result.pose_final.y]) : #replace true with termination condition

    #determine waypoint based on your algo
    #this is a dummy waypoint (replace the part below)
    if step_size < min(compute_distance_point_to_polygon(obstacle1,[result.pose_final.x,result.pose_final.y]), compute_distance_point_to_polygon(obstacle2,[result.pose_final.x,result.pose_final.y]) ):
        wp = MoveXYGoal()
        wp.pose_dest.x = step_size*3/math.sqrt(34) + result.pose_final.x
        wp.pose_dest.y = step_size*5/math.sqrt(34)+result.pose_final.x
        wp.pose_dest.theta = math.atan(3/5) #theta is the orientation of robot in radians (0 to 2pi)

        #send waypoint to turtlebot3 via move_xy server
        client.send_goal(wp)

        client.wait_for_result()

        #getting updated robot location
        result = client.get_result()



        #node2.write(result.pose_final.x,result.pose_final.y)
    
        #write to output file (replacing the part below)
        print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)
    