#!/usr/bin/env python
 
import math

def calc_dist_pointsdist(point1,point2):
    return math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)

def compute_distance_point_to_polygon(obstacle,[result.pose_final.x,result.pose_final.y]):
    l=[]
    n=len(obstacle)
    for i in range(n):
        l.append(computeDistancePointToLine(obstacle[(i)%n],obstacle[(i+1)%n],q)) 
    return min(l)

def find_closest_distance(position, obstacle_list):
    results = [obs.compute_distance_point_to_polygon(position) for obs in obstacle_list]
    closest_obs = np.argmin([v[0] for v in results])
    return obstacle_list[closest_obs], results[closest_obs] 

def has_reached_goal(current_pos, goal, step_size): 
    if compute_distance_between_points(current_pos, goal) > step_size: 
        return False
    return True

def move_towards_goal(current_pos, goal_pos, step_size):
     direction_to_goal = get_direction_from_points (current_pos, goal_pos) 
     next_position = current_pos + step_size * direction_to_goal
     return next_position

def is_about_to_hit_obstacle (next_pos, obstacle_list, step_size): 
    _,(closest_obs distance, obst segment) = find_closest obstacle (next_pos, obstacle_list)
    if closest_obs_distance < step_size:  
      return True
    return False
