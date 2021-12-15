import math
import numpy as np


actual_robot_radius = 1.0

search_radius = 50.0
robot_radius_PF = actual_robot_radius + 1.0


max_vel = [8.5, 8.5, 8.5]
max_vel_drones = [5.0, 5.0, 8.0]

positive_scaling_factor = 7.0


def enu_vector(g1, g2):
    n = g2[0] - g1[0]
    e = g2[1] - g1[1]
    u = g2[2] - g1[2]

    refLat = (g1[0]+g2[0])/2

    nm = n * 333400 / 3  # deltaNorth * 40008000 / 360
    em = e * 1001879 * math.cos(math.radians(refLat)) / 9  # deltaEast * 40075160 *cos(refLatitude) / 360

    return [em, nm, u]


def calc_repulsive_optential(coord, obstacles):
  diff_sq = lambda x, y: (x[0] - y[0])**2 + (x[1] - y[1])**2 + (x[2] - y[2])**2
  
  min_obstacle = [search_radius, search_radius, search_radius]
  min_distance = 15
  obst_rep_vel = []
  Vel_repulsive = [0, 0, 0]
  dist = [0, 0, 0]
  temp = [0, 0, 0]
  
  for obst in obstacles:
    if math.sqrt(diff_sq(obst, coord)) <= min_distance:
      min_obstacle = obst

      for i in range(len(coord)):
        dist[i] = min_obstacle[i] - coord[i]
        #чтобы избежать деления на ноль
        if dist[i] == 0:
          dist[i] = 1/max_vel[i]
        temp[i] = (1/dist[i]) - (1/robot_radius_PF)
        Vel_repulsive[i] = positive_scaling_factor * temp[i] * 1/(dist[i]**2)
        
      obst_rep_vel.append(Vel_repulsive)
  
  Vel_repulsive = [0, 0, 0]
  for i in range(len(obst_rep_vel)):
    Vel_repulsive[0] += obst_rep_vel[i][0]
    Vel_repulsive[1] += obst_rep_vel[i][1]
    Vel_repulsive[2] += obst_rep_vel[i][2]
  
  normal = [0, 0, 0]
  for i in range(len(coord)):
    normal[i] = Vel_repulsive[i]
  normal = math.sqrt(normal[0]**2 + normal[1]**2 + normal[2]**2)
    
  for i in range(3):
    if normal == 0:
      Vel_repulsive = [0, 0, 0]
      break
    else:
      Vel_repulsive[i] = max_vel[i] * Vel_repulsive[i]/normal
    
  return Vel_repulsive
  

def calc_repulsive_optential_gps(coord, obstacles):
  diff_sq = lambda x: (x[0])**2 + (x[1])**2 + (x[2])**2
  
  min_obstacle = [search_radius, search_radius, search_radius]
  min_distance = 15
  Vel_repulsive = [0, 0, 0]
  dist = [0, 0, 0]
  temp = [0, 0, 0]
  
  for obst in obstacles:
    if math.sqrt(diff_sq(enu_vector(obst, coord))) <= min_distance:
      min_obstacle = obst
      min_distance = math.sqrt(diff_sq(enu_vector(obst, coord)))
    if min_distance <= robot_radius_PF:
      for i in range(len(coord)):
        dist[i] = enu_vector(coord, min_obstacle)[i]
        #чтобы избежать деления на ноль
        if dist[i] == 0:
          dist[i] = 1/max_vel_drones[i]
        temp[i] = (1/dist[i]) - (1/robot_radius_PF)
        Vel_repulsive[i] = positive_scaling_factor * temp[i] * 1/(dist[i]**2)
    else:
      Vel_repulsive = [0.0, 0.0, 0.0]
    
  normal = [0, 0, 0]
  for i in range(len(coord)):
    normal[i] = Vel_repulsive[i]
  normal = math.sqrt(normal[0]**2 + normal[1]**2 + normal[2]**2)
    
  for i in range(3):
    if normal == 0:
      Vel_repulsive = [0, 0, 0]
      break
    else:
      Vel_repulsive[i] = max_vel_drones[i] * Vel_repulsive[i]/normal
      
  return Vel_repulsive
  
