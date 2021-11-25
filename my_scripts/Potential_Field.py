import math
import numpy as np

# в метрах
actual_robot_radius = 1.0

# TODO менять если сенсор не поддерживает такой большой дальности
search_radius = 50.0

# TODO Если робот будет реагировать на слишком далёкие препятствия (перед этим проверить логи препятствий)
# C Увеличением robot_radius_PF сила отталкивания уменьшается
robot_radius_PF = actual_robot_radius + 0.3


max_vel = [5.0, 5.0, 5.0]

positive_scaling_factor = 7.0


# ------------------------------------(НАЧАЛО) ФУНКЦИИ ПРЕОДОЛЕНИЯ ПРЕПЯТСТВИЙ LOCAL (НАЧАЛО)-------------------------------------------  


def calc_attractive_potential(coord, goal_local, goal_number):
  diff = [goal_local[goal_number][0] - coord[0], goal_local[goal_number][1] - coord[1], goal_local[goal_number][2] - coord[2]]
  diff = [0.5 * positive_scaling_factor * i for i in diff]
  return diff

def calc_repulsive_optential(coord, obstacles):
  diff_sq = lambda x, y: (x[0] - y[0])**2 + (x[1] - y[1])**2 + (x[2] - y[2])**2
  
  min_obstacle = [search_radius, search_radius, search_radius]
  min_distance = diff_sq(min_obstacle, coord)
  Vel_repulsive = [0, 0, 0]
  dist = [0, 0, 0]
  temp = [0, 0, 0]
  
  for obst in obstacles:
    if math.sqrt(diff_sq(obst, coord)) <= min_distance:
      min_obstacle = obst
      min_distance = math.sqrt(diff_sq(min_obstacle, coord))
    if min_distance <= robot_radius_PF:
      for i in range(len(coord)):
        dist[i] = min_obstacle[i] - coord[i]
        #чтобы избежать деления на ноль
        if dist[i] == 0:
          dist[i] = 1/max_vel[i]
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
      Vel_repulsive[i] = max_vel[i] * Vel_repulsive[i]/normal
  
  print(Vel_repulsive)
    
  return Vel_repulsive
  
  #TODO добавить rostopic ca_radar (информация о положении коптера), и распарсить его.
def calc_potential_field(copter_coord, goal_local, goal_number, walls):
  Vel = [0, 0, 0]
  
  Vel_Attractive = calc_attractive_potential(copter_coord, goal_local, goal_number)
  Vel_Repulsive = calc_repulsive_optential(copter_coord, walls)
  
  # нормируем вектор скорости
  # TODO заменить normal = sqrt(...) на normal.hyport
  normal = [0, 0, 0]
  for i in range(len(copter_coord)):
    normal[i] = Vel_Attractive[i] - Vel_Repulsive[i]
    
  normal = math.sqrt(normal[0]**2 + normal[1]**2 + normal[2]**2)
  
  for i in range(len(copter_coord)): 
    Vel[i] = abs(Vel_Attractive[i] - Vel_Repulsive[i])
    
    if abs(Vel[i]) >= abs(max_vel[i]):
      Vel[i] = Vel_Attractive[i] - Vel_Repulsive[i]
      Vel[i] = max_vel[i]*Vel[i]/normal
    else:
      Vel[i] = Vel_Attractive[i] - Vel_Repulsive[i]
      
  return Vel
  
# ==========================================(КОНЕЦ) ФУНКЦИИ ПРЕОДОЛЕНИЯ ПРЕПЯТСТВИЙ LOCAL(КОНЕЦ)========================================


# ------------------------------------(НАЧАЛО) ФУНКЦИИ ПРЕОДОЛЕНИЯ ПРЕПЯТСТВИЙ gps (НАЧАЛО)-------------------------------------------  

# Функция calc_attractive_potential_gps() запускается только из calc_potential_field_gps()
def calc_attractive_potential_gps(goal_gps):
  Vel_attractive = [0.5 * positive_scaling_factor * i for i in goal_gps]
  return Vel_attractive

# Функция calc_repulsive_optential_gps() запускается только из calc_potential_field_gps()
def calc_repulsive_optential_gps(obstacles):

  # инициализация переменных
  Vel_repulsive = [0, 0, 0]
  dist = [0, 0, 0]
  temp = [0, 0, 0]
  min_obstacle = [search_radius, search_radius, search_radius]
  
  # проверка на пустоту списка препятствий
  if len(obstacles) == 1:
    if len(obstacles[0]) == 0: 
      Vel_repulsive = [0.0, 0.0, 0.0]
      return Vel_repulsive
  elif len(obstacles) == 0:
    Vel_repulsive = [0.0, 0.0, 0.0]
    return Vel_repulsive 
  
  # нахождение координат наиболее близкой точки препятствия и расстояния до неё
  min_distance = math.sqrt(min_obstacle[0]**2 + min_obstacle[1]**2 + min_obstacle[2]**2)
  for obst in obstacles:
    obst_dist = math.sqrt(obst[0]**2 + obst[1]**2 + obst[2]**2)
    if obst_dist <= min_distance:
      min_obstacle = obst.copy()
      min_distance = obst_dist
  
  # Расчет отталкивающей скорости
  if min_distance <= robot_radius_PF:
    for i in range(len(obstacles)):
      dist[i] = min_obstacle[i]
      #чтобы избежать деления на ноль
      if dist[i] == 0:
        dist[i] = 1/max_vel[i]
      temp[i] = (1/dist[i]) - (1/robot_radius_PF)
      Vel_repulsive[i] = positive_scaling_factor * temp[i] * 1/(dist[i]**2)
  else:
    Vel_repulsive = [0.0, 0.0, 0.0]
    
  normal = [0, 0, 0]
  for i in range(len(goal_gps)):
    normal[i] = Vel_Repulsive[i]
  normal = math.sqrt(normal[0]**2 + normal[1]**2 + normal[2]**2)
    
  for i in range(3):  
    Vel_Repulsive[i] = max_vel[i] * Vel_Repulsive[i]/normal
  
  return Vel_repulsive
  
# На выходе функции скорость, а не сила. Сила якобы перерасчитывается в скорость домножением на силу 1
def calc_potential_field_gps(goal_gps, walls):
  Vel = [0, 0, 0]
  
  Vel_Attractive = calc_attractive_potential_gps(goal_gps)
  Vel_Repulsive = calc_repulsive_optential_gps(walls)
  
  # нормируем вектор скорости
  normal = [0, 0, 0]
  for i in range(len(goal_gps)):
    normal[i] = Vel_Attractive[i] - Vel_Repulsive[i]
  normal = math.sqrt(normal[0]**2 + normal[1]**2 + normal[2]**2)

  # Вычисляем скорость по разнице притягивающей и отталкивающей сил и отношению к нормали.
  for i in range(len(goal_gps)): 
    Vel[i] = abs(Vel_Attractive[i] - Vel_Repulsive[i])
    
    if abs(Vel[i]) >= abs(max_vel[i]):
      Vel[i] = Vel_Attractive[i] - Vel_Repulsive[i]
      Vel[i] = max_vel[i]*Vel[i]/normal
    else:
      Vel[i] = Vel_Attractive[i] - Vel_Repulsive[i]
      
  return Vel
  
# ==========================================(КОНЕЦ) ФУНКЦИИ ПРЕОДОЛЕНИЯ ПРЕПЯТСТВИЙ gps(КОНЕЦ)========================================

def call_PF(copter_coord, goal_local, goal_number, walls,  max_vel = [5.0, 5.0, 5.0], act_rob_rad = 1.0, search_radius = 10.0):
  actual_robot_radius = act_rob_rad
  Vel = calc_potential_field(copter_coord, goal_local, goal_number, walls)
  return Vel

# скорость, возвращаемую этой функцией можно преобразовывать, изменяя max_vel в глобальных константах файла
# или как-то по своему
def call_PF_gps(goal_dist, walls,  max_vel = [5.0, 5.0, 5.0], act_rob_rad = 1.0, search_radius = 10.0):
  actual_robot_radius = act_rob_rad
  Vel = calc_potential_field_gps(goal_dist, walls)
  return Vel
