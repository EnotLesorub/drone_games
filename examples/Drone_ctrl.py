#!/usr/bin/env python3
# coding=utf8

""" версия
21 ноя 19:07
изменения

1. Добавлены potential field
2. исправлены ошибки интерпретатора
3. ошибка 'global_position/local' НЕ ИСПРАВЛЕНА
4. функции potential field добавлены в другой файл. А скрипт переименован из PF_Planing в Drone_ctrl
5. Функция set_vel_xyz была заменена set_vel
6. PF_plan.sh также был изменен
7. Добавлено управление коптером по PF с чайками в виде препятствий. Вроде работает
8. Изменения из говнокода
9. Летаем по новым глобальным координатам целей (но локально т.к. gps еще нету). Сделали обновление номерац ели т.к. целей много.
10.
"""
import rospy
import time
import sys
import math
import numpy as np
import Potential_Field as PF
import convert_gps_local as convLoc

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix 
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState, ParamValue
from geographic_msgs.msg import GeoPointStamped

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome, ParamSet, ParamGet

# TODO избавиться от глобальных переменных, придумать как задавать лишь раз

takeoff_time = 40

bias_height = 20

goal_local = [[-30*5, 50*5, 100]]

goal_global = [[1.0046790641871626, 1.0009882945059274, 121.0],
               [1.0050389922015597, 1.0007187596801768, 131.0],
               [1.0047690461907620, 1.0003593798252748, 115.0],
               [1.0044091181763648, 1.0006289146596710, 105.0],
               [1.0041391721655668, 1.0002695348430380, 115.0],
               [1.0044991001799640, 1.0000000000000000, 105.0],
               [1.0048590281943612, 0.9997304651273407, 120.0],
               [1.0045890821835632, 0.9993710853230500, 115.0]]

instances_num = 1 #количество аппаратов
freq = 20 #Герц, частота посылки управляющих команд аппарату
node_name = "offboard_node"
data = {}
lz = {}

def subscribe_on_mavros_topics(suff, data_class):
  #подписываемся на Mavros топики всех аппаратов
  for n in range(1, instances_num + 1):
    data[n] = {}
    topic = f"/mavros{n}/{suff}"
    rospy.Subscriber(topic, data_class, topic_cb, callback_args = (n, suff))

def topic_cb(msg, callback_args):
  n, suff = callback_args
  data[n][suff] = msg

def service_proxy(n, path, arg_type, *args, **kwds):
  service = rospy.ServiceProxy(f"/mavros{n}/{path}", arg_type)
  ret = service(*args, **kwds)

  rospy.loginfo(f"{n}: {path} {args}, {kwds} => {ret}")
  return ret

def arming(n, to_arm):
  d = data[n].get("state")
  if d is not None and d.armed != to_arm:
    service_proxy(n, "cmd/arming", CommandBool, to_arm)

def set_mode(n, new_mode):
  d = data[n].get("state")
  if d is not None and d.mode != new_mode:
    service_proxy(n, "set_mode", SetMode, custom_mode=new_mode)

def set_param(n, name, i = 0, r = 0.0):
  return service_proxy(n, "param/set", ParamSet, name, ParamValue(i,r)).success

def get_param(n, name):
  ret = service_proxy(n, "param/get", ParamGet, param_id=name)

  v = None
  if ret.success:
    i = ret.value.integer
    r = ret.value.real
    if i != 0:
      v = i
    elif r != 0.0:
      v = r
    else:
      v = 0

  return v

def callback(_data):
  data[1].update({'ca_radar/rel_obstacles': _data.data})
  
def subscribe_on_topics():
  # глобальная (GPS) система координат
  # высота задана в элипсоиде WGS-84 и не равна высоте над уровнем моря, см. https://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level
  
  subscribe_on_mavros_topics("global_position/global", NavSatFix)
  subscribe_on_mavros_topics("global_position/local", Odometry)
  rospy.Subscriber("ca_radar/rel_obstacles", String, callback)
  #локальная система координат, точка отсчета = место включения аппарата
  subscribe_on_mavros_topics("local_position/pose", PoseStamped)
  subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped)

  #состояние
  subscribe_on_mavros_topics("state", State)
  subscribe_on_mavros_topics("extended_state", ExtendedState)
  

def on_shutdown_cb():
  rospy.logfatal("shutdown")

#ROS/Mavros работают в системе координат ENU(Восток-Север-Вверх), автопилот px4 и протокол сообщений Mavlink используют систему координат NED(Север-Восток-Вниз)
#см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED

#Управление по точкам, локальная система координат.
def set_pos(pt, x, y, z):
  pt.type_mask = pt.IGNORE_VX | pt.IGNORE_VY | pt.IGNORE_VZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

  #Смещение на восток
  pt.position.x = x
  #Смещение на север
  pt.position.y = y
  #Высота, направление вверх
  pt.position.z = z

#Управление по скоростям, локальная система координат, направления совпадают с оными в глобальной системе координат
def set_vel(pt, vx, vy, vz):
  pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

  #Скорость, направление на восток
  pt.velocity.x = vx
  #Скорость, направление на север
  pt.velocity.y = vy
  #Скорость, направление вверх
  pt.velocity.z = vz

# ------------------------------------(НАЧАЛО) функция парсинга стен (НАЧАЛО)-------------------------------------------  

def parcing_obs_xyz(Walls_Pos):
  Walls_Pos = list(map(float, Walls_Pos.split()))
  
  walls = []
  if len(Walls_Pos) > 1:
    for i in range(len(Walls_Pos)//5):
      walls.append([Walls_Pos[i * 5 + 2],
                    Walls_Pos[i * 5 + 3],
                    Walls_Pos[i * 5 + 4]])
  
  return walls
# ====================================(КОНЕЦ) функция парсинга стен (КОНЕЦ)===============================================


#взлет коптера
def pf_takeoff(pt, n, dt):
  if dt<takeoff_time:
    #скорость вверх
    set_vel(pt, 0, 0, 5)

    #армимся и взлетаем с заданной скоростью
    if dt>takeoff_time/2:
      arming(n, True)
  if dt > takeoff_time and dt < takeoff_time+1:
    set_vel(pt, 0, 0, 0)


def log_Position(pt):
  Position_log.write(str(data[1]) + '\n')
  Position_log.write('\n')

def get_q_xyz_gps(goal_number):
  xyz = [0, 0, 0]
  pos_gps_ = [0, 0, 0]
  pos_gps = data[1]["global_position/global"]
  pos_gps_[0] = pos_gps.latitude
  pos_gps_[1] = pos_gps.longitude
  pos_gps_[2] = pos_gps.altitude
  xyz = convLoc.enu_vector(pos_gps_, goal_global[goal_number])
  return xyz
  
  
def get_q_xyz_IMU():
  Drone_Pos_IMU = data[1]["local_position/pose"].pose.position
  qx = Drone_Pos_IMU.x
  qy = Drone_Pos_IMU.y
  qz = Drone_Pos_IMU.z
  return [qx, qy, qz]

def set_vel_to_goal(pt, goal_number, goal, Vel, copter_coord):
  vel_req = [0, 0, 0]
  for i in range(len(copter_coord)):
    if (copter_coord[i] <=  goal[goal_number][i] + 1) and (copter_coord[i] >= goal[goal_number][i] - 1):
      vel_req[i] = 0
    else:
      vel_req[i] = Vel[i]
      
  set_vel(pt, vel_req[0], vel_req[1], vel_req[2])
  return vel_req
  
def set_vel_to_goal_gps(pt, goal_dist, Vel):
  vel_req = [0, 0, 0]
  for i in range(len(goal_dist)):
    if (abs(goal_dist[i]) <= 1):
      vel_req[i] = 0
    else:
      vel_req[i] = Vel[i]
      
  set_vel(pt, vel_req[0], vel_req[1], vel_req[2])
  return vel_req

# ------------------------------------(НАЧАЛО) функция цикла управления (НАЧАЛО)-------------------------------------------  

#пример управления коптерами
def pf_example(pt, n, dt, goal_number):
  goal_dist = [0, 0, 0]
  Vel=[0,0,0]
  vel_req=[0,0,0]
  walls=[]
  pf_takeoff(pt, n, dt)
  
  #---------по gps координатам и расстоянию до цели------------
  # По факту мы преобразуем координаты gps в расстояние до цели. 
  # И подаём значение расстояния на вход адаптированным функциям расчета потенциальных полей и скорости.
  
  if dt > takeoff_time + 1:
    try:
      Walls_Pos_str = data[1]["ca_radar/rel_obstacles"]
      walls = parcing_obs_xyz(Walls_Pos_str)
    except Exception as error:
      print("the exception is: ", error)
    try:
      goal_dist = get_q_xyz_gps(goal_number)
      Vel = PF.call_PF_gps(goal_dist, walls)
      vel_req = set_vel_to_goal_gps(pt, goal_dist, Vel)
      if vel_req[0] == 0 and vel_req[1] == 0 and vel_req[2] == 0:
        goal_number = goal_number + 1
    except Exception as error:
      print("the exception is: ", error)

  print('GPS target:', goal_global[goal_number], goal_number)
  print('GPS copter:', goal_dist)
  print('vel_req:', vel_req)
  print('Vel:', Vel)
  print('walls:', len(walls))
  print('distance:', math.sqrt((goal_dist[0])**2 + (goal_dist[1])**2 + (goal_dist[2])**2))
  print('/------------/')
  return goal_number
  
# ====================================(КОНЕЦ) функция цикла управления (КОНЕЦ)===============================================

  
def offboard_loop():
  pub_pt = {}
  #создаем топики, для публикации управляющих значений
  for n in range(1, instances_num + 1):
    pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)
  
  pt = PositionTarget()
  #см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
  pt.coordinate_frame = pt.FRAME_LOCAL_NED
  
  t0 = time.time()
  goal_number = 0

  #цикл управления
  rate = rospy.Rate(freq)
  while not rospy.is_shutdown():
    dt = time.time() - t0

    #управляем каждым аппаратом централизованно
    for n in range(1, instances_num + 1):
      set_mode(n, "OFFBOARD")
      goal_number = pf_example(pt, n, dt, goal_number)
      pub_pt[n].publish(pt)

    rate.sleep()

if __name__ == '__main__':
    
  instances_num = 1
  rospy.init_node(node_name)
  rospy.loginfo(node_name + " started")

  subscribe_on_topics()

  rospy.on_shutdown(on_shutdown_cb)
  try:
    offboard_loop()
  except rospy.ROSInterruptException:
    pass

  rospy.spin()
