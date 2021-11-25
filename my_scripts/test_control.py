#!/usr/bin/env python3
# coding=utf8

import rospy
import time
import sys
import math
import numpy as np
import Potential_Field as pf

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState, ParamValue, HilGPS
from geographic_msgs.msg import GeoPointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome, ParamSet, ParamGet

instances_num = 1 #количество аппаратов
freq = 40 #Герц, частота посылки управляющих команд аппарату
node_name = "offboard_node"
data = {}
lz = {}

form_num = 1
target_dest = np.zeros(6)

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
  data[1].update({'formations_generator/formation': _data.data})

def subscribe_on_topics():
  # глобальная (GPS) система координат
  # высота задана в элипсоиде WGS-84 и не равна высоте над уровнем моря, см. https://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level
  subscribe_on_mavros_topics("global_position/global", NavSatFix)
  subscribe_on_mavros_topics("global_position/local", Odometry)
  rospy.Subscriber("formations_generator/formation", String, callback)
  

  #локальная система координат, точка отсчета = место включения аппарата
  subscribe_on_mavros_topics("local_position/pose", PoseStamped)
  subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped)

  #состояние
  subscribe_on_mavros_topics("state", State)
  subscribe_on_mavros_topics("extended_state", ExtendedState)


def on_shutdown_cb():
  rospy.logfatal("shutdown")


err_prev = np.zeros((6, 3))
sum_err = np.zeros((6, 3))
d_t = 1/freq

def pid(data, n, drone_num):
  drone_num -= 1
  Kp = np.array([0.75, 0.75, 1], float) # 0 - x
  Kd = np.array([0.35, 0.35, 0.3], float) # 1 - y
  Ki = np.array([1.2, 1.2, 1], float) # 2 - z

  P = data * Kp[n]
  D = (data - err_prev[drone_num][n])/d_t * Kd[n]
  I = (data + err_prev[drone_num][n])*d_t * Ki[n]

  PID = P + I + D
  err_prev[drone_num][n] = data
  sum_err[drone_num][n] += data
  
  if PID > 18:
    PID = 18
  
  return PID
  

def set_pos_pid(pt, x, y, z, n):
  pos_loc = data[n]["local_position/pose"].pose.position
  loc_xyz = [pos_loc.x, pos_loc.y, pos_loc.z]
  
  obstacles = []
  
  for i in range(6):
    if i+1 == n:
      continue
    
    obs = data[i+1]["local_position/pose"].pose.position
    obs_xyz = [obs.x, obs.y, obs.z]
    obstacles.append(obs_xyz)

  v_rep = pf.calc_repulsive_optential(loc_xyz, obstacles)
  
  vx = pid(x - loc_xyz[0], 0, n) - v_rep[0]
  vy = pid(y - loc_xyz[1], 1, n) - v_rep[1]
  vz = pid(z - loc_xyz[2], 2, n) - v_rep[2]
  
  set_vel(pt, vx, vy, vz)
  

def read_form(target_num, n):
  target_point = np.zeros(3)
  
  try:
    n -= 1
    form = data[1]["formations_generator/formation"]
    form = form.split()
  
    if len(form) < 4:
      form = "f l 0.0 -67.0 5.0 0.0 -69.0 5.0 0.0 -71.0 5.0 0.0 -73.0 5.0 0.0 -75.0 5.0 0.0 -77.0 5.0".split()
    
    if target_num == 1:
      target_point[0] = float(form[2+3*n]) + 41.0
      target_point[1] = float(form[3+3*n]) - 72.0
      target_point[2] = float(form[4+3*n]) + 3.0
      
    elif target_num == 2:
      target_point[0] = float(form[2+3*n]) + 41.0
      target_point[1] = float(form[3+3*n]) + 72.0
      target_point[2] = float(form[4+3*n]) + 3.0
      
    elif target_num == 3:
      target_point[0] = float(form[2+3*n]) - 41.0
      target_point[1] = float(form[3+3*n]) + 72.0
      target_point[2] = float(form[4+3*n]) + 3.0
      
    elif target_num == 4:
      target_point[0] = float(form[2+3*n]) - 41.0
      target_point[1] = float(form[3+3*n]) - 72.0
      target_point[2] = float(form[4+3*n]) + 3.0
      
    elif target_num == 5:
      target_point[0] = float(form[2+3*n]) + 41.0
      target_point[1] = float(form[3+3*n]) - 72.0
      target_point[2] = float(form[4+3*n]) + 3.0
      
    elif target_num == 6:
      target_point[0] = float(form[2+3*n]) + 41.0
      target_point[1] = float(form[3+3*n]) + 72.0
      target_point[2] = float(form[4+3*n]) + 3.0
      
    elif target_num == 7:
      target_point[0] = float(form[2+3*n]) - 41.0
      target_point[1] = float(form[3+3*n]) + 72.0
      target_point[2] = float(form[4+3*n]) + 3.0
      
    elif target_num == 8:
      target_point[0] = float(form[2+3*n]) - 41.0
      target_point[1] = float(form[3+3*n]) - 72.0
      target_point[2] = float(form[4+3*n]) + 3.0
      
    else:
      target_point[0] = float(form[2+3*n])
      target_point[1] = float(form[3+3*n])
      target_point[2] = float(form[4+3*n])
      
  except Exception as error:
    print("error is: ", error)
  
  return(target_point)


#Управление по скоростям, локальная система координат, направления совпадают с оными в глобальной системе координат
def set_vel(pt, vx, vy, vz):
  pt.type_mask = pt.IGNORE_PX | pt.IGNORE_PY | pt.IGNORE_PZ | pt.IGNORE_AFX | pt.IGNORE_AFY | pt.IGNORE_AFZ | pt.IGNORE_YAW | pt.IGNORE_YAW_RATE

  #Скорость, направление на восток
  pt.velocity.x = vx
  #Скорость, направление на север
  pt.velocity.y = vy
  #Скорость, направление вверх
  pt.velocity.z = vz

#взлет коптера
def mc_takeoff(pt, n, dt):
  if dt<10:
    #скорость вверх
    set_vel(pt, 0, 0, 4)

    #армимся и взлетаем с заданной скоростью
    if dt>5:
      arming(n, True)


def tp_check(tp, drone_num):
  pos_loc = data[drone_num]["local_position/pose"].pose.position
  #print(tp)
  
  #print(pos_loc.x - tp[0], pos_loc.y - tp[1], pos_loc.z - tp[2])
  
  if abs(pos_loc.x - tp[0]) <= 0.4 and abs(pos_loc.y - tp[1]) <= 0.4 and abs(pos_loc.z - tp[2]) <= 0.15:
    return True
  else:
    return False


def mc_example(pt, n, dt):
  global form_num
  global target_dest
  mc_takeoff(pt, n, dt)
    
  if dt>10:
    target_point = read_form(form_num, n)
    set_pos_pid(pt, target_point[0], target_point[1], target_point[2], n)
    #print(target_point)
  
    if tp_check(target_point, n) == True:
      target_dest[n-1] = 1
    
    if target_dest[0] == 1 and target_dest[1] == 1 and target_dest[2] == 1 and target_dest[3] == 1 and target_dest[4] == 1 and target_dest[5] == 1:
      form_num += 1
      target_dest = [0, 0, 0, 0, 0, 0]
      
    #print(form_num)
    #print(target_dest)
  
  #снижаем на землю
  if dt>500:
    set_vel(pt, 0, 0, -1)


def offboard_loop():
  pub_pt = {}
  #создаем топики, для публикации управляющих значений
  for n in range(1, instances_num + 1):
    pub_pt[n] = rospy.Publisher(f"/mavros{n}/setpoint_raw/local", PositionTarget, queue_size=10)

  pt = PositionTarget()
  #см. также описание mavlink сообщения https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
  pt.coordinate_frame = pt.FRAME_LOCAL_NED

  t0 = time.time()
  
  #цикл управления
  rate = rospy.Rate(freq)
  while not rospy.is_shutdown():
    dt = time.time() - t0

    #управляем каждым аппаратом централизованно
    for n in range(1, instances_num + 1):
      set_mode(n, "OFFBOARD")
      mc_example(pt, n, dt)
      pub_pt[n].publish(pt)

    rate.sleep()

if __name__ == '__main__':
  if len(sys.argv) > 1:
    instances_num = int(sys.argv[1])

  rospy.init_node(node_name)
  rospy.loginfo(node_name + " started")

  subscribe_on_topics()

  rospy.on_shutdown(on_shutdown_cb)

  try:
    offboard_loop()
  except rospy.ROSInterruptException:
    pass

  rospy.spin()
