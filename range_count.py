#!/usr/bin/python3

import rospy
import actionlib
import tf
import yaml
import math
import time
import os

from market_teachpoint.msg import teach_pointAction, teach_pointFeedback, teach_pointResult
from irs_thermal_camera_ros.msg import RecordVideoAction, RecordVideoGoal
from irs_thermal_camera_ros.msg import CameraControl
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Int8

# 전역 변수
total_distance = 0.0
target_speed = 0.3  # 설정된 목표 속도 (전체 속도의 평균값값)
time_taken = 0.0  # 예상 이동 시간
pos_path_global = None  # 경로 저장 (Path 메시지 전체 저장)

def global_pth_cb(msg):
    global pos_path_global
    pos_path_global = msg  # msg 전체를 저장 (Path 메시지 전체)

def move_time_count():
    global total_distance, target_speed, time_taken
    if pos_path_global is None:
        rospy.logwarn("No path data available")
        return

    # 경로 상의 각 점들에 대해 거리 계산
    total_distance = 0.0  # 새로운 경로가 들어오면 이전 값을 초기화
    for i in range(1, len(pos_path_global.poses)):
        distance = calculate_distance(pos_path_global.poses[i-1].pose.position, pos_path_global.poses[i].pose.position)
        total_distance += distance
    rospy.loginfo("Total path distance: %.2f meters", total_distance)

    # 목표 지점까지의 예상 이동 시간 계산
    if target_speed > 0:
        time_taken = total_distance / target_speed
        rospy.loginfo("Estimated time to reach goal: %.2f seconds", time_taken)
    else:
        rospy.logwarn("Target speed is 0, cannot calculate time")

def calculate_distance(pos1, pos2):
    # 두 점 간의 거리 계산 (유클리드 거리)
    return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)

def movebase_client(x, y, z, w):
    global pos_path_global
    
    rospy.wait_for_service('/move_base/clear_costmaps')
    clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    clear_costmap()

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.w = w 

    client.send_goal(goal)
    now = rospy.Time.now()

    # 경로 길이를 20으로 설정, 길이가 20보다 적으면 대기 후 거리 계산
    while len(pos_path_global.poses) < 20:
        if rospy.Time.now().secs - now.secs > 2:
            move_time_count()  # 예상 시간 계산
            break
    else:
        pass
        #check_first()

    wait = client.wait_for_result()
    
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("MOVE DONE !!!!")
    
    return client.get_state()

if __name__ == "__main__":
    rospy.init_node("robot_navigation")
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, global_pth_cb)  # 경로 구독
    rospy.spin()
