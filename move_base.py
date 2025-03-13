import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import time
import math

TIME_MARGIN = 0.3  # 30% 여유 시간
POSITION_TOLERANCE = 0.5  # 목표 지점 도착 허용 거리
YAW_TOLERANCE = 0.1  # 방향 허용 오차
PROGRESS_CHECK_TIME = 3.0  # 진행률 체크 주기 (초)
PROGRESS_THRESHOLD = 0.1  # 목표 접근 최소 거리 변화 (m)

current_x, current_y = 0, 0  # /robot_pose에서 가져올 값

def robot_pose_callback(msg):
    """로봇 현재 위치 업데이트"""
    global current_x, current_y
    current_x = msg.pose.position.x
    current_y = msg.pose.position.y

def robot_speed_callback(msg):
    """로봇 이동속도 추출 업데이트"""
    global 

def estimate_travel_time(distance, speed=0.5):
    """이동 거리와 속도를 기반으로 예상 이동 시간 계산"""
    return distance / speed

def has_reached_position(goal_x, goal_y):
    """목표 위치 근처에 도달했는지 확인 (경로 진행률 고려)"""
    global current_x, current_y

    # 일정 시간 간격으로 진행률 체크
    prev_distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
    rospy.sleep(PROGRESS_CHECK_TIME)
    new_distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

    # 일정 시간 동안 거리가 거의 변하지 않으면 "막힘"으로 판단
    return new_distance < POSITION_TOLERANCE and abs(new_distance - prev_distance) < PROGRESS_THRESHOLD

def send_goal(goal_x, goal_y):
    """move_base에 목표를 전송하고 도착 여부를 확인"""
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.orientation.w = 1.0  # 기본 방향

    rospy.loginfo(f"목표 지점 전송: x={goal_x}, y={goal_y}")
    
    start_time = time.time()
    client.send_goal(goal)

    # 예상 이동 시간 계산
    distance = math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
    expected_time = estimate_travel_time(distance)
    max_time = expected_time * (1 + TIME_MARGIN)  # 30% 여유 시간 적용

    while time.time() - start_time < max_time:
        if has_reached_position(goal_x, goal_y):  
            rospy.loginfo("목표 도착 확인!")
            return True  # 정상 도착

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("목표 도착 완료!")
            return True  

        rospy.sleep(0.5)  

    rospy.logwarn("시간 초과! 장애물로 인해 도착 실패, 다음 포인트로 이동")
    client.cancel_goal()
    return False  

# ROS 노드 실행
rospy.init_node("goal_progress_checker")
rospy.Subscriber("/robot_pose", PoseStamped, robot_pose_callback)
rospy.Subscriber("/imu/data", Imu, robot_speed_callback)

goal_x, goal_y = 5, 3  # YAML에서 가져온 목표 값 예제
if not send_goal(goal_x, goal_y):
    rospy.loginfo("다음 목표로 이동")
