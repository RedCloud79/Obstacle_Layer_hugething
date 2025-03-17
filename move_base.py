import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import math

global pos_path_global
pos_path_global = []

class MoveBaseClient:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.current_linear_speed = 0.0
        self.previous_position = None
        self.last_movement_time = rospy.Time.now()
    
    def odom_callback(self, msg):
        self.current_linear_speed = msg.twist.twist.linear.x
        pos_path_global.append(msg.pose.pose)  # 위치 데이터를 리스트에 저장

    def calculate_distance(self, pose1, pose2):
        return math.sqrt((pose1.position.x - pose2.position.x) ** 2 + (pose1.position.y - pose2.position.y) ** 2)
    
    def movebase_client(self, x, y, z, w):
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
        
        while len(pos_path_global) < 20:  # pos_path_global을 리스트로 사용
            if rospy.Time.now().secs - now.secs > 2:
                break
        
        wait = client.wait_for_result()
        
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:         
            rospy.loginfo("MOVE DONE !!!!")
        
        return client.get_state()
    
    def check_progress(self, threshold_time=5, stop_threshold=0.05):
        if self.previous_position is None:
            self.previous_position = PoseStamped()
            self.previous_position.pose.position.x = 0
            self.previous_position.pose.position.y = 0
            return
        
        distance_moved = self.calculate_distance(self.previous_position.pose, PoseStamped().pose)
        if distance_moved > stop_threshold:
            self.last_movement_time = rospy.Time.now()
        elif rospy.Time.now().secs - self.last_movement_time.secs > threshold_time:
            rospy.logwarn("Robot is stuck!")
