import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class PathLengthCalculator:
    def __init__(self):
        self.path_length = 0.0
        self.pos_path_global = Path()

        # 경로 구독
        self.path_sub = rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.path_callback)

    def global_pth_cb(self, msg):
        self.pos_path_global = msg
        self.pos_path_global = Path()

    def path_callback(self, path):
        # 경로 점들(Pose) 간의 거리 계산
        total_length = 0.0
        for i in range(1, len(self.pos_path_global.poses)):
            p1 = path.poses[i-1].pose.position
            p2 = path.poses[i].pose.position

            # 두 점 사이의 유클리드 거리 계산
            dist = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            total_length += dist

        self.path_length = total_length
        rospy.loginfo(f"Total Path Length: {self.path_length} meters")

if __name__ == "__main__":
    rospy.init_node('path_length_calculator')
    path_calculator = PathLengthCalculator()
    rospy.spin()
