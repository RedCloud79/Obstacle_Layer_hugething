import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class IMUToSpeed:
    def __init__(self):
        self.prev_time = rospy.Time.now()
        self.velocity = Twist()

        # X, Y, Z 축의 속도 초기화
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # IMU 데이터 구독
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback)

    def imu_callback(self, imu_data):
        # 시간 차이 계산
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time

        # 가속도 데이터 (m/s^2) 추출
        ax = imu_data.linear_acceleration.x
        ay = imu_data.linear_acceleration.y
        az = imu_data.linear_acceleration.z

        # 가속도를 적분하여 속도 계산 (속도는 가속도 * 시간)
        self.vx += ax * dt
        self.vy += ay * dt
        self.vz += az * dt

        # 속도 값 출력
        rospy.loginfo(f"Velocity: vx={self.vx}, vy={self.vy}, vz={self.vz}")

if __name__ == "__main__":
    rospy.init_node('imu_to_speed')
    imu_to_speed = IMUToSpeed()
    rospy.spin()


import rospy
from nav_msgs.msg import Odometry

class OdometryToSpeed:
    def __init__(self):
        # Odometry 데이터 구독
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, odom_data):
        # 선속도 정보 추출
        linear_velocity = odom_data.twist.twist.linear
        vx = linear_velocity.x
        vy = linear_velocity.y
        vz = linear_velocity.z

        # 속도 출력
        rospy.loginfo(f"Velocity: vx={vx}, vy={vy}, vz={vz}")

if __name__ == "__main__":
    rospy.init_node('odom_to_speed')
    odom_to_speed = OdometryToSpeed()
    rospy.spin()
