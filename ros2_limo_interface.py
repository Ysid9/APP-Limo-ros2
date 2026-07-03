import math
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class LimoROS2Interface(Node):
    def __init__(self, safety_limits=False):
        super().__init__('limo_controller')
        self._safety_limits = safety_limits
        self._position = [0.0, 0.0, 0.0]
        self._lock = threading.Lock()
        self._odom_received = False

        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self._spin_thread.start()

        print('LimoROS2Interface ready — waiting for first /odom message...')
        while not self._odom_received:
            time.sleep(0.05)
        print('First /odom received.')

    def _odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        with self._lock:
            self._position = [x, y, theta]
            self._odom_received = True

    def get_position(self):
        with self._lock:
            return list(self._position)

    def set_cmd_vel(self, v_lin, v_lat, v_ang=0.0):
        if self._safety_limits:
            v_lin = max(-0.3, min(0.3, v_lin))
            v_lat = max(-0.3, min(0.3, v_lat))
            v_ang = max(-0.8, min(0.8, v_ang))
        msg = Twist()
        msg.linear.x  = float(v_lin)
        msg.linear.y  = float(v_lat)
        msg.angular.z = float(v_ang)
        self._cmd_pub.publish(msg)

    def cleanup(self):
        self.set_cmd_vel(0.0, 0.0, 0.0)
        self.destroy_node()
