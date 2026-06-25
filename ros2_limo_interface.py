import math
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros


class LimoROS2Interface(Node):
    def __init__(self, safety_limits=False):
        super().__init__('limo_controller')
        self._safety_limits = safety_limits
        self._position = [0.0, 0.0, 0.0]
        self._lock = threading.Lock()
        self._pose_ready = False

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._cmd_pub     = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pose_timer  = self.create_timer(0.02, self._update_pose)

        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self._spin_thread.start()

        print('LimoROS2Interface ready — en attente du TF map→base_link...')
        while not self._pose_ready:
            time.sleep(0.1)
        print('TF map→base_link disponible.')

    def _update_pose(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            theta = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
            )
            with self._lock:
                self._position = [x, y, theta]
                self._pose_ready = True
        except Exception:
            pass

    def get_position(self):
        with self._lock:
            return list(self._position)

    def set_cmd_vel(self, v_lin, v_ang):
        if self._safety_limits:
            v_lin = max(-0.5, min(0.5, v_lin))
            v_ang = max(-1.0, min(1.0, v_ang))
        msg = Twist()
        msg.linear.x  = float(v_lin)
        msg.angular.z = float(v_ang)
        self._cmd_pub.publish(msg)

    def cleanup(self):
        self.set_cmd_vel(0.0, 0.0)
        self.destroy_node()
