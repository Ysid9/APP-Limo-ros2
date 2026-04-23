"""
ros2_limo_simulation.py
========================
Interface ROS2 pour le robot LIMO en mode différentiel 4WD.
Remplacement direct de zmq_pioneer_simulation.py (CoppeliaSim/ZMQ).

Interface identique à ZMQPioneerSimulation utilisée par torch_online.py :
    .r              — rayon des roues  [m]
    .R              — demi-voie        [m]
    .gain           — gain moteur
    get_position()          → [x, y, theta]
    set_motor_velocity(u)   → convertit [v_gauche, v_droite] en Twist
    cleanup()               → stoppe le robot, détruit le nœud ROS2

Cinématique différentielle (LIMO 4WD) :
    v_linéaire  = r · (v_droite + v_gauche) / 2
    v_angulaire = r · (v_droite − v_gauche) / (2·R)

Topic odométrie :
    Gazebo  (limo_ros2 / diff_drive plugin) : '/odometry'
    Robot réel (limo_bringup)               : '/odom'
    ⇒ À choisir dans ros2_torch_run.py via le paramètre ODOM_TOPIC.

Auteur  : Yasser — stage AMARSMER, CERV/LabSTICC, ENIB
Date    : 2026
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


# ═══════════════════════════════════════════════════════════════════════
# Paramètres cinématiques du LIMO — mode différentiel 4WD
# ═══════════════════════════════════════════════════════════════════════
WHEEL_RADIUS = 0.045      # r  [m]  rayon des roues
HALF_TRACK   = 0.1725     # R  [m]  demi-voie (voie complète = 0.345 m)
GAIN         = 5.0        # gain appliqué aux commandes (idem Pioneer)

def to_rad(deg):
    return 2*math.pi*deg/360

def to_deg(rad):
    return rad*360/(2*math.pi)

class ROS2LimoSimulation:
    """
    Nœud ROS2 minimal pour contrôler le LIMO (Gazebo ou réel).

    Expose les mêmes attributs et méthodes que ZMQPioneerSimulation
    afin que torch_online.py fonctionne sans modification.
    """

    def __init__(self,
                 cmd_vel_topic: str = '/cmd_vel',
                 odom_topic: str = '/odom',
                 node_name: str = 'limo_nn_controller'):
        """
        Args:
            cmd_vel_topic : topic de commande moteur (Twist)
            odom_topic    : topic d'odométrie (Odometry)
            node_name     : nom du nœud ROS2
        """
        # --- Paramètres cinématiques (utilisés par torch_online.py) ---
        self.r    = WHEEL_RADIUS
        self.R    = HALF_TRACK
        self.gain = GAIN

        # --- État interne ---
        self._position      = [2.5, 2.5, to_rad(45)]   # [x, y, theta]
        self._odom_received = False
        self._lock          = threading.Lock()

        # --- Initialisation ROS2 ---
        if not rclpy.ok():
            rclpy.init()

        self._node = Node(node_name)

        # Publisher : commandes moteur
        self._cmd_pub = self._node.create_publisher(Twist, cmd_vel_topic, 10)

        # Subscriber : odométrie
        self._odom_sub = self._node.create_subscription(
            Odometry, odom_topic, self._odom_callback, 10
        )

        # Thread de spin (daemon → s'arrête avec le programme)
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

        # Attendre la première mesure d'odométrie
        print(f'[ROS2LimoSimulation] Attente odométrie sur "{odom_topic}"...')
        t0 = time.time()
        while time.time() - t0 < 10.0:
            with self._lock:
                if self._odom_received:
                    break
            time.sleep(0.05)
        else:
            print(
                f'[ROS2LimoSimulation] ⚠️  Pas d\'odométrie reçue sur "{odom_topic}" '
                'après 10 s.\n'
                '   → Gazebo / limo_bringup lancé ?  ROS_DOMAIN_ID identique ?'
            )

        print('[ROS2LimoSimulation] ✅ Prêt.')

    # ──────────────────────────────────────────────────────────────────
    # Callback odométrie
    # ──────────────────────────────────────────────────────────────────
    def _odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        with self._lock:
            self._position     = [x, y, yaw]
            self._odom_received = True

    # ──────────────────────────────────────────────────────────────────
    # Boucle de spin ROS2 (thread daemon)
    # ──────────────────────────────────────────────────────────────────
    def _spin_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.01)

    # ══════════════════════════════════════════════════════════════════
    # Interface publique — identique à ZMQPioneerSimulation
    # ══════════════════════════════════════════════════════════════════

    def get_position(self) -> list:
        """Retourne [x (m), y (m), theta (rad)] depuis l'odométrie."""
        with self._lock:
            return list(self._position)

    def set_motor_velocity(self, control: list):
        """
        Commande les moteurs à partir de vitesses de roues [rad/s].

        Reçoit la sortie du réseau de neurones [v_gauche, v_droite],
        applique le gain, puis convertit en message Twist via la
        cinématique différentielle :

            v_linéaire  = r · (v_droite + v_gauche) / 2
            v_angulaire = r · (v_droite − v_gauche) / (2·R)

        Args:
            control : [v_gauche, v_droite]  (sortie brute du réseau)
        """
        v_gauche = self.gain * control[0]
        v_droite = self.gain * control[1]

        v_lineaire  = self.r * (v_droite + v_gauche) / 2.0
        v_angulaire = self.r * (v_droite - v_gauche) / (2.0 * self.R)

        twist = Twist()
        twist.linear.x  = float(v_lineaire)
        twist.angular.z = float(v_angulaire)

        # twist = Twist()
        # twist.linear.x  = float(self.gain * control[0])   
        # twist.angular.z = float(self.gain * control[1])   

        self._cmd_pub.publish(twist)

    def cleanup(self):
        """Stoppe le robot et détruit le nœud ROS2."""
        self.set_motor_velocity([0.0, 0.0])
        time.sleep(0.1)
        self._node.destroy_node()
        print('[ROS2LimoSimulation] Nœud ROS2 nettoyé.')
