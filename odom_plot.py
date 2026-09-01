#!/usr/bin/env python3
"""
odom_plot.py
============
Visualisation en temps réel de l'odométrie du LIMO.
Affiche la trajectoire et l'orientation du robot sur un plan 2D.

Usage :
    python3 odom_plot.py
    python3 odom_plot.py --topic /odom
    python3 odom_plot.py --topic /wheel/odom
    python3 odom_plot.py --history 500
"""

import argparse
import math
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from collections import deque


class OdomPlotter(Node):

    def __init__(self, topic, history):
        super().__init__('odom_plotter')
        self._lock = threading.Lock()
        self._xs = deque(maxlen=history)
        self._ys = deque(maxlen=history)
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._received = False

        self.create_subscription(Odometry, topic, self._cb, 10)
        self.get_logger().info(f'Écoute sur {topic}...')

    def _cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        )
        with self._lock:
            self._x = x
            self._y = y
            self._theta = theta
            self._xs.append(x)
            self._ys.append(y)
            self._received = True

    def get_state(self):
        with self._lock:
            return (
                list(self._xs),
                list(self._ys),
                self._x,
                self._y,
                self._theta,
                self._received,
            )


def main():
    parser = argparse.ArgumentParser(description='Visualisation odométrie LIMO')
    parser.add_argument('--topic',   default='/odom',
                        help='Topic odométrie (défaut: /odom)')
    parser.add_argument('--history', type=int, default=2000,
                        help='Nombre de points conservés (défaut: 2000)')
    args = parser.parse_args()

    rclpy.init()
    node = OdomPlotter(args.topic, args.history)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # --- Figure ---
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title('LIMO — Odométrie temps réel')
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'Trajectoire — {args.topic}')

    traj_line,   = ax.plot([], [], 'b-', linewidth=1.0, alpha=0.6, label='Trajectoire')
    origin_dot,  = ax.plot([0], [0], 'g+', markersize=14, markeredgewidth=2, label='Origine')
    pos_dot,     = ax.plot([], [], 'ro', markersize=6, label='Position')
    arrow = None
    info_text = ax.text(0.02, 0.97, '', transform=ax.transAxes,
                        verticalalignment='top', fontsize=9,
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

    ARROW_LEN = 0.08
    VIEW_MARGIN = 0.5    # marge autour de la trajectoire (m)
    VIEW_MIN = 1.0       # fenêtre minimale (m)

    def update(_frame):
        nonlocal arrow
        xs, ys, x, y, theta, received = node.get_state()

        if not received:
            info_text.set_text('En attente de messages...')
            return traj_line, pos_dot, info_text

        # Trajectoire
        traj_line.set_data(xs, ys)

        # Position courante
        pos_dot.set_data([x], [y])

        # Flèche d'orientation
        if arrow is not None:
            arrow.remove()
        arrow = ax.annotate(
            '', xy=(x + ARROW_LEN * math.cos(theta),
                    y + ARROW_LEN * math.sin(theta)),
            xytext=(x, y),
            arrowprops=dict(arrowstyle='->', color='red', lw=2)
        )

        # Ajuster la vue
        all_x = xs + [0.0]
        all_y = ys + [0.0]
        xmin, xmax = min(all_x) - VIEW_MARGIN, max(all_x) + VIEW_MARGIN
        ymin, ymax = min(all_y) - VIEW_MARGIN, max(all_y) + VIEW_MARGIN
        cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
        half = max((xmax - xmin) / 2, (ymax - ymin) / 2, VIEW_MIN / 2)
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)

        # Texte info
        info_text.set_text(
            f'x = {x:+.3f} m\n'
            f'y = {y:+.3f} m\n'
            f'θ = {math.degrees(theta):+.1f}°\n'
            f'points : {len(xs)}'
        )

        return traj_line, pos_dot, info_text

    ax.legend(loc='upper right', fontsize=8)
    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

    def on_close(event):
        if rclpy.ok():
            rclpy.shutdown()

    fig.canvas.mpl_connect('close_event', on_close)

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        plt.close('all')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
