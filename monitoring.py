import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec
import math
import time
from collections import deque
import os
import queue


class RobotMonitor:
    def __init__(self, max_points=None, update_interval=300, world_bounds=(-10, 10, -10, 10)):
        self.max_points = max_points
        self.update_interval = update_interval
        self.world_bounds = world_bounds
        self.data_queue = queue.Queue()

        self.cost_history = deque(maxlen=max_points)
        self.trajectory   = deque(maxlen=max_points)
        self.full_trajectory = []   # toutes sessions (None = coupure de session)
        self.wheel_speeds = deque(maxlen=max_points)
        self.gradients    = deque(maxlen=max_points)
        self.distances    = deque(maxlen=max_points)
        self.timestamps   = deque(maxlen=max_points)
        self.start_set    = False

        self.fig = None
        self.ani = None
        self.running = False

    def setup_plots(self):
        plt.ion()
        self.fig = plt.figure(figsize=(15, 10), facecolor='#f8f9fa')
        self.fig.canvas.manager.set_window_title('Robot Pioneer - Monitoring en temps réel')
        gs = GridSpec(3, 3, figure=self.fig)

        self.ax_trajectory = self.fig.add_subplot(gs[0:2, 0:2])
        self.ax_trajectory.set_title('Trajectoire du robot', fontweight='bold')
        self.ax_trajectory.set_xlabel('Position X (m)')
        self.ax_trajectory.set_ylabel('Position Y (m)')
        self.ax_trajectory.grid(True, linestyle='--', alpha=0.7)
        self.ax_trajectory.set_xlim(self.world_bounds[0], self.world_bounds[1])
        self.ax_trajectory.set_ylim(self.world_bounds[2], self.world_bounds[3])
        self.ax_trajectory.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        self.ax_trajectory.axvline(x=0, color='k', linestyle='-', alpha=0.3)

        self.trajectory_line,  = self.ax_trajectory.plot([], [], 'b-', linewidth=2, label='Trajectoire')
        self.trajectory_point, = self.ax_trajectory.plot([], [], 'ro', markersize=8,  label='Position actuelle')
        self.target_point,     = self.ax_trajectory.plot([], [], 'g*', markersize=12, label='Cible')
        self.start_point,      = self.ax_trajectory.plot([], [], 'y^', markersize=10, label='Départ')
        self.ax_trajectory.legend(loc='upper right')

        self.ax_cost = self.fig.add_subplot(gs[0, 2])
        self.ax_cost.set_title('Fonction de coût', fontweight='bold')
        self.ax_cost.set_xlabel('Temps (s)')
        self.ax_cost.set_ylabel('Coût')
        self.ax_cost.grid(True, linestyle='--', alpha=0.7)
        self.cost_line, = self.ax_cost.plot([], [], 'r-', linewidth=2)

        self.ax_wheels = self.fig.add_subplot(gs[1, 2])
        self.ax_wheels.set_title('Vitesses des roues', fontweight='bold')
        self.ax_wheels.set_xlabel('Temps (s)')
        self.ax_wheels.set_ylabel('Vitesse (rad/s)')
        self.ax_wheels.grid(True, linestyle='--', alpha=0.7)
        self.left_wheel_line,  = self.ax_wheels.plot([], [], 'b-', linewidth=2, label='Roue gauche')
        self.right_wheel_line, = self.ax_wheels.plot([], [], 'g-', linewidth=2, label='Roue droite')
        self.ax_wheels.legend(loc='upper right')

        self.ax_gradient = self.fig.add_subplot(gs[2, 0])
        self.ax_gradient.set_title('Gradient', fontweight='bold')
        self.ax_gradient.set_xlabel('Temps (s)')
        self.ax_gradient.set_ylabel('Valeur du gradient')
        self.ax_gradient.grid(True, linestyle='--', alpha=0.7)
        self.grad_line_left,  = self.ax_gradient.plot([], [], 'c-', linewidth=2, label='Grad roue gauche')
        self.grad_line_right, = self.ax_gradient.plot([], [], 'm-', linewidth=2, label='Grad roue droite')
        self.ax_gradient.legend(loc='upper right')

        self.ax_distance_xy = self.fig.add_subplot(gs[2, 1])
        self.ax_distance_xy.set_title('Erreurs de position X et Y', fontweight='bold')
        self.ax_distance_xy.set_xlabel('Temps (s)')
        self.ax_distance_xy.set_ylabel('Erreur')
        self.ax_distance_xy.grid(True, linestyle='--', alpha=0.7)
        self.distance_x_line, = self.ax_distance_xy.plot([], [], 'r-', linewidth=2, label='Erreur X')
        self.distance_y_line, = self.ax_distance_xy.plot([], [], 'g-', linewidth=2, label='Erreur Y')
        self.ax_distance_xy.legend(loc='upper right')

        self.ax_distance_theta = self.fig.add_subplot(gs[2, 2])
        self.ax_distance_theta.set_title('Erreur θ', fontweight='bold')
        self.ax_distance_theta.set_xlabel('Temps (s)')
        self.ax_distance_theta.set_ylabel('Erreur θ')
        self.ax_distance_theta.grid(True, linestyle='--', alpha=0.7)
        self.distance_theta_line, = self.ax_distance_theta.plot([], [], 'b-', linewidth=2, label='Erreur θ')
        self.ax_distance_theta.legend(loc='upper right')

        self.fig.tight_layout(pad=3.0)
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plots,
            interval=self.update_interval,
            blit=False,
            cache_frame_data=False,
        )

    def update_plots(self, frame):
        while not self.data_queue.empty():
            try:
                data_point = self.data_queue.get_nowait()
                if data_point['type'] == 'data':
                    self.cost_history.append(data_point['cost'])
                    pos = data_point['position']
                    self.trajectory.append(pos)
                    self.full_trajectory.append(pos)
                    self.wheel_speeds.append(data_point['wheel_speeds'])
                    self.gradients.append(data_point['gradient'])
                    self.distances.append(data_point['distances'])
                    self.timestamps.append(data_point['timestamp'])
                elif data_point['type'] == 'target':
                    t = data_point['target']
                    self.target_point.set_data([t[0]], [t[1]])
                elif data_point['type'] == 'new_session':
                    self.trajectory.clear()
                    self.start_set = False
                    if self.full_trajectory:
                        self.full_trajectory.append(None)
                self.data_queue.task_done()
            except queue.Empty:
                break

        if not self.timestamps:
            return

        t_start  = self.timestamps[0]
        rel_time = [(t - t_start) for t in self.timestamps]
        max_t    = max(rel_time[-1], 1.0)

        if self.trajectory:
            x_data = [p[0] for p in self.trajectory]
            y_data = [p[1] for p in self.trajectory]
            self.trajectory_line.set_data(x_data, y_data)
            self.trajectory_point.set_data([x_data[-1]], [y_data[-1]])
            if not self.start_set:
                self.start_point.set_data([x_data[0]], [y_data[0]])
                self.start_set = True

        if self.cost_history:
            self.cost_line.set_data(rel_time, self.cost_history)
            self.ax_cost.set_xlim(0, max_t)
            self.ax_cost.set_ylim(0, max(max(self.cost_history) * 1.1, 0.1))

        if self.wheel_speeds:
            left_speeds  = [ws[0] for ws in self.wheel_speeds]
            right_speeds = [ws[1] for ws in self.wheel_speeds]
            self.left_wheel_line.set_data(rel_time, left_speeds)
            self.right_wheel_line.set_data(rel_time, right_speeds)
            self.ax_wheels.set_xlim(0, max_t)
            min_s  = min(min(left_speeds), min(right_speeds))
            max_s  = max(max(left_speeds), max(right_speeds))
            margin = max((max_s - min_s) * 0.1, 0.1)
            self.ax_wheels.set_ylim(min_s - margin, max_s + margin)

        if self.gradients:
            grad_left  = [g[0] for g in self.gradients]
            grad_right = [g[1] for g in self.gradients]
            self.grad_line_left.set_data(rel_time, grad_left)
            self.grad_line_right.set_data(rel_time, grad_right)
            self.ax_gradient.set_xlim(0, max_t)
            all_grads = grad_left + grad_right
            min_g  = min(all_grads)
            max_g  = max(all_grads)
            margin = max((max_g - min_g) * 0.1, 0.1)
            self.ax_gradient.set_ylim(min_g - margin, max_g + margin)

        if self.distances:
            err_x     = [d[0] for d in self.distances]
            err_y     = [d[1] for d in self.distances]
            err_theta = [d[2] for d in self.distances]
            all_xy    = err_x + err_y
            min_xy = min(all_xy); max_xy = max(all_xy)
            margin_xy = max((max_xy - min_xy) * 0.1, 0.1)
            self.distance_x_line.set_data(rel_time, err_x)
            self.distance_y_line.set_data(rel_time, err_y)
            self.ax_distance_xy.set_xlim(0, max_t)
            self.ax_distance_xy.set_ylim(min_xy - margin_xy, max_xy + margin_xy)
            min_et = min(err_theta); max_et = max(err_theta)
            margin_t = max((max_et - min_et) * 0.1, 0.1)
            self.distance_theta_line.set_data(rel_time, err_theta)
            self.ax_distance_theta.set_xlim(0, max_t)
            self.ax_distance_theta.set_ylim(min_et - margin_t, max_et + margin_t)

        self.fig.canvas.flush_events()

    def new_session(self):
        self.data_queue.put({'type': 'new_session'})

    def start(self):
        if not self.running:
            self.running = True
            self.setup_plots()

    def stop(self):
        if self.running:
            self.running = False
            if self.fig:
                plt.close(self.fig)

    def add_data_point(self, position, wheel_speeds, gradient, cost, distances):
        if self.running:
            self.data_queue.put({
                'type':        'data',
                'position':    position,
                'wheel_speeds': wheel_speeds,
                'gradient':    gradient,
                'cost':        cost,
                'distances':   distances,
                'timestamp':   time.time()
            })

    def set_target(self, target):
        if self.running:
            self.data_queue.put({'type': 'target', 'target': target})

    def save_plots(self, base_filename="robot_monitoring", timestamp=True,
                   results_dir='res', final=False):
        if timestamp:
            base_filename = f"{base_filename}_{time.strftime('%Y%m%d_%H%M%S')}"

        os.makedirs(results_dir, exist_ok=True)

        if not self.fig:
            return

        if final and self.full_trajectory:
            xs, ys = [], []
            segments = []
            current_seg = []
            for pt in self.full_trajectory:
                if pt is None:
                    if current_seg:
                        segments.append(current_seg)
                    current_seg = []
                    xs.append(float('nan'))
                    ys.append(float('nan'))
                else:
                    current_seg.append(pt)
                    xs.append(pt[0])
                    ys.append(pt[1])
            if current_seg:
                segments.append(current_seg)

            self.trajectory_line.set_data(xs, ys)
            for seg in segments:
                self.ax_trajectory.plot(seg[0][0], seg[0][1], 'y^', markersize=10)
            valid_xs = [v for v in xs if not math.isnan(v)]
            valid_ys = [v for v in ys if not math.isnan(v)]
            if valid_xs and valid_ys:
                pad = 0.5
                self.ax_trajectory.set_xlim(min(valid_xs) - pad, max(valid_xs) + pad)
                self.ax_trajectory.set_ylim(min(valid_ys) - pad, max(valid_ys) + pad)

        save_path = os.path.join(results_dir, f"{base_filename}_all.png")
        self.fig.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Monitoring saved to {save_path}")

        if final and self.full_trajectory and segments:
            traj_fig, ax_traj = plt.subplots(figsize=(8, 8))
            ax_traj.set_title('Trajectoire complète — toutes sessions', fontweight='bold')
            ax_traj.set_xlabel('x (m)'); ax_traj.set_ylabel('y (m)')
            ax_traj.grid(True, linestyle='--', alpha=0.7)
            ax_traj.axhline(y=0, color='k', linestyle='-', alpha=0.3)
            ax_traj.axvline(x=0, color='k', linestyle='-', alpha=0.3)
            colors = plt.cm.tab10.colors
            for i, seg in enumerate(segments):
                color = colors[i % len(colors)]
                sx = [p[0] for p in seg]; sy = [p[1] for p in seg]
                ax_traj.plot(sx, sy, '-', color=color, linewidth=1.5, label=f'Session {i+1}')
                ax_traj.plot(sx[0], sy[0], '^', color=color, markersize=10)
            ax_traj.plot(0, 0, 'r*', markersize=15, label='Cible (0,0,0)', zorder=5)
            ax_traj.legend(loc='upper right')
            ax_traj.set_aspect('equal', adjustable='datalim')
            traj_path = os.path.join(results_dir, f"{base_filename}_trajectory.png")
            traj_fig.tight_layout()
            traj_fig.savefig(traj_path, dpi=300, bbox_inches='tight')
            plt.close(traj_fig)
            print(f"Trajectory summary saved to {traj_path}")


class RobotMonitorAdapter:
    def __init__(self, world_bounds=(-10, 10, -10, 10)):
        self.monitor = RobotMonitor(world_bounds=world_bounds)
        self.last_target = None

    def start_monitoring(self):
        self.monitor.start()

    def stop_monitoring(self):
        self.monitor.stop()

    def start_session(self):
        self.monitor.new_session()

    def set_target(self, target):
        self.last_target = target
        self.monitor.set_target(target)

    def update(self, position, wheel_speeds, gradient=None, cost=None):
        distances = self._calculate_distances(position, self.last_target or [0, 0, 0])
        self.monitor.add_data_point(
            position=position,
            wheel_speeds=wheel_speeds,
            gradient=gradient if gradient is not None else [0, 0],
            cost=cost if cost is not None else self._calculate_cost(position, self.last_target or [0, 0, 0]),
            distances=distances
        )

    def _calculate_cost(self, position, target):
        def theta_s(x, y):
            return math.tanh(10.*x)*math.atan(1.*y)
        alpha_x = 1/6; alpha_y = 1/6; alpha_theta = 1/math.pi
        return (alpha_x**2*(position[0]-target[0])**2 +
                alpha_y**2*(position[1]-target[1])**2 +
                alpha_theta**2*(position[2]-target[2]-theta_s(position[0], position[1]))**2)

    def _calculate_distances(self, position, target):
        theta_diff = (position[2] - target[2]) % (2 * math.pi)
        if theta_diff > math.pi:
            theta_diff -= 2 * math.pi
        return [position[0]-target[0], position[1]-target[1], theta_diff]

    def save_results(self, base_filename="robot_results", results_dir='res', final=False):
        self.monitor.save_plots(base_filename, timestamp=False,
                                results_dir=results_dir, final=final)
