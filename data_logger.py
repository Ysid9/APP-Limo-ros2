import csv
import math
import os
import time


class DataLogger:
    def __init__(self):
        self._start_time = None
        self._rows = []

    def log(self, position, target, command, grad, cost):
        if self._start_time is None:
            self._start_time = time.time()

        t = time.time() - self._start_time
        x, y, theta = position
        tx, ty, ttheta = target
        v_gauche, v_droite = command[0], command[1]
        e_theta_raw = (theta - ttheta) % (2 * math.pi)
        if e_theta_raw > math.pi:
            e_theta_raw -= 2 * math.pi

        self._rows.append({
            'timestamp': round(t, 4),
            'x':         round(x, 4),
            'y':         round(y, 4),
            'theta':     round(theta, 4),
            'e_x':       round(x - tx, 4),
            'e_y':       round(y - ty, 4),
            'e_theta':   round(e_theta_raw, 4),
            'v_gauche':  round(v_gauche, 4),
            'v_droite':  round(v_droite, 4),
            'grad_0':    round(grad[0], 6),
            'grad_1':    round(grad[1], 6),
            'cost':      round(cost, 6),
        })

    def save(self, filename):
        if not self._rows:
            return
        os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else '.', exist_ok=True)
        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self._rows[0].keys())
            writer.writeheader()
            writer.writerows(self._rows)
        print(f"Data saved to {filename} ({len(self._rows)} iterations)")

    @classmethod
    def from_csv(cls, path):
        dl = cls()
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                dl._rows.append({k: float(v) for k, v in row.items()})
        return dl

    def save_plot(self, png_path, show=False):
        if not self._rows:
            return
        try:
            import sys
            import matplotlib
            if 'matplotlib.pyplot' not in sys.modules:
                matplotlib.use('Qt5Agg' if show else 'Agg')
            import matplotlib.pyplot as plt
            from matplotlib.gridspec import GridSpec
        except Exception:
            print("matplotlib not available — skipping plot generation")
            return

        t   = [r['timestamp'] for r in self._rows]
        x   = [r['x']         for r in self._rows]
        y   = [r['y']         for r in self._rows]
        th  = [r['theta']     for r in self._rows]
        ex  = [r['e_x']       for r in self._rows]
        ey  = [r['e_y']       for r in self._rows]
        et  = [r['e_theta']   for r in self._rows]
        vg  = [r['v_gauche']  for r in self._rows]
        vd  = [r['v_droite']  for r in self._rows]
        g0  = [r['grad_0']    for r in self._rows]
        g1  = [r['grad_1']    for r in self._rows]
        co  = [r['cost']      for r in self._rows]

        fig = plt.figure(figsize=(16, 10))
        fig.suptitle(os.path.basename(png_path), fontsize=11)
        gs = GridSpec(3, 3, figure=fig)

        ax = fig.add_subplot(gs[0:2, 0])
        ax.plot(x, y, 'b-', linewidth=1)
        ax.plot(x[0], y[0], 'go', markersize=8, label='start')
        ax.plot(0, 0, 'r*', markersize=12, label='target')
        ax.set_title('Trajectoire XY'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.legend(); ax.grid(True); ax.set_aspect('equal', adjustable='datalim')

        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(t, co, 'r-', linewidth=1)
        ax2.set_title('Coût'); ax2.set_xlabel('t (s)'); ax2.grid(True)

        ax3 = fig.add_subplot(gs[0, 2])
        ax3.plot(t, ex, label='e_x'); ax3.plot(t, ey, label='e_y'); ax3.plot(t, et, label='e_θ')
        ax3.set_title('Erreurs'); ax3.set_xlabel('t (s)'); ax3.legend(); ax3.grid(True)

        ax4 = fig.add_subplot(gs[1, 1])
        ax4.plot(t, vg, label='v_gauche'); ax4.plot(t, vd, label='v_droite')
        ax4.set_title('Vitesses roues (rad/s)'); ax4.set_xlabel('t (s)'); ax4.legend(); ax4.grid(True)

        ax5 = fig.add_subplot(gs[1, 2])
        ax5.plot(t, g0, label='grad gauche'); ax5.plot(t, g1, label='grad droite')
        ax5.set_title('Gradient'); ax5.set_xlabel('t (s)'); ax5.legend(); ax5.grid(True)

        ax6 = fig.add_subplot(gs[2, 1])
        ax6.plot(t, x, label='x'); ax6.plot(t, y, label='y'); ax6.plot(t, th, label='θ')
        ax6.set_title('Position'); ax6.set_xlabel('t (s)'); ax6.legend(); ax6.grid(True)

        fig.tight_layout()
        os.makedirs(os.path.dirname(png_path) if os.path.dirname(png_path) else '.', exist_ok=True)
        fig.savefig(png_path, dpi=150, bbox_inches='tight')
        if show:
            plt.show()
        plt.close(fig)
        print(f"Plot saved to {png_path}")

    def reset(self):
        self._start_time = None
        self._rows = []

    @classmethod
    def save_run_summary(cls, session_csv_paths, out_dir):
        """Génère à la fin du run : CSV total, PNG total, PNG trajectoires multi-sessions."""
        if not session_csv_paths:
            return

        try:
            import matplotlib
            import matplotlib.pyplot as plt
        except Exception:
            print("matplotlib not available — skipping run summary plots")
            return

        session_loggers = [cls.from_csv(p) for p in session_csv_paths]

        # CSV total (toutes sessions concaténées)
        total_rows = []
        for idx, dl in enumerate(session_loggers):
            for row in dl._rows:
                total_rows.append({'session': idx + 1, **row})

        if total_rows:
            total_csv = os.path.join(out_dir, 'run_total.csv')
            os.makedirs(out_dir, exist_ok=True)
            with open(total_csv, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=total_rows[0].keys())
                writer.writeheader()
                writer.writerows(total_rows)
            print(f"Combined CSV saved to {total_csv} ({len(total_rows)} iterations)")

            # PNG total (toutes données combinées)
            combined = cls()
            combined._rows = [row for dl in session_loggers for row in dl._rows]
            combined.save_plot(os.path.join(out_dir, 'run_total.png'))

        # PNG trajectoires : une couleur par session
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_title('Trajectoires — toutes sessions', fontweight='bold')
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        ax.axvline(x=0, color='k', linestyle='-', alpha=0.3)

        colors = plt.cm.tab10.colors
        for i, dl in enumerate(session_loggers):
            if not dl._rows:
                continue
            color = colors[i % len(colors)]
            x = [r['x'] for r in dl._rows]
            y = [r['y'] for r in dl._rows]
            ax.plot(x, y, '-', color=color, linewidth=1.5, label=f'Session {i + 1}')
            ax.plot(x[0], y[0], '^', color=color, markersize=10)

        ax.plot(0, 0, 'r*', markersize=15, label='Cible', zorder=5)
        ax.legend(loc='upper right')
        ax.set_aspect('equal', adjustable='datalim')
        fig.tight_layout()
        traj_path = os.path.join(out_dir, 'run_trajectoires.png')
        fig.savefig(traj_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"Trajectory plot saved to {traj_path}")
