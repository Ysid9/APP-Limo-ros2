import csv
import math
import os
import time

LIMO_HALF_TRACK = 0.086  # demi-voie Limo : track 172 mm → 86 mm


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
        v_lin, v_ang = command[0], command[1]
        e_theta_raw = (theta - ttheta) % (2 * math.pi)
        if e_theta_raw > math.pi:
            e_theta_raw -= 2 * math.pi

        self._rows.append({
            'timestamp':      round(t, 4),
            'x':              round(x, 4),
            'y':              round(y, 4),
            'theta':          round(theta, 4),
            'e_x':            round(x - tx, 4),
            'e_y':            round(y - ty, 4),
            'e_theta':        round(e_theta_raw, 4),
            'v_lin':          round(v_lin, 4),
            'v_ang':          round(v_ang, 4),
            'v_wheel_left':   round(v_lin - v_ang * LIMO_HALF_TRACK, 4),
            'v_wheel_right':  round(v_lin + v_ang * LIMO_HALF_TRACK, 4),
            'grad_0':         round(grad[0], 6),
            'grad_1':         round(grad[1], 6),
            'cost':           round(cost, 6),
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

        t  = [r['timestamp']     for r in self._rows]
        x  = [r['x']             for r in self._rows]
        y  = [r['y']             for r in self._rows]
        ex = [r['e_x']           for r in self._rows]
        ey = [r['e_y']           for r in self._rows]
        et = [r['e_theta']       for r in self._rows]
        vl = [r['v_lin']         for r in self._rows]
        va = [r['v_ang']         for r in self._rows]
        wl = [r['v_wheel_left']  for r in self._rows]
        wr = [r['v_wheel_right'] for r in self._rows]
        g0 = [r['grad_0']        for r in self._rows]
        g1 = [r['grad_1']        for r in self._rows]
        co = [r['cost']          for r in self._rows]

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
        ax4.plot(t, vl, label='v_lin'); ax4.plot(t, va, label='v_ang')
        ax4.set_title('Vitesses robot'); ax4.set_xlabel('t (s)'); ax4.legend(); ax4.grid(True)

        ax5 = fig.add_subplot(gs[1, 2])
        ax5.plot(t, wl, label='gauche'); ax5.plot(t, wr, label='droite')
        ax5.set_title('Vitesses roues (m/s)'); ax5.set_xlabel('t (s)'); ax5.legend(); ax5.grid(True)

        ax6 = fig.add_subplot(gs[2, 1])
        ax6.plot(t, g0, label='grad v_lin'); ax6.plot(t, g1, label='grad v_ang')
        ax6.set_title('Gradient'); ax6.set_xlabel('t (s)'); ax6.legend(); ax6.grid(True)

        fig.tight_layout()
        os.makedirs(os.path.dirname(png_path) if os.path.dirname(png_path) else '.', exist_ok=True)
        fig.savefig(png_path, dpi=150, bbox_inches='tight')
        if show:
            plt.show()
        plt.close(fig)
        print(f"Plot saved to {png_path}")

    def extend(self, other, session=None):
        if not other._rows:
            return
        offset = self._rows[-1]['timestamp'] if self._rows else 0.0
        for row in other._rows:
            new_row = dict(row)
            new_row['timestamp'] = round(row['timestamp'] + offset, 4)
            if session is not None:
                new_row['session'] = session
            self._rows.append(new_row)

    def reset(self):
        self._start_time = None
        self._rows = []
