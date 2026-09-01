import csv
import math
import os
import time

LIMO_HALF_TRACK = 0.086      # demi-voie Limo : track 172 mm → 86 mm (limo_driver.h track_)
LIMO_HALF_WHEELBASE = 0.1    # demi-empattement Limo : wheelbase 200 mm → 100 mm (limo_driver.h wheelbase_)


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
        v_lin, v_lat, v_ang = command[0], command[1], command[2]
        e_theta_raw = (theta - ttheta) % (2 * math.pi)
        if e_theta_raw > math.pi:
            e_theta_raw -= 2 * math.pi

        # Cinématique inverse mecanum 4 roues (lx = demi-empattement, ly = demi-voie)
        k = LIMO_HALF_WHEELBASE + LIMO_HALF_TRACK
        v_wheel_fl = v_lin - v_lat - k * v_ang
        v_wheel_fr = v_lin + v_lat + k * v_ang
        v_wheel_rl = v_lin + v_lat - k * v_ang
        v_wheel_rr = v_lin - v_lat + k * v_ang

        self._rows.append({
            'timestamp':      round(t, 4),
            'x':              round(x, 4),
            'y':              round(y, 4),
            'theta':          round(theta, 4),
            'e_x':            round(x - tx, 4),
            'e_y':            round(y - ty, 4),
            'e_theta':        round(e_theta_raw, 4),
            'v_lin':          round(v_lin, 4),
            'v_lat':          round(v_lat, 4),
            'v_ang':          round(v_ang, 4),
            'v_wheel_fl':     round(v_wheel_fl, 4),
            'v_wheel_fr':     round(v_wheel_fr, 4),
            'v_wheel_rl':     round(v_wheel_rl, 4),
            'v_wheel_rr':     round(v_wheel_rr, 4),
            'grad_0':         round(grad[0], 6),
            'grad_1':         round(grad[1], 6),
            'grad_2':         round(grad[2], 6),
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
        th = [r['theta']         for r in self._rows]
        ex = [r['e_x']           for r in self._rows]
        ey = [r['e_y']           for r in self._rows]
        et = [r['e_theta']       for r in self._rows]
        vl = [r['v_lin']         for r in self._rows]
        vt = [r['v_lat']         for r in self._rows]
        va = [r['v_ang']         for r in self._rows]
        wfl = [r['v_wheel_fl']   for r in self._rows]
        wfr = [r['v_wheel_fr']   for r in self._rows]
        wrl = [r['v_wheel_rl']   for r in self._rows]
        wrr = [r['v_wheel_rr']   for r in self._rows]
        g0 = [r['grad_0']        for r in self._rows]
        g1 = [r['grad_1']        for r in self._rows]
        g2 = [r['grad_2']        for r in self._rows]
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
        ax4.plot(t, vl, label='v_lin'); ax4.plot(t, vt, label='v_lat'); ax4.plot(t, va, label='v_ang')
        ax4.set_title('Vitesses robot'); ax4.set_xlabel('t (s)'); ax4.legend(); ax4.grid(True)

        ax5 = fig.add_subplot(gs[1, 2])
        ax5.plot(t, wfl, label='avant-gauche'); ax5.plot(t, wfr, label='avant-droite')
        ax5.plot(t, wrl, label='arrière-gauche'); ax5.plot(t, wrr, label='arrière-droite')
        ax5.set_title('Vitesses roues mecanum (m/s)'); ax5.set_xlabel('t (s)'); ax5.legend(); ax5.grid(True)

        ax6 = fig.add_subplot(gs[2, 1])
        ax6.plot(t, g0, label='grad v_lin'); ax6.plot(t, g1, label='grad v_lat'); ax6.plot(t, g2, label='grad v_ang')
        ax6.set_title('Gradient'); ax6.set_xlabel('t (s)'); ax6.legend(); ax6.grid(True)

        ax7 = fig.add_subplot(gs[2, 2])
        ax7.plot(t, x, label='x'); ax7.plot(t, y, label='y'); ax7.plot(t, th, label='θ')
        ax7.set_title('Position'); ax7.set_xlabel('t (s)'); ax7.legend(); ax7.grid(True)

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
