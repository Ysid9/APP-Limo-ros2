#!/usr/bin/env python3
"""
visualize_cost.py — Visualisation haute qualité du paysage de coût appris.

Fenêtre matplotlib (3 panneaux 2D) :
  1. Heatmap J(x,y) + isolignes
  2. Champ de vecteurs normalisés (direction commandée, couleur = |v_lin|)
  3. Carte de sortie réseau v_lin

Fenêtre navigateur (plotly, WebGL) :
  - Surface 3D interactive fluide
"""
import json, math, sys, os
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.ticker import AutoMinorLocator
import plotly.graph_objects as go
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from torch_back import PioneerNN

# ── Paramètres ───────────────────────────────────────────────────────────────
WEIGHT_FILE = 'last_w_torch_ackermann.json'
TARGET      = [0.0, 0.0, 0.0]
THETA_ROBOT = 0.0
EXTENT      = 3.0
N_COST      = 220
N_ARROWS    = 24
ALPHA       = [1/6, 1/6, 1/math.pi]
CMAP_COST   = 'inferno_r'
DPI         = 140

def theta_s_np(X, Y):
    return np.tanh(10. * X) * np.arctan(Y)

# ── Chargement du réseau ─────────────────────────────────────────────────────
with open(WEIGHT_FILE) as f:
    obj = json.load(f)
net = PioneerNN(obj['input_size'], obj['hidden_size'], obj['output_size'])
net.load_weights_from_json(obj)
net.eval()
print(f"Réseau : {obj['input_size']}→{obj['hidden_size']}→{obj['output_size']}")
print(f"Cible : {TARGET}   θ robot : {math.degrees(THETA_ROBOT):.1f}°")

# ── Grille coût ───────────────────────────────────────────────────────────────
xs = np.linspace(-EXTENT, EXTENT, N_COST)
ys = np.linspace(-EXTENT, EXTENT, N_COST)
X2, Y2 = np.meshgrid(xs, ys)

TS = theta_s_np(X2, Y2)
ex = (X2 - TARGET[0]) * ALPHA[0]
ey = (Y2 - TARGET[1]) * ALPHA[1]
et = (THETA_ROBOT - TARGET[2] - TS) * ALPHA[2]
J_grid = ex**2 + ey**2 + et**2

# ── Commandes réseau sur grille ───────────────────────────────────────────────
inp_flat = np.stack([ex.ravel(), ey.ravel(), et.ravel()], axis=1).astype(np.float32)
with torch.no_grad():
    cmd_flat = net(torch.from_numpy(inp_flat)).numpy()

v_lin_grid = cmd_flat[:, 0].reshape(N_COST, N_COST)

# ── Grille flèches ────────────────────────────────────────────────────────────
xa = np.linspace(-EXTENT, EXTENT, N_ARROWS)
ya = np.linspace(-EXTENT, EXTENT, N_ARROWS)
Xa, Ya = np.meshgrid(xa, ya)
TSa = theta_s_np(Xa, Ya)
exa = (Xa - TARGET[0]) * ALPHA[0]
eya = (Ya - TARGET[1]) * ALPHA[1]
eta = (THETA_ROBOT - TARGET[2] - TSa) * ALPHA[2]

inp_a = np.stack([exa.ravel(), eya.ravel(), eta.ravel()], axis=1).astype(np.float32)
with torch.no_grad():
    cmd_a = net(torch.from_numpy(inp_a)).numpy()

vl_a  = cmd_a[:, 0].reshape(N_ARROWS, N_ARROWS)
c_th  = math.cos(THETA_ROBOT)
s_th  = math.sin(THETA_ROBOT)
U_raw = vl_a * c_th
V_raw = vl_a * s_th
mag   = np.hypot(U_raw, V_raw)
mag[mag == 0] = 1.0
U_norm = U_raw / mag
V_norm = V_raw / mag
C_arr  = np.abs(vl_a)

# ═══════════════════════════════════════════════════════════════════════════════
# PLOTLY — surface 3D dans le navigateur (WebGL, 60 fps)
# ═══════════════════════════════════════════════════════════════════════════════
print("Ouverture de la surface 3D dans le navigateur…")
tick_vals = np.arange(-EXTENT, EXTENT + 0.01, 1.0).tolist()

_axis_common = dict(
    showbackground=True,
    backgroundcolor='#f0f2f5',
    gridcolor='#999999',
    gridwidth=1,
    zerolinecolor='#333333',
    zerolinewidth=2,
    showgrid=True,
    showline=True,
    linecolor='#333333',
    linewidth=2,
    showticklabels=True,
    tickfont=dict(size=11, color='#111111'),
    tickvals=tick_vals,
    ticktext=[f'{v:.0f}' for v in tick_vals],
)

fig3d = go.Figure(data=[
    go.Surface(
        x=xs, y=ys, z=J_grid,
        colorscale='Inferno_r',
        reversescale=False,
        showscale=True,
        colorbar=dict(
            title=dict(text='J', font=dict(size=13)),
            tickfont=dict(size=11),
            thickness=18, len=0.75,
        ),
        lighting=dict(ambient=0.7, diffuse=0.6, roughness=0.5, specular=0.2),
        # isolignes projetées sur le sol du plan xy
        contours=dict(
            x=dict(show=True, color='#555555', width=1, usecolormap=False,
                   start=-EXTENT, end=EXTENT, size=1.0, project=dict(x=True)),
            y=dict(show=True, color='#555555', width=1, usecolormap=False,
                   start=-EXTENT, end=EXTENT, size=1.0, project=dict(y=True)),
            z=dict(show=False),
        ),
    )
])
fig3d.update_layout(
    title=dict(
        text=f'Coût J — surface 3D interactive<br>'
             f'<sup>{WEIGHT_FILE} · θ={math.degrees(THETA_ROBOT):.0f}°</sup>',
        font=dict(size=14), x=0.5, xanchor='center'
    ),
    scene=dict(
        xaxis=dict(title=dict(text='x (m)', font=dict(size=13)), **_axis_common),
        yaxis=dict(title=dict(text='y (m)', font=dict(size=13)), **_axis_common),
        zaxis=dict(title=dict(text='J',     font=dict(size=13)),
                   **{k: v for k, v in _axis_common.items()
                      if k not in ('tickvals', 'ticktext')}),
        camera=dict(eye=dict(x=1.4, y=-1.6, z=1.0)),
        aspectmode='manual',
        aspectratio=dict(x=1, y=1, z=0.6),
    ),
    paper_bgcolor='white',
    margin=dict(l=10, r=10, t=80, b=10),
    width=950, height=750,
)
fig3d.show()   # ouvre dans le navigateur par défaut

# ═══════════════════════════════════════════════════════════════════════════════
# MATPLOTLIB — 3 panneaux 2D
# ═══════════════════════════════════════════════════════════════════════════════
plt.style.use('seaborn-v0_8-whitegrid')
fig = plt.figure(figsize=(20, 7), dpi=DPI)
fig.patch.set_facecolor('white')
fig.suptitle(
    f'Paysage de coût appris — {WEIGHT_FILE}   (θ robot = {math.degrees(THETA_ROBOT):.0f}°)',
    fontsize=13, color='#111111', fontweight='bold', y=0.98
)

def style_ax(ax, title):
    ax.set_title(title, color='#111111', fontsize=11, pad=8)
    ax.set_xlabel('x (m)', color='#333333', fontsize=9)
    ax.set_ylabel('y (m)', color='#333333', fontsize=9)
    ax.tick_params(colors='#333333', labelsize=8)
    for sp in ax.spines.values():
        sp.set_edgecolor('#bbbbbb')
    ax.xaxis.set_minor_locator(AutoMinorLocator(2))
    ax.yaxis.set_minor_locator(AutoMinorLocator(2))
    ax.tick_params(which='minor', color='#cccccc', length=2)

# ── Panneau 1 : heatmap J + isolignes ────────────────────────────────────────
ax1 = fig.add_subplot(131, facecolor='white')
im1 = ax1.imshow(
    J_grid,
    extent=[-EXTENT, EXTENT, -EXTENT, EXTENT],
    origin='lower', cmap=CMAP_COST, aspect='equal',
    interpolation='bicubic', vmin=0
)
levels = np.percentile(J_grid, [10, 25, 40, 55, 70, 82, 92])
cs = ax1.contour(X2, Y2, J_grid, levels=levels, colors='white', linewidths=0.6, alpha=0.5)
ax1.clabel(cs, fmt='%.3f', fontsize=6, colors='white', inline_spacing=2)
ax1.plot(TARGET[0], TARGET[1], '+', ms=20, mew=2.5, color='cyan', zorder=5, label='cible')
cb1 = plt.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04)
cb1.set_label('J', color='#333', fontsize=9)
cb1.ax.yaxis.set_tick_params(color='#444', labelcolor='#444', labelsize=7)
style_ax(ax1, 'Coût J — vue dessus')
ax1.legend(loc='upper right', fontsize=8, facecolor='white', edgecolor='#aaa', labelcolor='#111')

# ── Panneau 2 : champ de flèches ─────────────────────────────────────────────
ax2 = fig.add_subplot(132, facecolor='white')
ax2.imshow(
    J_grid, extent=[-EXTENT, EXTENT, -EXTENT, EXTENT],
    origin='lower', cmap=CMAP_COST, aspect='equal',
    interpolation='bicubic', alpha=0.35, vmin=0
)
norm_c = mcolors.Normalize(vmin=C_arr.min(), vmax=C_arr.max())
qv = ax2.quiver(
    Xa, Ya, U_norm, V_norm, C_arr,
    cmap='cool', norm=norm_c,
    scale=N_ARROWS * 1.05, width=0.004,
    headwidth=4, headlength=5, headaxislength=4.5,
    pivot='mid'
)
ax2.plot(TARGET[0], TARGET[1], '+', ms=20, mew=2.5, color='blue', zorder=5, label='cible')
cb2 = plt.colorbar(qv, ax=ax2, fraction=0.046, pad=0.04)
cb2.set_label('|v_lin|', color='#333', fontsize=9)
cb2.ax.yaxis.set_tick_params(color='#444', labelcolor='#444', labelsize=7)
ax2.set_xlim(-EXTENT, EXTENT)
ax2.set_ylim(-EXTENT, EXTENT)
ax2.set_aspect('equal')
style_ax(ax2, 'Direction commandée (normalisée)')
ax2.legend(loc='upper right', fontsize=8, facecolor='white', edgecolor='#aaa', labelcolor='#111')

# ── Panneau 3 : v_lin sortie réseau ──────────────────────────────────────────
ax3 = fig.add_subplot(133, facecolor='white')
vmax4 = np.abs(v_lin_grid).max()
im3 = ax3.imshow(
    v_lin_grid,
    extent=[-EXTENT, EXTENT, -EXTENT, EXTENT],
    origin='lower', cmap='RdBu_r', aspect='equal',
    interpolation='bicubic',
    vmin=-vmax4, vmax=vmax4
)
ax3.plot(TARGET[0], TARGET[1], '+', ms=20, mew=2.5, color='blue', zorder=5, label='cible')
cb3 = plt.colorbar(im3, ax=ax3, fraction=0.046, pad=0.04)
cb3.set_label('v_lin (sortie réseau)', color='#333', fontsize=9)
cb3.ax.yaxis.set_tick_params(color='#444', labelcolor='#444', labelsize=7)
style_ax(ax3, 'Commande v_lin (réseau)')
ax3.legend(loc='upper right', fontsize=8, facecolor='white', edgecolor='#aaa', labelcolor='#111')

plt.tight_layout(rect=[0, 0, 1, 0.96])
out_png = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cost_landscape.png')
plt.savefig(out_png, dpi=DPI, facecolor='white', bbox_inches='tight')
print(f"Image sauvegardée → {out_png}")
plt.show()
