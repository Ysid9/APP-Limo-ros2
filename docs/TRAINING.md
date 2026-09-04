# Lancer une session d'apprentissage / de test

Branche `mecanum` — robot mecanum (4 roues holonomes), réseau à 3 sorties
(`v_lin`, `v_lat`, `v_ang`).

## En simulation (Gazebo)

**Terminal 1 — lancer Gazebo en mode mecanum :**
```bash
source /opt/ros/humble/setup.bash   # ou foxy
source ~/ros2_ws/install/setup.bash
ros2 launch limo_description gazebo_models_mcnamu.launch.py
```
Le robot Limo (modèle mecanum) apparaît à la position (0, 0, 0). Le
déplacer à la position initiale voulue (dans Gazebo, avant de lancer le
script) si besoin.

**Terminal 2 — lancer le trainer :**
```bash
source /opt/ros/humble/setup.bash   # ou foxy
source ~/ros2_ws/install/setup.bash
cd /chemin/vers/ce/depot
python3 torch_run_ros2.py
```

Prompts :
| Prompt | Réponse |
|---|---|
| `Real robot? (y/n)` | `n` pour Gazebo |
| `Enable real-time display? (y/n)` | `y` pour afficher les graphes de monitoring en direct (v_lin/v_lat/v_ang, 4 vitesses de roues) |
| `Do you want to load previous network? (y/n)` | `y` pour charger `last_w_torch_mecanum.json` (poids de la dernière session) — **`n` repart d'un réseau initialisé aléatoirement** (pas de reprise) |
| `Do you want to learn? (y/n)` | `y` pour activer la rétropropagation, `n` pour évaluer sans apprendre |
| `Enter the first target : x y radian` | **toujours `0 0 0`** — le mode étant holonome, une cible hors de l'axe avant du robot est correctement gérée (pas besoin d'orienter le robot vers la cible au préalable) |

Le robot converge vers la cible. Appuyer sur Entrée pour arrêter la session
en cours. En fin de script, choix de sauvegarder les poids
(`last_w_torch_mecanum.json`) — supprimer ce fichier pour repartir d'un
réseau initialisé aléatoirement au prochain lancement, même en répondant `y`.

Résultats (CSV + graphes) dans `res/gazebo/run_YYYYMMDD_HHMMSS/`.

## Sur robot réel

**Trois terminaux nécessaires**, dans cet ordre :

1. **Driver du robot, en mode mecanum explicitement** (sinon le robot reste
   en mode différentiel par défaut et ignore `v_lat`) :
   ```bash
   ros2 launch limo_base limo_base.launch.py use_mcnamu:=true
   ```
   Sur `.113`, ajouter aussi `port_name:=ttyUSB0` (voir [`ROBOTS.md`](ROBOTS.md)) :
   ```bash
   ros2 launch limo_base limo_base.launch.py use_mcnamu:=true port_name:=ttyUSB0
   ```
2. **Téléopération** (pour repositionner le robot entre les sessions) :
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   (voir [`INSTALL.md`](INSTALL.md) pour l'installer si absent)
3. **Apprentissage** — `torch_run_ros2.py` (prompts complets, cible **toujours
   `0 0 0`**) ou `torch_run_real.py` (version rapide : robot réel et cible
   `(0,0,0)` systématiques, monitoring auto, garde-fous de vitesse actifs
   — `v_lin`/`v_lat` limités à ±0.3 m/s, `v_ang` à ±0.8 rad/s) :
   ```bash
   python3 torch_run_ros2.py     # ou torch_run_real.py
   ```

**Procédure d'une session de terrain :**
1. Positionner le robot physiquement à `(0, 0, 0)` (repère de référence, terminal 2 = téléop).
2. Lancer le driver (terminal 1) puis l'apprentissage (terminal 3).
3. Déplacer le robot avec la téléop vers un point de départ éloigné de la cible.
4. Dès que le robot atteint la cible, appuyer sur Entrée dans le terminal 3 pour arrêter la session en cours.
5. Répondre `y` à "Do you want to continue?", repositionner le robot avec la téléop, relancer une nouvelle session.
6. Répéter 3-5 autant de fois que nécessaire.

Résultats dans `res/robot/run_YYYYMMDD_HHMMSS/` (restent en local sur la
machine qui a lancé le script — aucun envoi automatique ailleurs).

## Rejouer un CSV existant

```bash
python3 plot_results.py res/robot/run_20260825_200942/eval_session_1_20260825_201024.csv
```
`data_logger.py` recalcule aussi la cinématique inverse 4 roues
(`v_wheel_fl/fr/rl/rr`) à partir de `v_lin, v_lat, v_ang` et des dimensions
réelles du Limo (empattement 0.2 m, voie 0.172 m) — visibles dans le graphe
"Vitesses roues mecanum".

## Paramètres modifiables (`torch_run_ros2.py`)

```python
HL_size       = 1000   # taille de la couche cachée du réseau
LEARNING_RATE = 0.2    # pas d'apprentissage
```

### Ajouter une couche cachée supplémentaire

Le réseau (`torch_back.py`, classe `PioneerNN`) n'a qu'une seule couche
cachée. En ajouter une deuxième demande 3 modifications :

1. Dans `__init__`, ajouter la couche et son initialisation :
   ```python
   self.hidden2 = nn.Linear(hidden_size, hidden_size2)
   nn.init.xavier_uniform_(self.hidden2.weight)
   nn.init.zeros_(self.hidden2.bias)
   ```
2. Dans `forward`, l'insérer entre la couche cachée existante et la sortie :
   ```python
   x = self.activation(self.hidden(x))
   x = self.activation(self.hidden2(x))
   x = self.activation(self.output(x))
   ```
3. **Attention à la compatibilité des poids** : en changeant le nombre de
   couches cachées, les fichiers `last_w_torch_*.json` déjà sauvegardés ne
   pourront plus être chargés sur le nouveau réseau — et inversement, les
   poids du nouveau réseau ne seront pas compatibles avec l'ancienne
   architecture. Il faut repartir d'un réseau initialisé aléatoirement à
   chaque changement du nombre de couches.
