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
déplacer à la position initiale voulue avant de lancer le script si la
cible n'est pas l'origine.

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
| `Do you want to load previous network? (y/n)` | `y` pour charger `last_w_torch_mcnamu.json` (poids de la dernière session) |
| `Do you want to learn? (y/n)` | `y` pour activer la rétropropagation, `n` pour évaluer sans apprendre |
| `Enter the first target : x y radian` | ex : `0 0 0` — le mode étant holonome, une cible hors de l'axe avant du robot est correctement gérée (pas besoin d'orienter le robot vers la cible au préalable) |

Le robot converge vers la cible. Appuyer sur Entrée pour arrêter la session
en cours. En fin de script, choix de sauvegarder les poids
(`last_w_torch_mcnamu.json`) — supprimer ce fichier pour repartir d'un
réseau initialisé aléatoirement.

Résultats (CSV + graphes) dans `res/gazebo/run_YYYYMMDD_HHMMSS/`.

## Sur robot réel

Deux scripts possibles :

- **`torch_run_ros2.py`** (même script que ci-dessus, répondre `y` à
  "Real robot?") — cible et paramètres au choix à chaque prompt.
- **`torch_run_real.py`** — version rapide : robot réel systématique, cible
  toujours fixée à `(0, 0, 0)`, monitoring activé automatiquement, garde-fous
  de vitesse actifs (`v_lin`/`v_lat` limités à ±0.3 m/s, `v_ang` à ±0.8 rad/s
  dans `ros2_limo_interface.py`). À privilégier pour un test rapide sans
  repasser par tous les prompts.

Avant de lancer, le driver du robot doit tourner **en mode mecanum
explicitement** (`use_mcnamu:=true`, sinon le robot reste en mode
différentiel par défaut et ignore `v_lat`) :
```bash
ros2 launch limo_base limo_base.launch.py use_mcnamu:=true
```
Argument utile : `port_name:=ttyUSB1` si le port série par défaut ne
correspond pas (voir [`ROBOTS.md`](ROBOTS.md) pour le port réel de chaque
robot).

Résultats dans `res/robot/run_YYYYMMDD_HHMMSS/`. En fin de session,
`torch_run_ros2.py`/`torch_run_real.py` proposent d'envoyer ce dossier vers
le PC de dev par `scp` (adresse codée en dur en tête de script — à adapter
si le PC de destination change).

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
