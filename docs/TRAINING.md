# Lancer une session d'apprentissage / de test

Branche `diff` — robot différentiel, réseau à 2 sorties (`v_lin`, `v_ang`).

## En simulation (Gazebo)

**Terminal 1 — lancer Gazebo :**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch limo_description gazebo_models_diff.launch.py
```
Le robot Limo apparaît à la position (0, 0, 0). Le déplacer à la position
initiale voulue (dans Gazebo, avant de lancer le script) si la cible n'est
pas l'origine.

**Terminal 2 — lancer le trainer :**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd /chemin/vers/ce/depot
python3 torch_run_ros2.py
```

Prompts :
| Prompt | Réponse |
|---|---|
| `Real robot? (y/n)` | `n` pour Gazebo |
| `Enable real-time display? (y/n)` | `y` pour afficher les graphes de monitoring en direct |
| `Do you want to load previous network? (y/n)` | `y` pour charger `last_w_torch_3in.json` (poids de la dernière session) |
| `Do you want to learn? (y/n)` | `y` pour activer la rétropropagation, `n` pour évaluer sans apprendre |
| `Enter the first target : x y radian` | ex : `0 0 0` |

Le robot converge vers la cible. Appuyer sur Entrée pour arrêter la session
en cours. Entre deux sessions, repositionner le robot manuellement dans
Gazebo si besoin. En fin de script, choix de sauvegarder les poids
(`last_w_torch_3in.json`) — supprimer ce fichier pour repartir d'un réseau
initialisé aléatoirement.

Résultats (CSV + graphes) dans `res/gazebo/run_YYYYMMDD_HHMMSS/`.

## Sur robot réel

Deux scripts possibles :

- **`torch_run_ros2.py`** (même script que ci-dessus, répondre `y` à
  "Real robot?") — cible et paramètres au choix à chaque prompt.
- **`torch_run_real.py`** — version rapide : robot réel systématique, cible
  toujours fixée à `(0, 0, 0)`, monitoring activé automatiquement, garde-fous
  de vitesse actifs (`safety_limits=True` dans `ros2_limo_interface.py`).
  À privilégier pour un test rapide sans repasser par tous les prompts.

Avant de lancer, le nœud driver du robot doit tourner (voir
[`ROBOTS.md`](ROBOTS.md) pour la connexion/le déploiement) :
```bash
ros2 launch limo_base limo_base.launch.py
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
python3 plot_results.py res/gazebo/run_20260601_143022/training_session_1_20260601_143045.csv
```

## Paramètres modifiables (`torch_run_ros2.py`)

```python
HL_size       = 1000   # taille de la couche cachée du réseau
LEARNING_RATE = 0.2    # pas d'apprentissage
```
