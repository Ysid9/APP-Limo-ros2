"""
ros2_torch_run.py
==================
Point d'entrée de l'apprentissage neuronal en boucle fermée — version ROS2.
Adaptation directe de torch_run.py (CoppeliaSim/ZMQ) pour le robot LIMO.

Les modules d'apprentissage sont INCHANGÉS :
    torch_back.py    — réseau de neurones (PioneerNN)
    torch_online.py  — entraîneur en ligne (PyTorchOnlineTrainer)
    monitoring.py    — visualisation temps réel

Seule la couche d'interface change :
    zmq_pioneer_simulation.py  →  ros2_limo_simulation.py

Lancement (Gazebo) :
    # Terminal 1 — serveur Gazebo
    ros2 launch gazebo_ros gzserver.launch.py world:=worlds/empty.world

    # Terminal 2 — description du robot
    ros2 run robot_state_publisher robot_state_publisher \\
        --ros-args -p robot_description:="$(xacro \\
        $(ros2 pkg prefix limo_description)/share/limo_description/urdf/limo_four_diff.xacro)"

    # Terminal 3 — spawn du robot
    ros2 run gazebo_ros spawn_entity.py -entity limo -topic robot_description

    # Terminal 4 — apprentissage
    python3 ros2_torch_run.py

Lancement (robot réel) :
    # Terminal 1 — Lancer les drivers du robot
    ros2 launch limo_bringup limo_start.launch.py OU ros2 launch limo_base limo_base.launch.py

    # Terminal 2 — apprentissage
    python3 ros2_torch_run.py

Topic odométrie :
    Changer ODOM_TOPIC ci-dessous selon l'environnement :
        Gazebo      → '/odometry'
        Robot réel  → '/odom'

Auteur  : Yasser — stage AMARSMER, CERV/LabSTICC, ENIB
Date    : 2026
"""

import json
import threading
import atexit
import time

from ros2_limo_simulation import ROS2LimoSimulation
from torch_back import PioneerNN
from torch_online import PyTorchOnlineTrainer
from monitoring import RobotMonitorAdapter


# ═══════════════════════════════════════════════════════════════════════
# CONFIGURATION  —  modifier ici selon l'environnement
# ═══════════════════════════════════════════════════════════════════════

# Topics ROS2
CMD_VEL_TOPIC = '/cmd_vel'
ODOM_TOPIC    = '/odometry'          # '/odometry' pour Gazebo, '/odom' pour le vrai robot

# Limites du monde pour le monitoring (x_min, x_max, y_min, y_max)
WORLD_BOUNDS = (-10, 10, -10, 10)

# Fichier de sauvegarde des poids
WEIGHTS_FILE = 'last_w_torch.json'


# ═══════════════════════════════════════════════════════════════════════
# Initialisation du robot ROS2
# ═══════════════════════════════════════════════════════════════════════

robot   = ROS2LimoSimulation(cmd_vel_topic=CMD_VEL_TOPIC, odom_topic=ODOM_TOPIC)
monitor = RobotMonitorAdapter(world_bounds=WORLD_BOUNDS)
trainer = None


def cleanup():
    """Nettoyage à la sortie du programme."""
    print('Nettoyage en cours...')
    if hasattr(robot, 'cleanup'):
        robot.cleanup()
    if hasattr(monitor, 'stop_monitoring'):
        monitor.stop_monitoring()

atexit.register(cleanup)
time.sleep(1)   # laisser le temps au nœud ROS2 de se stabiliser


# ═══════════════════════════════════════════════════════════════════════
# Réseau de neurones
# ═══════════════════════════════════════════════════════════════════════

# Note : HL_size est écrasé à 1000 dans torch_back.py (valeur en dur)
HL_size     = 10
input_size  = 3      # erreur normalisée [dx, dy, dtheta]
output_size = 2      # vitesses roues [v_gauche, v_droite]

network = PioneerNN(input_size, HL_size, output_size)


# ═══════════════════════════════════════════════════════════════════════
# Affichage temps réel (monitoring matplotlib)
# ═══════════════════════════════════════════════════════════════════════

display_choice = ''
while display_choice.lower() not in ('y', 'n'):
    display_choice = input('Enable real-time display? (y/n) --> ')

if display_choice.lower() == 'y':
    monitor.start_monitoring()


# ═══════════════════════════════════════════════════════════════════════
# Chargement de poids existants
# ═══════════════════════════════════════════════════════════════════════

choice = input('Do you want to load previous network? (y/n) --> ')
if choice.lower() == 'y':
    with open(WEIGHTS_FILE) as fp:
        json_obj = json.load(fp)
    network.load_weights_from_json(json_obj, network.hidden_size)
    print(f'✅ Poids chargés depuis {WEIGHTS_FILE}')


# ═══════════════════════════════════════════════════════════════════════
# Création du trainer
# ═══════════════════════════════════════════════════════════════════════

monitor_instance = monitor if display_choice.lower() == 'y' else None
trainer = PyTorchOnlineTrainer(robot, network, monitor_instance)

choice = ''
while choice not in ('y', 'n'):
    choice = input('Do you want to learn? (y/n) --> ')
trainer.training = (choice.lower() == 'y')


# ═══════════════════════════════════════════════════════════════════════
# Première cible
# ═══════════════════════════════════════════════════════════════════════

target_input = input('Enter the first target : x y radian --> ')
target = [float(v) for v in target_input.split()]
if len(target) != 3:
    raise ValueError('Need exactly 3 values: x y theta_rad')


# ═══════════════════════════════════════════════════════════════════════
# Boucle principale d'entraînement
# ═══════════════════════════════════════════════════════════════════════

continue_running = True
session_count    = 0

while continue_running:
    session_count += 1
    print(f'\n⚙️  Session d\'entraînement #{session_count}')

    thread = threading.Thread(target=trainer.train, args=(target,))
    trainer.running = True
    thread.start()

    try:
        input('Appuyer sur Entrée pour arrêter la session en cours...')
        trainer.running = False
        thread.join(timeout=5)
        if thread.is_alive():
            print('⚠️  Le thread d\'entraînement ne s\'est pas arrêté à temps.')
    except KeyboardInterrupt:
        print('\nInterruption clavier. Arrêt...')
        trainer.running = False
        thread.join(timeout=5)

    # Sauvegarde des graphiques de la session
    if display_choice.lower() == 'y':
        monitor.save_results(
            f'session_{session_count}_{time.strftime("%Y%m%d_%H%M%S")}'
        )

    # Continuer ou arrêter ?
    choice = ''
    while choice.lower() not in ('y', 'n'):
        choice = input('Do you want to continue? (y/n) --> ')

    if choice.lower() == 'y':
        choice_learning = ''
        while choice_learning.lower() not in ('y', 'n'):
            choice_learning = input('Do you want to learn? (y/n) --> ')
        trainer.training = (choice_learning.lower() == 'y')

        target_input = input(
            'Move the robot to the initial point and enter the new target : x y radian --> '
        )
        target = [float(v) for v in target_input.split()]
        if len(target) != 3:
            raise ValueError('Need exactly 3 values')
    else:
        continue_running = False


# ═══════════════════════════════════════════════════════════════════════
# Sauvegarde des poids
# ═══════════════════════════════════════════════════════════════════════

# Ask to save weights
choice = ''
while choice not in ('y', 'n'):
    choice = input('Do you want to save the weights? (y/n) --> ').strip().lower()

if choice == 'y':
    json_obj = network.save_weights_to_json()
    with open(WEIGHTS_FILE, 'w') as fp:
        json.dump(json_obj, fp)
    timestamped = f'weights_{time.strftime("%Y%m%d_%H%M%S")}.json'
    with open(timestamped, 'w') as fp:
        json.dump(json_obj, fp)
    print(f'✅ Weights saved to {WEIGHTS_FILE} + {timestamped}')
else:
    print('⚠️  Weights NOT saved.')