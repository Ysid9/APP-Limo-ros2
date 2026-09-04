# APP-Limo : contrôleur en ligne (apprentissage par gradient) pour robot AgileX Limo

Ce dépôt contient un contrôleur de position par réseau de neurones entraîné en ligne
(descente de gradient manuelle sur une fonction de coût quadratique position+orientation),
appliqué à un robot mobile AgileX Limo, en simulation (Gazebo) ou sur robot réel.

Le même algorithme d'apprentissage existe en plusieurs variantes selon le mode de
déplacement du robot, chacune sur sa propre branche git :

| Branche | Mode | Sorties réseau | Simulation Gazebo | État |
|---|---|---|---|---|
| [`diff`](../../tree/diff) | Différentiel (2 roues motrices, non-holonome) | `v_lin, v_ang` (2) | Oui | Référence, stable |
| [`mecanum`](../../tree/mecanum) | Mecanum (4 roues, holonome) | `v_lin, v_lat, v_ang` (3) | Non, pas de modèle officiel | Référence, stable sur robot réel |
| [`ackermann`](../../tree/ackermann) | Ackermann (direction façon voiture) | `v_lin, v_ang` (2) | Non, pas de modèle officiel | Non commencé, branché depuis `diff` |

Il n'existe pas de modèle Gazebo officiel pour l'Ackermann ni le mecanum, la
simulation n'est pas possible pour ces deux modes actuellement, seul le robot
réel permet de tester.

**Vous êtes sur la branche `diff`.** Le code d'apprentissage (`torch_*.py`,
`data_logger.py`, `monitoring.py`) est quasi identique sur les 3 branches ; ce qui
change, c'est le nombre de sorties du réseau, la cinématique du robot dans
`limo_ros2/limo_base/src/limo_driver.cpp`, et les modèles Gazebo utilisés.

## Pour commencer

1. [`INSTALL.md`](INSTALL.md) : installer l'environnement (ROS2, Gazebo, dépendances Python)
2. [`TRAINING.md`](TRAINING.md) : lancer une session d'apprentissage/test, en simulation ou sur robot réel
3. [`ROBOTS.md`](ROBOTS.md) : se connecter aux robots physiques et déployer une modification du driver

## Structure du dépôt

```
torch_back.py            réseau PioneerNN (1 couche cachée, tanh, PyTorch)
torch_online.py          boucle d'apprentissage en ligne (coût, gradient, backward manuel)
data_logger.py           logging CSV des sessions
monitoring.py            visualisation temps réel (matplotlib)
ros2_limo_interface.py   interface ROS2 (/odom -> position, /cmd_vel -> commande)
zmq_pioneer_simulation.py interface CoppeliaSim -- LEGACY, voir note dans INSTALL.md
torch_run.py             lancement CoppeliaSim (Pioneer) -- LEGACY
torch_run_ros2.py        lancement ROS2 (Gazebo ou robot réel, au choix au démarrage)
torch_run_real.py        lancement rapide sur robot réel (cible fixe à l'origine)
plot_results.py          regénérer un graphe à partir d'un CSV de session existant
limo_ros2/               paquet ROS2 : driver C++ bas niveau, description URDF/xacro, messages
```
