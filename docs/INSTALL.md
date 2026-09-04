# Installation

## Ubuntu 22.04 + ROS2 Humble (poste de développement)

Cette branche (`diff`) et la branche `ackermann` ciblent Humble. La branche
`mecanum` doit aussi fonctionner sur ROS2 Foxy (utilisé sur l'un des deux
robots réels, voir [`ROBOTS.md`](ROBOTS.md)), le driver C++ gère les deux
via un include conditionnel (`__has_include`).

### 1. ROS2 Humble

```bash
sudo apt install software-properties-common curl

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-joint-state-publisher-gui \
  ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions
```

`ros-humble-teleop-twist-keyboard` sert à repositionner le robot manuellement
entre deux sessions d'apprentissage sur robot réel (voir
[`TRAINING.md`](TRAINING.md)).

### 2. Dépendances Python

```bash
pip install torch matplotlib PyQt5
```

### 3. Workspace ROS2

```bash
mkdir -p ~/ros2_ws/src
cp -r /chemin/vers/ce/depot/limo_ros2 ~/ros2_ws/src/

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Ensuite, voir [`TRAINING.md`](TRAINING.md) pour lancer Gazebo et une session
d'apprentissage.

## macOS

ROS2 et Gazebo ne tournent pas nativement sur macOS. Deux options :

- **Docker/VM Ubuntu 22.04** avec ROS2 Humble installé dedans (suivre les
  étapes ci-dessus à l'intérieur), c'est la voie recommandée pour continuer
  ce travail sur Mac.
- Le devcontainer fourni dans `limo_ros2/.devcontainer/` (Dockerfile déjà
  présent dans le dépôt), non testé dans le cadre de ce projet, à vérifier
  avant de s'y fier.

Le réseau de neurones (PyTorch) et le logging (`data_logger.py`,
`monitoring.py`) n'ont eux-mêmes aucune dépendance ROS2, ils peuvent tourner
sur macOS directement si vous voulez juste inspecter/rejouer des CSV de
sessions déjà enregistrées avec `plot_results.py`.
