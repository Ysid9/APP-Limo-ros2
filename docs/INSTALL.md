# Installation

## Ubuntu 22.04 + ROS2 : Humble ou Foxy

Les 3 branches (`diff`, `mecanum`, `ackermann`) fonctionnent aussi bien sous
Humble que sous Foxy, les deux robots
Limo utilisent l'un Humble, l'autre Foxy (voir [`ROBOTS.md`](ROBOTS.md)).

### 1. ROS2 (exemple Humble, remplacer `humble` par `foxy` si besoin)

```bash
# outils nécessaires pour ajouter un dépôt apt tiers
sudo apt install software-properties-common curl

# récupère la clé de signature du dépôt ROS2
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# ajoute le dépôt ROS2 aux sources apt
echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

# rafraîchit la liste des paquets disponibles
sudo apt update

# installe ROS2 Humble, Gazebo, l'affichage des joints, la téléop clavier et colcon
# (remplacer tous les ros-humble-* par ros-foxy-* sur une machine Foxy ;
#  python3-colcon-common-extensions ne change pas, il n'est pas lié à une distribution)
sudo apt install \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-joint-state-publisher-gui \
  ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions
```

`ros-humble-teleop-twist-keyboard` (remplacer `humble` par `foxy` sur une
machine Foxy) sert à repositionner le robot manuellement entre deux
sessions d'apprentissage sur robot réel (voir [`TRAINING.md`](TRAINING.md)).

### 2. Python

```bash
# installe Python 3 et pip
sudo apt install python3 python3-pip

# dépendances du projet
pip install torch matplotlib PyQt5
```

### 3. Workspace ROS2

```bash
# crée le workspace ROS2 et y copie le paquet du dépôt
mkdir -p ~/ros2_ws/src
cp -r /chemin/vers/ce/depot/limo_ros2 ~/ros2_ws/src/

# installe automatiquement les dépendances système déclarées par les
# paquets ROS2 (tf2, turtlesim, xacro, rviz2, etc.), au cas où l'une
# d'elles ne serait pas déjà couverte par ros-*-desktop
sudo apt install python3-rosdep
sudo rosdep init   # une seule fois par machine, ignorer l'erreur si déjà fait
rosdep update
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# charge l'environnement ROS2 et compile le paquet
source /opt/ros/humble/setup.bash   # ou foxy
colcon build

# source ROS2 et le workspace automatiquement à chaque ouverture de terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc   # ou foxy
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Ensuite, voir [`TRAINING.md`](TRAINING.md) pour lancer une session
d'apprentissage.

### 4. Dépendances optionnelles (scripts de visualisation)

Pas nécessaires pour l'apprentissage/test (`torch_run_ros2.py`,
`torch_run_real.py`) : seuls `odom_plot.py` (tracé d'odométrie en direct) et
`visualize_cost.py` (paysage de coût appris) en ont besoin.

```bash
sudo apt install python3-tk
pip install numpy plotly
```
