# Installation

## Ubuntu 22.04 + ROS2 : Humble ou Foxy

Cette branche (`mecanum`) doit fonctionner sur **les deux** distributions :
c'est celle utilisée en pratique sur les deux robots réels, l'un en Humble,
l'autre en Foxy.

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

### 2. Dépendances Python

```bash
pip install torch matplotlib PyQt5
```

### 3. Workspace ROS2

```bash
# crée le workspace ROS2 et y copie le paquet du dépôt
mkdir -p ~/ros2_ws/src
cp -r /chemin/vers/ce/depot/limo_ros2 ~/ros2_ws/src/

# charge l'environnement ROS2 et compile le paquet
cd ~/ros2_ws
source /opt/ros/humble/setup.bash   # ou foxy
colcon build

# source ROS2 et le workspace automatiquement à chaque ouverture de terminal
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc   # ou foxy
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Ensuite, voir [`TRAINING.md`](TRAINING.md) pour lancer une session
d'apprentissage.
