# Installation

Le workflow actif du projet est **Ubuntu + ROS2 + Gazebo**. CoppeliaSim
(`torch_run.py` / `zmq_pioneer_simulation.py`) est un chemin legacy antérieur au
passage sous ROS2, il reste dans le dépôt mais n'est plus maintenu ni testé,
et n'a de toute façon jamais géré le mode mecanum (2 sorties seulement).

## Ubuntu 22.04 + ROS2 : Humble ou Foxy

Cette branche (`mecanum`) doit fonctionner sur **les deux** distributions :
c'est celle utilisée en pratique sur les deux robots réels, l'un en Humble,
l'autre en Foxy (voir [`ROBOTS.md`](ROBOTS.md)). Le driver C++ gère la
différence de header `tf2_geometry_msgs` (`.hpp` sous Humble, `.h` sous
Foxy) via un include conditionnel (`__has_include`), aucune adaptation
manuelle nécessaire, `colcon build` doit passer sur les deux tel quel.

### 1. ROS2 (exemple Humble, remplacer `humble` par `foxy` si besoin)

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

`ros-humble-teleop-twist-keyboard` (remplacer `humble` par `foxy` sur une
machine Foxy) sert à repositionner le robot manuellement entre deux
sessions d'apprentissage sur robot réel (voir [`TRAINING.md`](TRAINING.md)).

### 2. Dépendances Python

```bash
pip install torch matplotlib PyQt5
```

### 3. Workspace ROS2

```bash
mkdir -p ~/ros2_ws/src
cp -r /chemin/vers/ce/depot/limo_ros2 ~/ros2_ws/src/

cd ~/ros2_ws
source /opt/ros/humble/setup.bash   # ou foxy
colcon build

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc   # ou foxy
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Ensuite, voir [`TRAINING.md`](TRAINING.md) pour lancer une session
d'apprentissage.

## macOS

ROS2 et Gazebo ne tournent pas nativement sur macOS. Deux options :

- **Docker/VM Ubuntu 22.04** avec ROS2 (Humble ou Foxy) installé dedans
  (suivre les étapes ci-dessus à l'intérieur), voie recommandée.
- Le devcontainer fourni dans `limo_ros2/.devcontainer/` (Dockerfile déjà
  présent dans le dépôt), non testé dans le cadre de ce projet, à vérifier
  avant de s'y fier.

Le réseau de neurones (PyTorch) et le logging (`data_logger.py`,
`monitoring.py`) n'ont eux-mêmes aucune dépendance ROS2, ils peuvent tourner
sur macOS directement si vous voulez juste inspecter/rejouer des CSV de
sessions déjà enregistrées avec `plot_results.py`.

## Note : CoppeliaSim (legacy, non maintenu, pas de mode mecanum)

`torch_run.py` charge une scène CoppeliaSim (`simu.ttt`) qui n'est **pas**
versionnée dans ce dépôt (exclue via `.gitignore`). Ce chemin ne gère de
toute façon que 2 sorties (robot Pioneer différentiel), sans intérêt pour
tester le mode mecanum. Privilégier le robot réel.
