# Reproducible environment for APP-Limo (branche diff) : ROS2 Humble + Gazebo
# + PyTorch, pour rejouer les sessions d'apprentissage en simulation exactement
# comme décrit dans docs/INSTALL.md et docs/TRAINING.md.

# Digest figé de osrf/ros:humble-desktop (Ubuntu 22.04 + ros-humble-desktop),
# pour que le build reste identique même si le tag "humble-desktop" bouge.
FROM osrf/ros:humble-desktop@sha256:fb07245b32187d74350be25323d8ad2f8ca5c25c325759911a1eff2267a49c1e

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

# Paquets système : mêmes que docs/INSTALL.md (Gazebo, affichage des joints,
# téléop clavier, colcon) + python3-tk pour les scripts de visualisation
# optionnels (odom_plot.py, visualize_cost.py).
# Pas de version figée ici (contrairement au FROM ci-dessus) : l'archive apt
# de ROS ne conserve que les révisions de build récentes de chaque paquet et
# purge les anciennes en continu (vérifié : les révisions installées sur la
# machine de référence il y a quelques mois n'y sont déjà plus), donc un
# épinglage exact casserait le build au premier rebuild. On reste sur la
# même ligne Humble / Gazebo 11, la plus récente disponible au moment du
# build.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-pip \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Workspace ROS2 : uniquement le paquet limo_ros2 (docs/INSTALL.md section 3).
# Compilé AVANT l'installation des dépendances Python du projet ci-dessous :
# pip installe un setuptools récent qui casse la génération des bindings
# Python de limo_msgs (conflit avec le module `packaging` système,
# canonicalize_version() ne supporte pas strip_trailing_zero) si on
# l'installe avant colcon build.
RUN mkdir -p /root/ros2_ws/src
COPY limo_ros2 /root/ros2_ws/src/limo_ros2

WORKDIR /root/ros2_ws
RUN . /opt/ros/humble/setup.sh \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build \
    && rm -rf /var/lib/apt/lists/*

# Dépendances Python du projet, versions figées (requirements.txt) :
# --upgrade est nécessaire, sinon pip considère numpy/matplotlib/PyQt5 déjà
# satisfaits par les versions apt (dépendances de rqt/rviz2) et les laisse
# à leur ancienne version au lieu de les remplacer par celles demandées.
COPY requirements.txt /root/requirements.txt
RUN pip3 install --no-cache-dir --upgrade -r /root/requirements.txt

# Code d'apprentissage (torch_*.py, data_logger.py, monitoring.py, etc.),
# copié à part du workspace ROS2 comme dans le dépôt.
WORKDIR /root/app
COPY *.py ./

# Source ROS2 + le workspace pour toute commande dans le conteneur :
# - /etc/bash.bashrc pour les shells interactifs (`docker exec -it ... bash`,
#   utilisé pour lancer Gazebo et le trainer dans deux terminaux séparés) ;
# - l'entrypoint pour la commande CMD/exécutée directement (`docker run ...`).
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo "source /root/ros2_ws/install/setup.bash" >> /etc/bash.bashrc

COPY docker-entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
