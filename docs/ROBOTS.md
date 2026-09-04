# Prise en main des robots physiques

Deux robots Limo réels sur le réseau local, utilisateur `agilex`,
**authentification par mot de passe uniquement** (pas de clé SSH configurée
au moment de la rédaction — chaque `scp`/`ssh` redemande le mot de passe).

| | `.111` | `.113` |
|---|---|---|
| IP | `192.168.1.111` | `192.168.1.113` |
| Distribution ROS2 | Humble (sourcée directement dans `~/.bashrc`) | Foxy — **dual-boot ROS1/ROS2** : `~/.bashrc` demande "ros: noetic(1) foxy(2) ?" à l'ouverture de chaque terminal, choisir **2** |
| Workspace ROS2 actif | `~/agilex_ws` | `~/limo_ros2_ws` ⚠️ `~/agilex_ws` sur cette machine est un workspace **ROS1** différent (paquet `limo_ros`, pas `limo_ros2`) — ne pas confondre |
| Scripts Python (mecanum) | `~/mecanum_torch` | `~/mecanum_torch` |
| Scripts Python (diff) | `~/Downloads/APP-Limo_ros2_curr` | `~/Downloads/APP-Limo_ros2_curr` |
| Port série MCU | `/dev/ttyUSB1` (pas `ttyUSB0`) | non vérifié |

Chaque robot n'a que ces emplacements-là pour le code — pas de copies
parallèles/obsolètes à ce jour (nettoyage fait en 2026-09).

PC de développement : `cerv@192.168.1.241` — les scripts robot y renvoient
leurs résultats de session par `scp` en fin de run (adresse codée en dur
dans `torch_run_ros2.py`/`torch_run_real.py`, à adapter si ce PC change).

## Déployer une modification du driver C++ (`limo_ros2`)

1. Éditer les fichiers dans le dépôt local (PC de dev), sur la bonne branche.
2. Copier chaque fichier modifié individuellement vers le chemin correspondant
   sous le workspace ROS2 **actif** du robot ciblé (tableau ci-dessus — pas
   dans `install/`, régénéré par colcon) :
   ```bash
   scp limo_ros2/limo_base/src/limo_driver.cpp \
       agilex@192.168.1.111:~/agilex_ws/src/limo_ros2/limo_base/src/limo_driver.cpp
   ```
3. Vérifier l'intégrité du transfert avant de reconstruire :
   ```bash
   md5sum limo_ros2/limo_base/src/limo_driver.cpp
   ssh agilex@192.168.1.111 "md5sum ~/agilex_ws/src/limo_ros2/limo_base/src/limo_driver.cpp"
   ```
4. Reconstruire sur le robot, après avoir sourcé la bonne distribution ROS2 :
   ```bash
   # .111 (Humble)
   ssh agilex@192.168.1.111 "source /opt/ros/humble/setup.bash && cd ~/agilex_ws && colcon build --packages-select limo_base"

   # .113 (Foxy)
   ssh agilex@192.168.1.113 "source /opt/ros/foxy/setup.bash && cd ~/limo_ros2_ws && colcon build --packages-select limo_base"
   ```

Pour un changement plus large (nouveaux fichiers, plusieurs paquets), il est
plus simple de resynchroniser tout `limo_ros2/` :
```bash
ssh agilex@192.168.1.111 "rm -rf ~/agilex_ws/src/limo_ros2"
scp -r limo_ros2 agilex@192.168.1.111:~/agilex_ws/src/limo_ros2
ssh agilex@192.168.1.111 "source /opt/ros/humble/setup.bash && cd ~/agilex_ws && colcon build --packages-select limo_base limo_description limo_msgs"
```

## Déployer une modification des scripts Python

Même principe, `scp` direct vers `~/mecanum_torch` ou
`~/Downloads/APP-Limo_ros2_curr` selon la branche — pas de compilation
nécessaire, les changements sont actifs au prochain lancement du script.

## Activer le mode mecanum sur un robot

Le driver gère 3 modes (`MODE_FOUR_DIFF`, `MODE_ACKERMANN`, `MODE_MCNAMU`)
dans le même binaire — le mode est soit imposé par le firmware du MCU au
démarrage, soit forcé par le paramètre `use_mcnamu` du launch ROS2 :
```bash
ros2 launch limo_base limo_base.launch.py use_mcnamu:=true
```
Voir `limo_driver.cpp::enableMcMode()` — envoie une trame de config au MCU
pour basculer le mode moteur au démarrage du nœud.

## Pièges connus (déjà rencontrés, déjà corrigés dans le code — pour info)

- Le driver (`limo_driver.cpp`, code fournisseur AgileX) avait un bug
  d'initialisation du cap odométrique (`real_theta_`) qui rendait
  l'odométrie incohérente au démarrage sur certaines machines selon ce qui
  traînait en mémoire — corrigé (voir historique git, commit "Fix odometry
  heading bugs in limo_driver.cpp").
- `tf2_geometry_msgs` : le header s'appelle `.hpp` sous Humble, `.h` sous
  Foxy — le driver utilise `__has_include` pour supporter les deux
  distributions avec le même code source.
