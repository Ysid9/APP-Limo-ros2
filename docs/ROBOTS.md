# Prise en main des robots physiques

Deux robots Limo réels sur le réseau local, utilisateur `agilex`,
**authentification par mot de passe uniquement** (pas de clé SSH configurée
au moment de la rédaction, chaque `scp`/`ssh` redemande le mot de passe).

| | LIMO | LIMO COBOT |
|---|---|---|
| IP | `192.168.1.111` | `192.168.1.113` |
| Distribution ROS2 | Humble (sourcée directement dans `~/.bashrc`) | Foxy, **dual-boot ROS1/ROS2** : `~/.bashrc` demande "ros: noetic(1) foxy(2) ?" à l'ouverture de chaque terminal, choisir **2** |
| Workspace ROS2 actif | `~/agilex_ws` | `~/limo_ros2_ws` (`~/agilex_ws` sur cette machine est un workspace **ROS1** différent, paquet `limo_ros`, pas `limo_ros2` ; ne pas confondre) |
| Scripts Python (mecanum) | `~/mecanum_torch` | `~/mecanum_torch` |
| Scripts Python (diff) | `~/Downloads/APP-Limo_ros2_curr` | `~/Downloads/APP-Limo_ros2_curr` |
| Port série MCU | `/dev/ttylimo` (lien udev persistant) | `/dev/ttylimo` (lien udev persistant) |

Chaque robot n'a que ces emplacements-là pour le code, pas de copies
parallèles/obsolètes à ce jour (nettoyage fait en 2026-09).

Le port série par défaut de `limo_base.launch.py` est `ttylimo`, un lien
symbolique udev qui pointe vers le bon périphérique physique sur chaque
robot. Il fonctionne tel quel sur les deux robots, pas besoin de préciser
`port_name` au lancement.

PC de développement : `cerv@192.168.1.241`. Les résultats de session
(`res/robot/run_.../`) restent en local sur la machine qui a lancé le
script, pas d'envoi automatique. Pour les récupérer manuellement :
```bash
scp -r agilex@192.168.1.111:~/mecanum_torch/res/robot/run_XXXX ./
```

## Déployer une modification du driver C++ (`limo_ros2`)

1. Éditer les fichiers dans le dépôt local (PC de dev), sur la bonne branche.
2. Copier chaque fichier modifié individuellement vers le chemin correspondant
   sous le workspace ROS2 **actif** du robot ciblé (tableau ci-dessus, pas
   dans `install/` qui est régénéré par colcon) :
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
   # LIMO (Humble)
   ssh agilex@192.168.1.111 "source /opt/ros/humble/setup.bash && cd ~/agilex_ws && colcon build --packages-select limo_base"

   # LIMO COBOT (Foxy)
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
`~/Downloads/APP-Limo_ros2_curr` selon la branche. Pas de compilation
nécessaire, les changements sont actifs au prochain lancement du script.

## Lancer le driver

Le driver de base seul (`limo_base.launch.py`) gère odométrie et moteurs.
Pour la version complète (driver de base + lidar + transformées
statiques), la commande diffère selon le robot :

```bash
# LIMO
ros2 launch limo_bringup limo_start.launch.py

# LIMO COBOT
ros2 launch limo_base limo_start.launch.py
```

## Activer le mode mecanum sur un robot

Le driver gère 3 modes (`MODE_FOUR_DIFF`, `MODE_ACKERMANN`, `MODE_MCNAMU`)
dans le même binaire. Le mode est soit imposé par le firmware du MCU au
démarrage, soit forcé par le paramètre `use_mcnamu` du launch ROS2 :
```bash
ros2 launch limo_base limo_base.launch.py use_mcnamu:=true
```
Voir `limo_driver.cpp::enableMcMode()`, qui envoie une trame de config au
MCU pour basculer le mode moteur au démarrage du nœud.

## Configuration réseau sur LIMO COBOT

LIMO COBOT fonctionne uniquement en découverte ROS2 locale
(`ROS_LOCALHOST_ONLY=1`, déjà dans `~/.bashrc`) : une exposition au trafic
réseau normal fait planter `limo_base` de façon intermittente (bug de
parsing dans CycloneDDS, déclenché par un paquet de découverte d'un autre
participant du réseau). Conséquence pratique : les scripts doivent tourner
directement sur ce robot (via SSH), jamais depuis le PC à distance. LIMO
n'a pas cette contrainte.
