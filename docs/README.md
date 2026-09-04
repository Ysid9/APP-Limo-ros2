# APP-Limo — contrôleur en ligne (apprentissage par gradient) pour robot AgileX Limo

Ce dépôt contient un contrôleur de position par réseau de neurones entraîné en ligne
(descente de gradient manuelle sur une fonction de coût quadratique position+orientation),
appliqué à un robot mobile AgileX Limo — en simulation (Gazebo) ou sur robot réel.

Le même algorithme d'apprentissage existe en plusieurs variantes selon le mode de
déplacement du robot, chacune sur sa propre branche git :

| Branche | Mode | Sorties réseau | État |
|---|---|---|---|
| [`diff`](../../tree/diff) | Différentiel (2 roues motrices, non-holonome) | `v_lin, v_ang` (2) | Référence, stable |
| [`mecanum`](../../tree/mecanum) | Mecanum (4 roues, holonome) | `v_lin, v_lat, v_ang` (3) | Référence, stable |
| [`ackermann`](../../tree/ackermann) | Ackermann (direction façon voiture) | `v_lin, v_ang` (2) | **Non commencé** |

**Vous êtes sur la branche `ackermann`.** Elle a été créée depuis `diff` (même
lignage historique : le projet avait un mode Ackermann avant le commit
"Retour diff drive — suppression Ackermann", supprimé au profit du différentiel).
Pour l'instant cette branche est identique à `diff` — le contenu de `docs/`
ci-dessous est hérité tel quel et **reste à adapter** au fur et à mesure du
travail Ackermann.

Ce qui existe déjà côté driver C++ (`limo_ros2/limo_base/src/limo_driver.cpp`,
mode `MODE_ACKERMANN`) et a été vérifié mathématiquement correct : les
fonctions `convertInnerAngleToCentral`/`convertCentralAngleToInner` (relation
angle roue interne <-> angle de braquage central, cinématique bicyclette
standard). Pas encore vérifié/adapté : `torch_online.py` (gradient calculé
pour un robot différentiel, à revoir pour la cinématique Ackermann réelle),
`ros2_limo_interface.py`, et les modèles Gazebo (`limo_car/` existe dans le
dépôt mais a été mis de côté par le passé — voir
[Ackermann Gazebo Model, notes de session antérieures] pourquoi).

## Pour commencer

1. [`INSTALL.md`](INSTALL.md) — installer l'environnement (hérité de `diff`, à vérifier)
2. [`TRAINING.md`](TRAINING.md) — hérité de `diff`, **ne reflète pas encore Ackermann** (pas de launch Gazebo Ackermann documenté ici)
3. [`ROBOTS.md`](ROBOTS.md) — connexion aux deux robots physiques (contenu identique aux autres branches)

## Structure du dépôt (héritée de `diff`)

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
                          (limo_car/ = modèle Ackermann Gazebo, présent mais pas encore
                          raccordé/testé sur cette branche)
```
