# Reproduire les expériences avec Docker

L'image Docker fournit un environnement identique à celui décrit dans
[`INSTALL.md`](INSTALL.md) (Ubuntu 22.04, ROS2 Humble, Gazebo, PyTorch),
pour lancer une session d'apprentissage en simulation (voir
[`TRAINING.md`](TRAINING.md)) de façon reproductible sur n'importe quelle
machine Linux avec Docker installé.

## Prérequis

- Docker et Docker Compose (`docker compose version`)
- Un serveur X11 (poste Linux classique). Autoriser le conteneur à s'y
  connecter, une fois par session :
  ```bash
  xhost +local:docker
  ```

## Construire l'image

```bash
docker compose build
```

Compile l'image (installation ROS2/Gazebo/PyTorch + `colcon build` du
paquet `limo_ros2`).

## Lancer le conteneur

```bash
docker compose up -d
```

Le dossier `res/` du dépôt est monté dans le conteneur : les résultats
(CSV, graphes, poids) écrits par les scripts apparaissent directement sur
la machine hôte, dans `res/gazebo/run_YYYYMMDD_HHMMSS/`.

## Lancer une session (2 terminaux, comme dans TRAINING.md)

**Terminal 1, Gazebo :**
```bash
docker exec -it app-limo-diff bash
ros2 launch limo_description gazebo_models_diff.launch.py
```

**Terminal 2, l'apprentissage :**
```bash
docker exec -it app-limo-diff bash
python3 torch_run_ros2.py
```

ROS2, Gazebo et le workspace sont déjà sourcés automatiquement dans chaque
`docker exec -it ... bash` (ajouté à `/etc/bash.bashrc` à la construction de
l'image). Suivre ensuite les prompts décrits dans
[`TRAINING.md`](TRAINING.md).

## Arrêter / nettoyer

```bash
docker compose down
```

## Reproductibilité : versions figées

Pour que tout le monde obtienne les mêmes résultats numériques :

- **Image de base** (`Dockerfile`) figée par digest (`osrf/ros:humble-desktop@sha256:...`,
  Ubuntu 22.04 + `ros-humble-desktop`). Les paquets Gazebo/ROS installés
  par-dessus (`ros-humble-gazebo-ros-pkgs`,
  `ros-humble-joint-state-publisher-gui`, `ros-humble-teleop-twist-keyboard`)
  ne sont volontairement **pas** figés à une version précise : l'archive apt
  de ROS ne conserve que les révisions de build récentes et purge les
  anciennes en continu (testé : une version installée quelques mois plus tôt
  n'y était déjà plus), un épinglage exact casserait donc le build au
  premier rebuild. On reste sur la même ligne Humble / Gazebo 11, la plus
  récente disponible au moment du build — en pratique ce sont les
  dépendances Python ci-dessous qui font la plus grosse différence sur les
  résultats numériques.
- **Dépendances Python** (`requirements.txt`) : `torch`, `numpy`,
  `matplotlib`, `PyQt5`, `plotly` figés aux versions de l'environnement de
  référence. PyTorch tourne en CPU (le code du projet n'appelle jamais
  `.cuda()`/`.to(device)`, un GPU n'apporterait rien ici et forcerait
  `nvidia-container-toolkit` sur toutes les machines de l'équipe).

Ces deux fichiers sont ce qu'il faut modifier pour changer une version ;
éviter de faire `pip install` ou `apt install` à la main dans un conteneur
en cours d'exécution, ces changements ne survivraient pas à un rebuild et
ne seraient pas partagés avec le reste de l'équipe.
