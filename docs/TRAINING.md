# Lancer une session d'apprentissage / de test

Branche `diff` — robot différentiel, réseau à 2 sorties (`v_lin`, `v_ang`).

## En simulation (Gazebo)

**Terminal 1 — lancer Gazebo :**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch limo_description gazebo_models_diff.launch.py
```
Le robot Limo apparaît à la position (0, 0, 0). Le déplacer à la position
initiale voulue (dans Gazebo, avant de lancer le script) si besoin.

**Terminal 2 — lancer le trainer :**
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd /chemin/vers/ce/depot
python3 torch_run_ros2.py
```

Prompts :
| Prompt | Réponse |
|---|---|
| `Real robot? (y/n)` | `n` pour Gazebo |
| `Enable real-time display? (y/n)` | `y` pour afficher les graphes de monitoring en direct |
| `Do you want to load previous network? (y/n)` | `y` pour charger `last_w_torch_diff.json` (poids de la dernière session) — **`n` repart d'un réseau initialisé aléatoirement** (pas de reprise) |
| `Do you want to learn? (y/n)` | `y` pour activer la rétropropagation, `n` pour évaluer sans apprendre |
| `Enter the first target : x y radian` | **toujours `0 0 0`** |

Le robot converge vers la cible. Appuyer sur Entrée pour arrêter la session
en cours. Entre deux sessions, repositionner le robot manuellement dans
Gazebo si besoin. En fin de script, choix de sauvegarder les poids
(`last_w_torch_diff.json`) — supprimer ce fichier pour repartir d'un réseau
initialisé aléatoirement au prochain lancement, même en répondant `y`.

Résultats (CSV + graphes) dans `res/gazebo/run_YYYYMMDD_HHMMSS/`.

## Sur robot réel

**Trois terminaux nécessaires**, dans cet ordre :

1. **Driver du robot** :
   ```bash
   ros2 launch limo_base limo_base.launch.py
   ```
   Sur `.113`, ajouter `port_name:=ttyUSB0` (voir [`ROBOTS.md`](ROBOTS.md)) :
   ```bash
   ros2 launch limo_base limo_base.launch.py port_name:=ttyUSB0
   ```
2. **Téléopération** (pour repositionner le robot entre les sessions) :
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   (voir [`INSTALL.md`](INSTALL.md) pour l'installer si absent)
3. **Apprentissage** — `torch_run_ros2.py` (prompts complets, cible **toujours
   `0 0 0`**) ou `torch_run_real.py` (version rapide : robot réel et cible
   `(0,0,0)` systématiques, monitoring auto, pas de prompts de config) :
   ```bash
   python3 torch_run_ros2.py     # ou torch_run_real.py
   ```

**Procédure d'une session de terrain :**
1. Positionner le robot physiquement à `(0, 0, 0)` (repère de référence, terminal 2 = téléop).
2. Lancer le driver (terminal 1).
3. Déplacer le robot avec la téléop vers un point de départ éloigné de la cible, puis lancer l'apprentissage (terminal 3).
4. Dès que le robot atteint la cible, appuyer sur Entrée dans le terminal 3 pour arrêter la session en cours.
5. Répondre `y` à "Do you want to continue?", repositionner le robot avec la téléop, relancer une nouvelle session.
6. Répéter 3-5 autant de fois que nécessaire.

Résultats dans `res/robot/run_YYYYMMDD_HHMMSS/` (restent en local sur la
machine qui a lancé le script).

## Rejouer un CSV existant

```bash
python3 plot_results.py res/gazebo/run_20260601_143022/training_session_1_20260601_143045.csv
```

## Paramètres modifiables (`torch_run_ros2.py`)

```python
HL_size       = 1000   # taille de la couche cachée du réseau
LEARNING_RATE = 0.2    # pas d'apprentissage
```

### Ajouter une couche cachée supplémentaire

Le réseau (`torch_back.py`, classe `PioneerNN`) n'a qu'une seule couche
cachée. En ajouter une deuxième demande 3 modifications :

1. Dans `__init__`, ajouter la couche et son initialisation :
   ```python
   self.hidden2 = nn.Linear(hidden_size, hidden_size2)
   nn.init.xavier_uniform_(self.hidden2.weight)
   nn.init.zeros_(self.hidden2.bias)
   ```
2. Dans `forward`, l'insérer entre la couche cachée existante et la sortie :
   ```python
   x = self.activation(self.hidden(x))
   x = self.activation(self.hidden2(x))
   x = self.activation(self.output(x))
   ```
3. **`load_weights_from_json`/`save_weights_to_json`** sont écrits pour
   exactement 2 couches (clés `"input_weights"`/`"output_weights"`) — il
   faut y ajouter une clé `"hidden2_weights"` gérée symétriquement, sinon
   les fichiers `last_w_torch_*.json` existants ne sont plus compatibles
   avec le nouveau réseau (il faudra repartir de poids aléatoires).
