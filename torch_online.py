import time
import math
import torch
import numpy as np

def theta_s(x, y):
    return math.tanh(10.*x)*math.atan(1.*y)

class PyTorchOnlineTrainer:
    def __init__(self, robot, nn_model, monitor=None):
        """
        Args:
            robot (Robot): instance du robot suivant le modèle de ZMQPioneerSimulation
            nn_model (PioneerNN): modèle PyTorch du réseau de neurones
        """
        self.robot = robot
        self.network = nn_model
        # Facteurs de normalisation pour x et y
        self.alpha = [1/6, 1/6, 1/(math.pi)]

        # État de l'apprentissage
        self.running = False
        self.training = False

        # Add monitor support
        self.monitor = monitor
        self.last_gradient = [0, 0] if monitor else None


    def train(self, target):
        """
        Procédure d'apprentissage en ligne

        Args:
            target (list): position cible [x, y, theta]
        """

        # Update monitor if available
        if self.monitor:
            self.monitor.set_target(target)

        # Obtenir la position actuelle du robot
        position = self.robot.get_position()

        # Calculer l'entrée du réseau : [err_x, err_y, sin(e_theta), cos(e_theta)-1]
        # atan2 recadre e_theta dans [-π, π] ; cos-1 assure l'attracteur naturel à la cible
        e_theta = math.atan2(math.sin(position[2] - target[2] - theta_s(position[0], position[1])),
                             math.cos(position[2] - target[2] - theta_s(position[0], position[1])))
        network_input = [
            (position[0] - target[0]) * self.alpha[0],
            (position[1] - target[1]) * self.alpha[1],
            math.sin(e_theta),
            math.cos(e_theta) - 1,
        ]

        # Boucle d'apprentissage
        while self.running:
            # Mesurer le temps de début pour le calcul du delta_t
            debut = time.time()

            # Forward pass - obtenir les commandes de vitesse des roues
            input_tensor = torch.tensor(network_input, dtype=torch.float32)
            if self.training:
                input_tensor.requires_grad_(True)
                command = self.network(input_tensor).tolist()
            else:
                with torch.no_grad():
                    command = self.network(input_tensor).tolist()

            # Calculer le critère avant de déplacer le robot
            alpha_x = 1/6
            alpha_y = 1/6
            alpha_teta = 1.0/(math.pi)

            e_theta = math.atan2(math.sin(position[2] - target[2] - theta_s(position[0], position[1])),
                                 math.cos(position[2] - target[2] - theta_s(position[0], position[1])))
            crit_av = (alpha_x * alpha_x * (position[0] - target[0])**2 +
                       alpha_y * alpha_y * (position[1] - target[1])**2 +
                       alpha_teta * alpha_teta * e_theta**2)

            # Appliquer les commandes au robot
            self.robot.set_cmd_vel(command[0], command[1])

            # Attendre un court instant
            time.sleep(0.050)

            # Obtenir la nouvelle position du robot
            position = self.robot.get_position()

            # Mettre à jour l'entrée du réseau
            e_theta = math.atan2(math.sin(position[2] - target[2] - theta_s(position[0], position[1])),
                                 math.cos(position[2] - target[2] - theta_s(position[0], position[1])))
            network_input = [
                (position[0] - target[0]) * self.alpha[0],
                (position[1] - target[1]) * self.alpha[1],
                math.sin(e_theta),
                math.cos(e_theta) - 1,
            ]

            # Calculer le critère après déplacement
            crit_ap = (alpha_x * alpha_x * (position[0] - target[0])**2 +
                       alpha_y * alpha_y * (position[1] - target[1])**2 +
                       alpha_teta * alpha_teta * e_theta**2)

            if self.monitor:
                self.monitor.update(
                    position=position,
                    wheel_speeds=command,
                    gradient=[0, 0],
                    cost=crit_av,
                    sincos=[math.sin(e_theta), math.cos(e_theta) - 1]
                )

            # Apprentissage (si activé)
            if self.training:
                delta_t = (time.time() - debut)

                # Gradient du critère par rapport aux sorties [v_lin, v_ang]
                grad = [
                    -2*(alpha_x*alpha_x*(position[0]-target[0])*math.cos(position[2])
                       +alpha_y*alpha_y*(position[1]-target[1])*math.sin(position[2])),

                    -2*alpha_teta*alpha_teta*e_theta
                ]

                if self.monitor:
                    self.monitor.update(
                        position=position,
                        wheel_speeds=command,
                        gradient=grad,
                        cost=crit_av,
                        sincos=[math.sin(e_theta), math.cos(e_theta) - 1]
                    )

                grad_tensor = torch.tensor(grad, dtype=torch.float32)
                self.manual_backward(input_tensor, grad_tensor, 0.02, 0)

        # Arrêter le robot à la fin de l'apprentissage
        self.robot.set_cmd_vel(0, 0)
        self.running = False

    def manual_backward(self, inputs, grad_tensor, learning_rate, momentum):
        """
        Effectue manuellement une étape de rétropropagation avec le gradient fourni

        Args:
            inputs: les entrées du réseau
            grad_tensor: le gradient du critère par rapport aux sorties
            learning_rate: le taux d'apprentissage
            momentum: le facteur de momentum (non utilisé)
        """
        self.network.zero_grad()

        # Forward pass pour établir le graphe de calcul
        outputs = self.network(inputs)

        # Rétropropager le gradient
        outputs.backward(gradient=-grad_tensor)
        torch.nn.utils.clip_grad_norm_(self.network.parameters(), max_norm=1.0)
        # Mise à jour des poids
        for param in self.network.parameters():
            if param.grad is not None:
                param.data.add_(param.grad, alpha=-learning_rate)
