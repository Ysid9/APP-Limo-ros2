import time
import math
import torch

def theta_s(x, y):
    return math.tanh(10.*x)*math.atan(1.*y)

class PyTorchOnlineTrainer:
    def __init__(self, robot, nn_model, monitor=None, logger=None, learning_rate=0.2,
                 tolerance_cost=None, tolerance_time=2.0):
        self.robot = robot
        self.network = nn_model
        self.learning_rate = learning_rate
        self.alpha = [1/6, 1/6, 1/(math.pi)]

        self.running = False
        self.training = False

        self.monitor = monitor
        self.logger = logger

        # Tolérance de convergence : arrêt si coût < tolerance_cost pendant tolerance_time s
        self.tolerance_cost = tolerance_cost
        self.tolerance_time = tolerance_time
        
    def train(self, target):
        """
        Procédure d'apprentissage en ligne
        
        Args:
            target (list): position cible [x,y,theta]
        """
        
        if self.monitor:
            self.monitor.start_session()
            self.monitor.set_target(target)
            
        # Obtenir la position actuelle du robot
        position = self.robot.get_position()
        
        # Calculer l'entrée du réseau (erreur normalisée)
        network_input = [0, 0, 0]
        network_input[0] = (position[0] - target[0]) * self.alpha[0]
        network_input[1] = (position[1] - target[1]) * self.alpha[1]
        network_input[2] = (position[2] - target[2] - theta_s(position[0], position[1])) * self.alpha[2]
        
        _below_since = None

        while self.running:
            input_tensor = torch.tensor(network_input, dtype=torch.float32)
            if self.training:
                output_tensor = self.network(input_tensor)
                command = output_tensor.tolist()
            else:
                with torch.no_grad():
                    command = self.network(input_tensor).tolist()

            crit_av = (self.alpha[0]**2 * (position[0] - target[0])**2 +
                       self.alpha[1]**2 * (position[1] - target[1])**2 +
                       self.alpha[2]**2 * (position[2] - target[2] -
                                           theta_s(position[0], position[1]))**2)

            self.robot.set_cmd_vel(command[0], command[1])
            time.sleep(0.050)
            position = self.robot.get_position()

            network_input[0] = (position[0] - target[0]) * self.alpha[0]
            network_input[1] = (position[1] - target[1]) * self.alpha[1]
            network_input[2] = (position[2] - target[2] - theta_s(position[0], position[1])) * self.alpha[2]

            grad = [0.0, 0.0]

            if self.training:
                e_theta = position[2] - target[2] - theta_s(position[0], position[1])
                grad = [
                    -2*(self.alpha[0]**2*(position[0]-target[0])*math.cos(position[2])
                       +self.alpha[1]**2*(position[1]-target[1])*math.sin(position[2])),
                    -2*self.alpha[2]**2*e_theta
                ]

            if self.monitor:
                self.monitor.update(
                    position=position,
                    wheel_speeds=command,
                    gradient=grad,
                    cost=crit_av
                )

            if self.logger:
                self.logger.log(position, target, command, grad, crit_av)

            if self.training:
                grad_tensor = torch.tensor(grad, dtype=torch.float32)
                self.manual_backward(output_tensor, grad_tensor, self.learning_rate, 0)

            if self.tolerance_cost is not None:
                if crit_av < self.tolerance_cost:
                    if _below_since is None:
                        _below_since = time.time()
                    elif time.time() - _below_since >= self.tolerance_time:
                        print(f"Convergence — coût {crit_av:.6f} < {self.tolerance_cost} "
                              f"pendant {self.tolerance_time}s, arrêt de la session.")
                        self.running = False
                else:
                    _below_since = None

        self.robot.set_cmd_vel(0, 0)
    
    def manual_backward(self, outputs, grad_tensor, learning_rate, momentum):
        self.network.zero_grad()
        outputs.backward(gradient=-grad_tensor)
        for param in self.network.parameters():
            if param.grad is not None:
                param.data.add_(param.grad, alpha=-learning_rate)
