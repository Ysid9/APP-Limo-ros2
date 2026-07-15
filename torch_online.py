import time
import math
import torch

# Ancienne theta_s — conçue pour le quadrant (+x,+y).
# Problème : tanh(10x) sature dès |x|>0.5, donc theta_s ≈ sign(x)·atan(y)
# identique dans les quadrants opposés (ex: (2.5,2.5) et (-2.5,-2.5) donnent
# tous deux theta_s ≈ +68°). Le réseau reçoit le même input[2] dans des
# situations qui nécessitent des comportements opposés → ambiguïté,
# impossibilité de généraliser à tous les quadrants.
#
# def theta_s(x, y):
#     return math.tanh(10.*x) * math.atan(1.*y)

# Nouvelle theta_s — direction réelle vers la cible depuis (x,y).
# Donne un signal angulaire unique et cohérent dans tous les quadrants :
# le robot apprend à s'orienter vers la cible, quelle que soit sa position.
def theta_s(x, y, x_t=0.0, y_t=0.0):
    return math.atan2(y_t - y, x_t - x)

# theta_s_prime — modulation de theta_s par la distance à la cible.
# Loin (d grand) : theta_s_prime ≈ -theta_s  →  e_theta ≈ θ - θ_target - theta_s
#                  le robot s'oriente vers la cible (comportement theta_s standard)
# Près (d → 0)  : theta_s_prime → 0          →  e_theta ≈ θ - θ_target
#                  le robot s'aligne sur l'orientation finale souhaitée
# beta < 0 contrôle la vitesse de transition (défaut -1.0 : transition à ~1 m)
def theta_s_prime(x, y, x_t=0.0, y_t=0.0, beta=-5.0):
    d = math.sqrt((x - x_t)**2 + (y - y_t)**2)
    return -(1.0 - math.exp(beta * d)) * theta_s(x, y, x_t, y_t)

class PyTorchOnlineTrainer:
    def __init__(self, robot, nn_model, monitor=None, logger=None, learning_rate=0.2):
        self.robot = robot
        self.network = nn_model
        self.learning_rate = learning_rate
        self.alpha = [1/6, 1/6, 1/(math.pi)]

        self.running = False
        self.training = False

        self.monitor = monitor
        self.logger = logger

    def train(self, target):
        if self.monitor:
            self.monitor.start_session()
            self.monitor.set_target(target)

        position = self.robot.get_position()

        network_input = [0, 0, 0]
        network_input[0] = (position[0] - target[0]) * self.alpha[0]
        network_input[1] = (position[1] - target[1]) * self.alpha[1]
        network_input[2] = (position[2] - target[2] + theta_s_prime(position[0], position[1], target[0], target[1])) * self.alpha[2]

        while self.running:
            input_tensor = torch.tensor(network_input, dtype=torch.float32)
            if self.training:
                output_tensor = self.network(input_tensor)
                command = output_tensor.tolist()
            else:
                with torch.no_grad():
                    command = self.network(input_tensor).tolist()

            e_theta = position[2] - target[2] + theta_s_prime(position[0], position[1], target[0], target[1])
            crit_av = (self.alpha[0]**2 * (position[0] - target[0])**2 +
                       self.alpha[1]**2 * (position[1] - target[1])**2 +
                       self.alpha[2]**2 * e_theta**2)

            self.robot.set_cmd_vel(command[0], command[1])
            time.sleep(0.050)
            position = self.robot.get_position()

            network_input[0] = (position[0] - target[0]) * self.alpha[0]
            network_input[1] = (position[1] - target[1]) * self.alpha[1]
            e_theta = position[2] - target[2] + theta_s_prime(position[0], position[1], target[0], target[1])
            network_input[2] = e_theta * self.alpha[2]

            grad = [0.0, 0.0]

            if self.training:
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

        self.robot.set_cmd_vel(0, 0)

    def manual_backward(self, outputs, grad_tensor, learning_rate, momentum):
        self.network.zero_grad()
        outputs.backward(gradient=-grad_tensor)
        for param in self.network.parameters():
            if param.grad is not None:
                param.data.add_(param.grad, alpha=-learning_rate)
