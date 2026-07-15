import time
import math
import torch


def theta_s(x, y, x_t=0.0, y_t=0.0):
    return math.atan2(y_t - y, x_t - x)


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
        network_input[2] = (position[2] - target[2] - theta_s(position[0], position[1], target[0], target[1])) * self.alpha[2]

        while self.running:
            input_tensor = torch.tensor(network_input, dtype=torch.float32)
            if self.training:
                output_tensor = self.network(input_tensor)
                command = output_tensor.tolist()
            else:
                with torch.no_grad():
                    command = self.network(input_tensor).tolist()

            e_theta = position[2] - target[2] - theta_s_prime(position[0], position[1], target[0], target[1])
            crit_av = (self.alpha[0]**2 * (position[0] - target[0])**2 +
                       self.alpha[1]**2 * (position[1] - target[1])**2 +
                       self.alpha[2]**2 * e_theta**2)

            self.robot.set_cmd_vel(command[0], command[1])
            time.sleep(0.050)
            position = self.robot.get_position()

            network_input[0] = (position[0] - target[0]) * self.alpha[0]
            network_input[1] = (position[1] - target[1]) * self.alpha[1]
            e_theta = position[2] - target[2] - theta_s_prime(position[0], position[1], target[0], target[1])
            network_input[2] = position[2] - target[2] * self.alpha[2]

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
