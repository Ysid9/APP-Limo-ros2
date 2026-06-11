import time
import math
import torch


def theta_s(x, y):
    return math.tanh(10.*x)*math.atan(1.*y)


class PyTorchOnlineTrainer:
    def __init__(self, robot, nn_model, monitor=None, logger=None):
        self.robot = robot
        self.network = nn_model
        self.alpha = [1/6, 1/6, 1/(math.pi)]

        self.running = False
        self.training = False

        self.optimizer = torch.optim.SGD(self.network.parameters(), lr=0.2, momentum=0)

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
        network_input[2] = (position[2] - target[2] - theta_s(position[0], position[1])) * self.alpha[2]

        while self.running:
            debut = time.time()

            input_tensor = torch.tensor(network_input, dtype=torch.float32)
            if self.training:
                input_tensor.requires_grad_(True)
                command = self.network(input_tensor).tolist()
            else:
                with torch.no_grad():
                    command = self.network(input_tensor).tolist()

            alpha_x    = 1/6
            alpha_y    = 1/6
            alpha_teta = 1.0/(math.pi)

            crit_av = (alpha_x * alpha_x * (position[0] - target[0])**2 +
                       alpha_y * alpha_y * (position[1] - target[1])**2 +
                       alpha_teta * alpha_teta * (position[2] - target[2] -
                                                  theta_s(position[0], position[1]))**2)

            self.robot.set_motor_velocity(command)
            time.sleep(0.050)
            position = self.robot.get_position()

            network_input[0] = (position[0] - target[0]) * self.alpha[0]
            network_input[1] = (position[1] - target[1]) * self.alpha[1]
            network_input[2] = (position[2] - target[2] - theta_s(position[0], position[1])) * self.alpha[2]

            crit_ap = (alpha_x * alpha_x * (position[0] - target[0])**2 +
                       alpha_y * alpha_y * (position[1] - target[1])**2 +
                       alpha_teta * alpha_teta * (position[2] - target[2] -
                                                  theta_s(position[0], position[1]))**2)

            grad = [0.0, 0.0]

            if self.training:
                delta_t = (time.time() - debut)

                grad = [
                    (-2/delta_t)*(alpha_x*alpha_x*(position[0]-target[0])*delta_t*self.robot.r*math.cos(position[2])
                     +alpha_y*alpha_y*(position[1]-target[1])*delta_t*self.robot.r*math.sin(position[2])
                     -alpha_teta*alpha_teta*(position[2]-target[2]-theta_s(position[0], position[1]))*delta_t*self.robot.r/(2*self.robot.R)),

                    (-2/delta_t)*(alpha_x*alpha_x*(position[0]-target[0])*delta_t*self.robot.r*math.cos(position[2])
                     +alpha_y*alpha_y*(position[1]-target[1])*delta_t*self.robot.r*math.sin(position[2])
                     +alpha_teta*alpha_teta*(position[2]-target[2]-theta_s(position[0], position[1]))*delta_t*self.robot.r/(2*self.robot.R))
                ]

                if self.monitor:
                    self.monitor.update(
                        position=position,
                        wheel_speeds=command,
                        gradient=grad,
                        cost=crit_av
                    )

                if crit_ap <= crit_av:
                    self.optimizer.zero_grad()
                    grad_tensor = torch.tensor(grad, dtype=torch.float32)
                    self.manual_backward(input_tensor, grad_tensor, 0.2, 0)
                else:
                    self.optimizer.zero_grad()
                    grad_tensor = torch.tensor(grad, dtype=torch.float32)
                    self.manual_backward(input_tensor, grad_tensor, 0.2, 0)

            if self.logger:
                self.logger.log(position, target, command, grad, crit_av)

        self.robot.set_motor_velocity([0, 0])
        self.running = False

    def manual_backward(self, inputs, grad_tensor, learning_rate, momentum):
        self.optimizer.zero_grad()
        outputs = self.network(inputs)
        outputs.backward(gradient=-grad_tensor)
        for param in self.network.parameters():
            if param.grad is not None:
                param.data.add_(param.grad, alpha=-learning_rate)
