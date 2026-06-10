import time
import math
import torch


def theta_s(x, y):
    return math.tanh(10.*x)*math.atan(1.*y)


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
        network_input[2] = (position[2] - target[2] - theta_s(position[0], position[1])) * self.alpha[2]

        while self.running:
            debut = time.time()

            input_tensor = torch.tensor(network_input, dtype=torch.float32)
            if self.training:
                output_tensor = self.network(input_tensor)
                command = output_tensor.tolist()
            else:
                with torch.no_grad():
                    command = self.network(input_tensor).tolist()

            alpha_x    = self.alpha[0]
            alpha_y    = self.alpha[1]
            alpha_teta = self.alpha[2]

            crit_av = (alpha_x**2 * (position[0] - target[0])**2 +
                       alpha_y**2 * (position[1] - target[1])**2 +
                       alpha_teta**2 * (position[2] - target[2] -
                                        theta_s(position[0], position[1]))**2)

            self.robot.set_motor_velocity(command)
            time.sleep(0.050)
            position = self.robot.get_position()

            network_input[0] = (position[0] - target[0]) * self.alpha[0]
            network_input[1] = (position[1] - target[1]) * self.alpha[1]
            network_input[2] = (position[2] - target[2] - theta_s(position[0], position[1])) * self.alpha[2]

            grad = [0.0, 0.0]

            if self.training:
                delta_t = time.time() - debut
                e_theta = position[2] - target[2] - theta_s(position[0], position[1])
                grad = [
                    (-2/delta_t)*(alpha_x**2*(position[0]-target[0])*delta_t*self.robot.r*math.cos(position[2])
                     + alpha_y**2*(position[1]-target[1])*delta_t*self.robot.r*math.sin(position[2])
                     - alpha_teta**2*e_theta*delta_t*self.robot.r/(2*self.robot.R)),

                    (-2/delta_t)*(alpha_x**2*(position[0]-target[0])*delta_t*self.robot.r*math.cos(position[2])
                     + alpha_y**2*(position[1]-target[1])*delta_t*self.robot.r*math.sin(position[2])
                     + alpha_teta**2*e_theta*delta_t*self.robot.r/(2*self.robot.R))
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

        self.robot.set_motor_velocity([0, 0])

    def manual_backward(self, outputs, grad_tensor, learning_rate, momentum):
        self.network.zero_grad()
        outputs.backward(gradient=-grad_tensor)
        for param in self.network.parameters():
            if param.grad is not None:
                param.data.add_(param.grad, alpha=-learning_rate)
