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
        self._session_count = 0
        self._debug_file = None
        
    def _dlog(self, msg):
        print(msg)
        if self._debug_file:
            self._debug_file.write(msg + '\n')
            self._debug_file.flush()

    def train(self, target):
        self._session_count += 1
        debug_path = f'debug_session_{self._session_count}.txt'
        self._debug_file = open(debug_path, 'w')

        if self.monitor:
            self.monitor.start_session()
            self.monitor.set_target(target)
            
        position = self.robot.get_position()

        network_input = [0, 0, 0]
        network_input[0] = (position[0] - target[0]) * self.alpha[0]
        network_input[1] = (position[1] - target[1]) * self.alpha[1]
        network_input[2] = (position[2] - target[2] - theta_s(position[0], position[1])) * self.alpha[2]

        ts = theta_s(position[0], position[1])
        self._dlog(f'=== SESSION {self._session_count} START ===')
        self._dlog(f'target  = {[round(v,4) for v in target]}')
        self._dlog(f'pos     = {[round(v,4) for v in position]}')
        self._dlog(f'theta_s = {ts:.4f} rad ({math.degrees(ts):.1f} deg)')
        self._dlog(f'inputs  = {[round(v,4) for v in network_input]}')

        step = 0
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

            if step < 20 or step % 50 == 0:
                self._dlog(f'[{step:04d}] pos={[round(v,3) for v in position]} '
                           f'inp={[round(v,3) for v in network_input]} '
                           f'cmd=[{command[0]:.3f},{command[1]:.3f},{command[2]:.3f}] '
                           f'cost={crit_av:.5f}')
            step += 1

            self.robot.set_cmd_vel(command[0], command[1], command[2])
            time.sleep(0.050)
            position = self.robot.get_position()

            network_input[0] = (position[0] - target[0]) * self.alpha[0]
            network_input[1] = (position[1] - target[1]) * self.alpha[1]
            network_input[2] = (position[2] - target[2] - theta_s(position[0], position[1])) * self.alpha[2]

            grad = [0.0, 0.0, 0.0]

            if self.training:
                ex = position[0] - target[0]
                ey = position[1] - target[1]
                e_theta = position[2] - target[2] - theta_s(position[0], position[1])
                c, s = math.cos(position[2]), math.sin(position[2])
                grad = [
                    -2*(self.alpha[0]**2*ex*c + self.alpha[1]**2*ey*s),   # ∂J/∂v_x
                    -2*(-self.alpha[0]**2*ex*s + self.alpha[1]**2*ey*c),  # ∂J/∂v_y
                    -2*self.alpha[2]**2*e_theta                            # ∂J/∂v_ang
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
        
        self._dlog(f'=== SESSION {self._session_count} END (steps={step}) ===')
        self._debug_file.close()
        self._debug_file = None
        self.robot.set_cmd_vel(0, 0, 0)
    
    def manual_backward(self, outputs, grad_tensor, learning_rate, momentum):
        self.network.zero_grad()
        outputs.backward(gradient=-grad_tensor)
        for param in self.network.parameters():
            if param.grad is not None:
                param.data.add_(param.grad, alpha=-learning_rate)
