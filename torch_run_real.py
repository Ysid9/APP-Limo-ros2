"""
torch_run_real.py  —  version rapide pour tests sur robot réel
- Robot réel toujours (safety_limits=True)
- Monitoring activé automatiquement
- Cible toujours (0, 0, 0)
- Prompts : chargement poids / apprendre / continuer / sauvegarder
"""
import json
import os
import threading
import atexit
import time
import rclpy
from ros2_limo_interface import LimoROS2Interface
from torch_back import PioneerNN
from torch_online import PyTorchOnlineTrainer
from data_logger import DataLogger
try:
    from monitoring import RobotMonitorAdapter
    MONITORING_AVAILABLE = True
except Exception:
    MONITORING_AVAILABLE = False

WORLD_BOUNDS = (-10, 10, -10, 10)
TARGET  = [0.0, 0.0, 0.0]

rclpy.init()
run_stamp = time.strftime('%Y%m%d_%H%M%S')
run_dir = os.path.join('res', 'robot', f'run_{run_stamp}')
os.makedirs(run_dir, exist_ok=True)

robot = LimoROS2Interface(safety_limits=True)
monitor = RobotMonitorAdapter(world_bounds=WORLD_BOUNDS) if MONITORING_AVAILABLE else None
trainer = None

def cleanup():
    print("Cleaning up...")
    if trainer is not None:
        trainer.running = False
    if hasattr(robot, 'cleanup'):
        robot.cleanup()
    if monitor is not None:
        monitor.stop_monitoring()
    rclpy.shutdown()
atexit.register(cleanup)

HL_size       = 1000
LEARNING_RATE = 0.2
input_size    = 3
output_size   = 2

network = PioneerNN(input_size, HL_size, output_size)

if MONITORING_AVAILABLE:
    monitor.start_monitoring()
else:
    print('Monitoring not available (matplotlib not installed).')

choice = ''
while choice.lower() not in ('y', 'n'):
    choice = input('Do you want to load previous network? (y/n) --> ')
if choice.lower() == 'y':
    try:
        with open('last_w_torch_diff.json') as fp:
            json_obj = json.load(fp)
        network.load_weights_from_json(json_obj)
        print("Weights loaded from last_w_torch_diff.json")
    except FileNotFoundError:
        print("No weight file found, starting with random weights.")

logger = DataLogger()
trainer = PyTorchOnlineTrainer(robot, network, monitor, logger, learning_rate=LEARNING_RATE)

choice = ''
while choice not in ('y', 'n'):
    choice = input('Do you want to learn? (y/n) --> ')
trainer.training = (choice == 'y')

continue_running = True
session_count = 0

while continue_running:
    session_count += 1
    print(f"\nStarting session #{session_count}  — target: {TARGET}")

    thread = threading.Thread(target=trainer.train, args=(TARGET,))
    trainer.running = True
    thread.start()

    try:
        input("Press Enter to stop the current session")
        trainer.running = False
        thread.join(timeout=5)
        if thread.is_alive():
            print("Warning: training thread did not finish in time")
    except KeyboardInterrupt:
        print("\nStopping...")
        trainer.running = False
        thread.join(timeout=5)

    mode = "training" if trainer.training else "eval"
    stamp = f"session_{session_count}_{time.strftime('%Y%m%d_%H%M%S')}"
    csv_path = os.path.join(run_dir, f"{mode}_{stamp}.csv")
    logger.save(csv_path)
    logger.save_plot(csv_path.replace('.csv', '.png'))
    logger.reset()

    choice = ''
    while choice.lower() not in ('y', 'n'):
        choice = input("Do you want to continue? (y/n) --> ")
    continue_running = (choice.lower() == 'y')

save_choice = ''
while save_choice.lower() not in ('y', 'n'):
    save_choice = input("Do you want to save the weights? (y/n) --> ")
if save_choice.lower() == 'y':
    json_obj = network.save_weights_to_json()
    with open('last_w_torch_diff.json', 'w') as fp:
        json.dump(json_obj, fp)
    print("Weights saved to last_w_torch_diff.json")

if MONITORING_AVAILABLE:
    mode = "training" if trainer.training else "eval"
    monitor.save_results(f"{mode}_final_{time.strftime('%Y%m%d_%H%M%S')}",
                         results_dir=run_dir, final=True)
