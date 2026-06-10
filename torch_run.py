import json
import os
import threading
import atexit
import time
import torch
from zmq_pioneer_simulation import ZMQPioneerSimulation
from torch_back import PioneerNN
from torch_online import PyTorchOnlineTrainer
from monitoring import RobotMonitorAdapter
from data_logger import DataLogger

WORLD_BOUNDS = (-10, 10, -10, 10)

robot   = ZMQPioneerSimulation()
monitor = RobotMonitorAdapter(world_bounds=WORLD_BOUNDS)
trainer = None

def cleanup():
    print("Cleaning up...")
    if trainer is not None:
        trainer.running = False
    if hasattr(robot, 'cleanup'):
        robot.cleanup()
    if monitor is not None:
        monitor.stop_monitoring()
atexit.register(cleanup)

time.sleep(1)

HL_size       = 1000
LEARNING_RATE = 0.2
input_size    = 3
output_size   = 2

run_stamp = time.strftime('%Y%m%d_%H%M%S')
run_dir   = os.path.join('res', f'run_{run_stamp}')
os.makedirs(run_dir, exist_ok=True)

network = PioneerNN(input_size, HL_size, output_size)

display_choice = ''
while display_choice.lower() not in ('y', 'n'):
    display_choice = input('Enable real-time display? (y/n) --> ')
if display_choice.lower() == 'y':
    monitor.start_monitoring()

choice = ''
while choice.lower() not in ('y', 'n'):
    choice = input('Do you want to load previous network? (y/n) --> ')
if choice.lower() == 'y':
    try:
        with open('last_w_torch_3in.json') as fp:
            json_obj = json.load(fp)
        network.load_weights_from_json(json_obj)
        print("Weights loaded from last_w_torch_3in.json")
    except FileNotFoundError:
        print("No weight file found (last_w_torch_3in.json), starting with random weights.")

monitor_instance = monitor if display_choice.lower() == 'y' else None
logger  = DataLogger()
trainer = PyTorchOnlineTrainer(robot, network, monitor_instance, logger,
                               learning_rate=LEARNING_RATE)

choice = ''
while choice not in ('y', 'n'):
    choice = input('Do you want to learn? (y/n) --> ')
trainer.training = (choice == 'y')

target = []
while len(target) != 3:
    target = [float(v) for v in input("Enter the first target : x y radian --> ").split()]

continue_running = True
session_count    = 0

while continue_running:
    session_count += 1
    print(f"\nStarting session #{session_count}")

    thread = threading.Thread(target=trainer.train, args=(target,))
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

    mode  = "training" if trainer.training else "eval"
    stamp = f"session_{session_count}_{time.strftime('%Y%m%d_%H%M%S')}"
    csv_path = os.path.join(run_dir, f"{mode}_{stamp}.csv")
    logger.save(csv_path)
    logger.save_plot(csv_path.replace('.csv', '.png'))
    logger.reset()

    if display_choice.lower() == 'y':
        monitor.save_results(f"{mode}_{stamp}", results_dir=run_dir)

    choice = ''
    while choice.lower() not in ('y', 'n'):
        choice = input("Do you want to continue? (y/n) --> ")

    if choice.lower() == 'y':
        choice_learning = ''
        while choice_learning.lower() not in ('y', 'n'):
            choice_learning = input('Do you want to learn? (y/n) --> ')
        trainer.training = (choice_learning == 'y')

        target = []
        while len(target) != 3:
            target = [float(v) for v in input("Move robot to start position, then enter target : x y radian --> ").split()]
    else:
        continue_running = False

save_choice = ''
while save_choice.lower() not in ('y', 'n'):
    save_choice = input("Do you want to save the weights? (y/n) --> ")

if save_choice.lower() == 'y':
    json_obj = network.save_weights_to_json()
    with open('last_w_torch_3in.json', 'w') as fp:
        json.dump(json_obj, fp)
    print("Weights saved to last_w_torch_3in.json")
else:
    print("Weights not saved.")

if display_choice.lower() == 'y':
    mode = "training" if trainer.training else "eval"
    monitor.save_results(f"{mode}_final_{time.strftime('%Y%m%d_%H%M%S')}",
                         results_dir=run_dir, final=True)
