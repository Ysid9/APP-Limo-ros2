import torch
import json
import threading
import atexit
import time
import rclpy
from ros2_limo_interface import LimoROS2Interface
from torch_back import PioneerNN
from torch_online import PyTorchOnlineTrainer
try:
    from monitoring import RobotMonitorAdapter
    MONITORING_AVAILABLE = True
except ImportError:
    MONITORING_AVAILABLE = False

WORLD_BOUNDS = (-10, 10, -10, 10)

rclpy.init()
robot = LimoROS2Interface()
monitor = RobotMonitorAdapter(world_bounds=WORLD_BOUNDS) if MONITORING_AVAILABLE else None
trainer = None

def cleanup():
    print("Cleaning up...")
    if trainer is not None:
        trainer.running = False
    if hasattr(robot, 'cleanup'):
        robot.cleanup()
    if hasattr(monitor, 'stop_monitoring'):
        monitor.stop_monitoring()
    rclpy.shutdown()
atexit.register(cleanup)

HL_size = 1000
input_size = 3
output_size = 2

network = PioneerNN(input_size, HL_size, output_size)

display_choice = 'n'
if MONITORING_AVAILABLE:
    while display_choice.lower() not in ('y', 'n'):
        display_choice = input('Enable real-time display? (y/n) --> ')
    if display_choice.lower() == 'y':
        monitor.start_monitoring()
else:
    print('Monitoring not available (matplotlib not installed).')

choice = input('Do you want to load previous network? (y/n) --> ')
if choice == 'y':
    try:
        with open('last_w_torch_3in.json') as fp:
            json_obj = json.load(fp)
        network.load_weights_from_json(json_obj)
        print("Weights loaded from last_w_torch_3in.json")
    except FileNotFoundError:
        print("No weight file found (last_w_torch_3in.json), starting with random weights.")

monitor_instance = monitor if display_choice.lower() == 'y' else None
trainer = PyTorchOnlineTrainer(robot, network, monitor_instance)

choice = ''
while choice not in ('y', 'n'):
    choice = input('Do you want to learn? (y/n) --> ')
trainer.training = (choice == 'y')

target_input = input("Enter the first target : x y radian --> ")
target = [float(v) for v in target_input.split()]
if len(target) != 3:
    raise ValueError("Need exactly 3 values")

continue_running = True
session_count = 0

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

    if display_choice.lower() == 'y':
        monitor.save_results(f"session_{session_count}_{time.strftime('%Y%m%d_%H%M%S')}")

    choice = ''
    while choice.lower() not in ('y', 'n'):
        choice = input("Do you want to continue? (y/n) --> ")

    if choice.lower() == 'y':
        choice_learning = ''
        while choice_learning.lower() not in ('y', 'n'):
            choice_learning = input('Do you want to learn? (y/n) --> ')
        trainer.training = (choice_learning == 'y')

        target_input = input("Move robot to start position with teleop, then enter target : x y radian --> ")
        target = [float(v) for v in target_input.split()]
        if len(target) != 3:
            raise ValueError("Need exactly 3 values")
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
    monitor.save_results(f"final_results_{time.strftime('%Y%m%d_%H%M%S')}")
