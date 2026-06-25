import json
import os
import subprocess
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
PC_USER = 'cerv'
PC_IP   = '192.168.1.241'
PC_DEST = '/home/cerv/Downloads/APP-Limo_ros2_curr/res/'

rclpy.init()
real_robot = ''
while real_robot.lower() not in ('y', 'n'):
    real_robot = input('Real robot? (y/n) --> ')
is_real_robot = (real_robot.lower() == 'y')
subdir = 'robot' if is_real_robot else 'gazebo'
run_stamp = time.strftime('%Y%m%d_%H%M%S')
run_dir = os.path.join('res', subdir, f'run_{run_stamp}')
os.makedirs(run_dir, exist_ok=True)

robot = LimoROS2Interface(safety_limits=is_real_robot)
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

HL_size = 1000
LEARNING_RATE = 0.2
input_size = 3
output_size = 3

network = PioneerNN(input_size, HL_size, output_size)

display_choice = ''
if MONITORING_AVAILABLE:
    while display_choice.lower() not in ('y', 'n'):
        display_choice = input('Enable real-time display? (y/n) --> ')
    if display_choice.lower() == 'y':
        monitor.start_monitoring()
else:
    display_choice = 'n'
    print('Monitoring not available (matplotlib not installed).')

choice = ''
while choice.lower() not in ('y', 'n'):
    choice = input('Do you want to load previous network? (y/n) --> ')
if choice.lower() == 'y':
    try:
        with open('last_w_torch_mcnamu.json') as fp:
            json_obj = json.load(fp)
        network.load_weights_from_json(json_obj)
        print("Weights loaded from last_w_torch_mcnamu.json")
    except FileNotFoundError:
        print("No weight file found (last_w_torch_mcnamu.json), starting with random weights.")

monitor_instance = monitor if display_choice.lower() == 'y' else None
logger = DataLogger()
trainer = PyTorchOnlineTrainer(robot, network, monitor_instance, logger, learning_rate=LEARNING_RATE)

choice = ''
while choice not in ('y', 'n'):
    choice = input('Do you want to learn? (y/n) --> ')
trainer.training = (choice == 'y')

target = []
while len(target) != 3:
    target = [float(v) for v in input("Enter the first target : x y radian --> ").split()]

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

    mode = "training" if trainer.training else "eval"
    stamp = f"session_{session_count}_{time.strftime('%Y%m%d_%H%M%S')}"
    csv_path = os.path.join(run_dir, f"{mode}_{stamp}.csv")
    logger.save(csv_path)
    logger.save_plot(csv_path.replace('.csv', '.png'))
    logger.reset()

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
    with open('last_w_torch_mcnamu.json', 'w') as fp:
        json.dump(json_obj, fp)
    print("Weights saved to last_w_torch_mcnamu.json")
else:
    print("Weights not saved.")

if display_choice.lower() == 'y':
    mode = "training" if trainer.training else "eval"
    monitor.save_results(f"{mode}_final_{time.strftime('%Y%m%d_%H%M%S')}",
                         results_dir=run_dir,
                         final=True)

if is_real_robot:
    send_choice = ''
    while send_choice.lower() not in ('y', 'n'):
        send_choice = input(f"Send {run_dir} to PC? (y/n) --> ")
    if send_choice.lower() == 'y':
        dest = f"{PC_USER}@{PC_IP}:{PC_DEST}{subdir}/"
        print(f"Sending {run_dir} → {dest}")
        result = subprocess.run(['scp', '-r', run_dir, dest])
        if result.returncode == 0:
            print("Transfer complete.")
        else:
            print("Transfer failed — check SSH/SCP access to the PC.")
