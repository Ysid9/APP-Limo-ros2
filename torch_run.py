import torch
import json
import threading
import atexit
import time
from zmq_pioneer_simulation import ZMQPioneerSimulation
from torch_back import PioneerNN
from torch_online import PyTorchOnlineTrainer
from monitoring import RobotMonitorAdapter

# Définir les dimensions du monde pour le monitoring
WORLD_BOUNDS = (-10, 10, -10, 10)  # x_min, x_max, y_min, y_max

# Initialize the robot with ZMQ Remote API
robot = ZMQPioneerSimulation()
monitor = RobotMonitorAdapter(world_bounds=WORLD_BOUNDS)
trainer = None

# Register cleanup function to ensure simulation is stopped
def cleanup():
    print("Cleaning up and stopping simulation...")
    if trainer is not None:
        trainer.running = False
    if hasattr(robot, 'cleanup'):
        robot.cleanup()
    if hasattr(monitor, 'stop_monitoring'):
        monitor.stop_monitoring()
atexit.register(cleanup)

# Wait a moment to ensure the simulation is fully started
time.sleep(1)

# Configuration de base  (par défaut pour la couche cachée.Peut etre modifié dans le module torch_back.py)
HL_size = 1000
input_size = 3
output_size = 2

# Création du réseau PyTorch
network = PioneerNN(input_size, HL_size, output_size)

# Ask for display preference
display_choice = ''
while display_choice.lower() not in ('y', 'n'):
    display_choice = input('Enable real-time display? (y/n) --> ')

if display_choice.lower() == 'y':
    monitor.start_monitoring()

# Chargement de poids existants
choice = input('Do you want to load previous network? (y/n) --> ')
if choice == 'y':
    try:
        with open('last_w_torch_pioneer.json') as fp:
            json_obj = json.load(fp)
        network.load_weights_from_json(json_obj)
        print("Weights loaded from last_w_torch_pioneer.json")
    except FileNotFoundError:
        print("No weight file found (last_w_torch_pioneer.json), starting with random weights.")
    

# Initialiser le trainer PyTorch avec monitoring
monitor_instance = monitor if display_choice.lower() == 'y' else None
trainer = PyTorchOnlineTrainer(robot, network, monitor_instance)

choice = ''
while choice!='y' and choice !='n':
    choice = input('Do you want to learn? (y/n) --> ')

trainer.training = (choice.lower() == 'y')

# Demander la cible
target_input = input("Enter the first target : x y radian --> ")
target = target_input.split()
if len(target) != 3:
    raise ValueError("Need exactly 3 values")
for i in range(len(target)):
    target[i] = float(target[i])


# Boucle principale d'entraînement
continue_running = True
session_count = 0

while continue_running:
    session_count += 1
    print(f"\n⚙️ Starting training session #{session_count}")

    thread = threading.Thread(target=trainer.train, args=(target,))
    trainer.running = True
    thread.start()

    try:
        input("Press Enter to stop the current training")
        trainer.running = False
        
        thread.join(timeout=5)
        if thread.is_alive():
            print("⚠️ Training thread did not finish in time, continuing anyway")
            
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Stopping training...")
        trainer.running = False
        thread.join(timeout=5)

    if display_choice.lower() == 'y':
        monitor.save_results(f"session_{session_count}_{time.strftime('%Y%m%d_%H%M%S')}")

    choice = ''
    while choice.lower() not in ['y', 'n']:
        choice = input("Do you want to continue? (y/n) --> ")

    if choice.lower() == 'y':
        choice_learning = ''
        while choice_learning.lower() not in ['y', 'n']:
            choice_learning = input('Do you want to learn? (y/n) --> ')
        
        trainer.training = (choice_learning.lower() == 'y')
        
        target_input = input("Move the robot to the initial point and enter the new target : x y radian --> ")
        target = [float(x) for x in target_input.split()]
        if len(target) != 3:
            raise ValueError("Need exactly 3 values")
        
    else:
        continue_running = False

save_choice = ''
while save_choice.lower() not in ['y', 'n']:
    save_choice = input("Do you want to save the weights? (y/n) --> ")

if save_choice.lower() == 'y':
    json_obj = network.save_weights_to_json()
    with open('last_w_torch_pioneer.json', 'w') as fp:
        json.dump(json_obj, fp)
    print("Weights saved to last_w_torch_pioneer.json")
else:
    print("Weights not saved.")

if display_choice.lower() == 'y':
    monitor.save_results(f"final_results_{time.strftime('%Y%m%d_%H%M%S')}")
else:
    print("⚠️ No results saved, monitoring was disabled")


