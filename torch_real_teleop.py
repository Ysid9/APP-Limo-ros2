"""
ros2_torch_run_real.py
=======================
Neural network closed-loop learning — REAL ROBOT version.
User-friendly interface for demonstrations and experiments on AgileX LIMO.

Two modes:
    DEMO        — loads pre-trained weights, no learning, robot goes to target
    LEARNING    — progressive training, use teleop to position the robot

Between sessions, the user drives the robot to the desired starting position
using teleop_twist_keyboard in a separate terminal.

Usage:
    Terminal 1 — robot driver:
        ros2 launch limo_bringup limo_start.launch.py

    Terminal 2 — this script:
        python3 ros2_torch_run_real.py

    Terminal 3 — teleop (when prompted by the script):
        ros2 run teleop_twist_keyboard teleop_twist_keyboard

Author  : Yasser — AMARSMER project, CERV/LabSTICC, ENIB
Date    : 2026
"""

import json
import os
import sys
import threading
import atexit
import time
import math

from ros2_limo_simulation import ROS2LimoSimulation
from torch_back import PioneerNN
from torch_online import PyTorchOnlineTrainer
from monitoring import RobotMonitorAdapter


# ═══════════════════════════════════════════════════════════════════════
# CONFIGURATION — adjust these parameters for your environment
# ═══════════════════════════════════════════════════════════════════════

# ROS2 topics
CMD_VEL_TOPIC = '/cmd_vel'
ODOM_TOPIC    = '/wheel/odom'          # '/odometry' for Gazebo, '/odom' for real robot, '/wheel/odom'

# Monitoring world bounds (for matplotlib display)
WORLD_BOUNDS = (-10, 10, -10, 10)

# Weights file
WEIGHTS_FILE        = 'last_w_torch.json'
WEIGHTS_HISTORY_DIR = 'weights_history'

# Neural network dimensions
HL_SIZE     = 10     # overridden to 1000 in torch_back.py
INPUT_SIZE  = 3      # normalized error [dx, dy, dtheta]
OUTPUT_SIZE = 2      # wheel speeds [v_left, v_right]


# ═══════════════════════════════════════════════════════════════════════
# DISPLAY HELPERS
# ═══════════════════════════════════════════════════════════════════════

SEPARATOR = '═' * 60

def banner(text):
    print(f'\n{SEPARATOR}')
    print(f'  {text}')
    print(SEPARATOR)

def info(text):
    print(f'  ℹ️  {text}')

def success(text):
    print(f'  ✅ {text}')

def warning(text):
    print(f'  ⚠️  {text}')

def error_msg(text):
    print(f'  ❌ {text}')

def prompt(text):
    return input(f'  👉 {text}')


# ═══════════════════════════════════════════════════════════════════════
# WEIGHT SAVING HELPERS
# ═══════════════════════════════════════════════════════════════════════

def do_save_weights(net):
    """Save weights to last_w_torch.json + timestamped copy."""
    json_obj = net.save_weights_to_json()

    with open(WEIGHTS_FILE, 'w') as fp:
        json.dump(json_obj, fp)
    success(f'Weights saved to {WEIGHTS_FILE}')

    os.makedirs(WEIGHTS_HISTORY_DIR, exist_ok=True)
    history_file = os.path.join(
        WEIGHTS_HISTORY_DIR,
        f'weights_{time.strftime("%Y%m%d_%H%M%S")}.json'
    )
    with open(history_file, 'w') as fp:
        json.dump(json_obj, fp)
    success(f'Copy saved to {history_file}')


def ask_and_save_weights(net):
    """Ask the user, then save if yes."""
    choice = ''
    while choice not in ('y', 'n'):
        choice = prompt('Save weights from this session? (y/n) --> ').strip().lower()
    if choice == 'y':
        do_save_weights(net)
    else:
        warning('Weights NOT saved.')


# ═══════════════════════════════════════════════════════════════════════
# TELEOP PAUSE — wait for user to position robot via external teleop
# ═══════════════════════════════════════════════════════════════════════

def wait_for_teleop(robot):
    """
    Pause the script while the user positions the robot using
    teleop_twist_keyboard in a separate terminal.
    Displays live position feedback while waiting.
    """
    banner('POSITION THE ROBOT')
    print()
    info('In a separate terminal, run:')
    print()
    print('      ros2 run teleop_twist_keyboard teleop_twist_keyboard')
    print()
    info('Drive the robot to the desired starting position.')
    info('When done, close teleop (Ctrl+C in that terminal),')
    info('then come back here and press ENTER.')
    print()
    info('Live position (updates every 0.2s):')
    print()

    # Flag to stop the position display thread
    stop_display = threading.Event()

    def display_position():
        """Background thread: refresh position on the same line."""
        while not stop_display.is_set():
            pos = robot.get_position()
            sys.stdout.write(
                f'\r      x={pos[0]:+7.3f} m   y={pos[1]:+7.3f} m   '
                f'θ={math.degrees(pos[2]):+7.1f}°     '
            )
            sys.stdout.flush()
            stop_display.wait(0.2)

    # Start live display
    display_thread = threading.Thread(target=display_position, daemon=True)
    display_thread.start()

    # Wait for ENTER (blocks until user presses ENTER)
    input()

    # Stop the display thread
    stop_display.set()
    display_thread.join(timeout=1)

    # Print final position on a clean line
    pos = robot.get_position()
    print()
    success(f'Position locked: x={pos[0]:.2f}  y={pos[1]:.2f}  θ={math.degrees(pos[2]):.1f}°')
    return pos


# ═══════════════════════════════════════════════════════════════════════
# INITIALIZATION
# ═══════════════════════════════════════════════════════════════════════

banner('LIMO Neural Controller — Real Robot')
print()

robot   = ROS2LimoSimulation(cmd_vel_topic=CMD_VEL_TOPIC, odom_topic=ODOM_TOPIC)
monitor = RobotMonitorAdapter(world_bounds=WORLD_BOUNDS)
trainer = None

def cleanup():
    print('\n  Cleaning up...')
    if hasattr(robot, 'cleanup'):
        robot.cleanup()
    if hasattr(monitor, 'stop_monitoring'):
        monitor.stop_monitoring()

atexit.register(cleanup)
time.sleep(1)

# Monitoring always on
monitor.start_monitoring()

# Create neural network
network = PioneerNN(INPUT_SIZE, HL_SIZE, OUTPUT_SIZE)


# ═══════════════════════════════════════════════════════════════════════
# MODE SELECTION
# ═══════════════════════════════════════════════════════════════════════

weights_exist = os.path.isfile(WEIGHTS_FILE)

banner('MODE SELECTION')
print()
info('DEMO mode     — Uses pre-trained weights. No learning.')
info('                The robot goes directly to the target.')
print()
info('LEARNING mode — Progressive training.')
info('                Use teleop to position, then the robot learns.')
print()

if weights_exist:
    success(f'Pre-trained weights found: {WEIGHTS_FILE}')
else:
    warning(f'No weights file found ({WEIGHTS_FILE})')
    warning('DEMO mode is not available without pre-trained weights.')
print()

mode = ''
valid_modes = ['demo', 'learning'] if weights_exist else ['learning']
while mode not in valid_modes:
    if weights_exist:
        mode = prompt('Select mode (demo / learning) --> ').strip().lower()
    else:
        info('Starting in LEARNING mode (no weights file found).')
        mode = 'learning'
print()


# ═══════════════════════════════════════════════════════════════════════
# LOAD WEIGHTS
# ═══════════════════════════════════════════════════════════════════════

if mode == 'demo':
    with open(WEIGHTS_FILE) as fp:
        json_obj = json.load(fp)
    network.load_weights_from_json(json_obj, network.hidden_size)
    success('Weights loaded. Robot is ready for demonstration.')

elif mode == 'learning' and weights_exist:
    choice = ''
    while choice not in ('y', 'n'):
        choice = prompt('Load existing weights as starting point? (y/n) --> ').strip().lower()
    if choice == 'y':
        with open(WEIGHTS_FILE) as fp:
            json_obj = json.load(fp)
        network.load_weights_from_json(json_obj, network.hidden_size)
        success('Weights loaded.')
    else:
        info('Starting with random weights.')


# ═══════════════════════════════════════════════════════════════════════
# CREATE TRAINER
# ═══════════════════════════════════════════════════════════════════════

trainer = PyTorchOnlineTrainer(robot, network, monitor)
trainer.training = (mode == 'learning')


# ═══════════════════════════════════════════════════════════════════════
# SESSION RUNNER
# ═══════════════════════════════════════════════════════════════════════

def run_session(target, session_num):
    """
    Run a single training/demo session.
    Returns: 'ok' or 'interrupted'
    """
    banner(f'SESSION #{session_num}')
    pos = robot.get_position()
    info(f'Current position : x={pos[0]:.2f}  y={pos[1]:.2f}  θ={math.degrees(pos[2]):.1f}°')
    info(f'Target           : x={target[0]:.2f}  y={target[1]:.2f}  θ={math.degrees(target[2]):.1f}°')
    dist = math.sqrt((pos[0]-target[0])**2 + (pos[1]-target[1])**2)
    info(f'Distance to target: {dist:.2f} m')
    mode_str = 'LEARNING' if trainer.training else 'DEMO (no learning)'
    info(f'Mode: {mode_str}')
    print()
    info('The robot will start moving.')
    info('Press ENTER at any time to stop the session.')
    print()

    thread = threading.Thread(target=trainer.train, args=(target,))
    trainer.running = True
    thread.start()

    try:
        input()
        trainer.running = False
        thread.join(timeout=5)
        result = 'ok'
    except KeyboardInterrupt:
        trainer.running = False
        thread.join(timeout=5)
        result = 'interrupted'

    print()
    pos = robot.get_position()
    info(f'Final position: x={pos[0]:.2f}  y={pos[1]:.2f}  θ={math.degrees(pos[2]):.1f}°')

    if result == 'interrupted':
        info('Session interrupted by user (Ctrl+C)')
    else:
        success('Session stopped by user.')

    monitor.save_results(f'session_{session_num}_{time.strftime("%Y%m%d_%H%M%S")}')

    return result


# ═══════════════════════════════════════════════════════════════════════
# DEMO MODE
# ═══════════════════════════════════════════════════════════════════════

if mode == 'demo':
    banner('DEMO MODE')
    print()
    info('The robot will go to the targets you specify.')
    info('Use teleop in a separate terminal to position the robot.')
    print()

    session_count = 0
    continue_running = True

    while continue_running:
        session_count += 1

        # Wait for user to position robot via external teleop
        wait_for_teleop(robot)

        print()
        target_input = prompt('Enter target (x y theta_rad) --> ').strip()
        try:
            target = [float(v) for v in target_input.split()]
            if len(target) != 3:
                raise ValueError()
        except ValueError:
            error_msg('Need exactly 3 values: x y theta_rad')
            continue

        run_session(target, session_count)

        print()
        choice = ''
        while choice not in ('y', 'n'):
            choice = prompt('Run another demo? (y/n) --> ').strip().lower()
        continue_running = (choice == 'y')


# ═══════════════════════════════════════════════════════════════════════
# LEARNING MODE
# ═══════════════════════════════════════════════════════════════════════

elif mode == 'learning':
    banner('LEARNING MODE')
    print()
    info('Training protocol:')
    info('  1. Use teleop in a separate terminal to position the robot')
    info('  2. Enter the target coordinates')
    info('  3. The robot learns to reach the target')
    info('  4. Repeat from different positions and distances')
    print()
    info('Tip: start close to the target (0.5m), then increase distance.')
    info('Tip: the target (0, 0, 0) is the position where the robot booted.')
    print()

    session_count    = 0
    continue_running = True

    while continue_running:
        session_count += 1

        # Wait for user to position robot via external teleop
        wait_for_teleop(robot)

        print()
        target_input = prompt('Enter target (x y theta_rad) --> ').strip()
        try:
            target = [float(v) for v in target_input.split()]
            if len(target) != 3:
                raise ValueError()
        except ValueError:
            error_msg('Need exactly 3 values: x y theta_rad')
            continue

        # Ask learning on/off for this session
        choice_learn = ''
        while choice_learn not in ('y', 'n'):
            choice_learn = prompt('Enable learning for this session? (y/n) --> ').strip().lower()
        trainer.training = (choice_learn == 'y')

        run_session(target, session_count)

        print()
        choice = ''
        while choice not in ('y', 'n'):
            choice = prompt('Continue? (y/n) --> ').strip().lower()
        continue_running = (choice == 'y')


# ═══════════════════════════════════════════════════════════════════════
# FINAL SAVE
# ═══════════════════════════════════════════════════════════════════════

banner('SESSION COMPLETE')

ask_and_save_weights(network)

monitor.save_results(f'final_results_{time.strftime("%Y%m%d_%H%M%S")}')
success('Monitoring results saved.')
print()
info('Thank you for using the LIMO Neural Controller!')
print()