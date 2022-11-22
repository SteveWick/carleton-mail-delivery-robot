#!/usr/bin/env python
# @author: Emily Clarke, Jacob Charpentier

import csv
import os

# ~~~~ DEFAULTS ~~~~~
magicNumbers = {
    # actionTranslator.py
    'ZERO_SPEED': 0.0,
    'FORWARD_X_SPEED': 0.2,
    'SLOW_FORWARD_X_SPEED': 0.1,
    'CREEP_FORWARD_X_SPEED': 0.05,
    'BACKWARD_X_SPEED': -0.2,
    'LEFT_Z_SPEED': 3.5,
    'RIGHT_Z_SPEED': -3.5,
    'SLEFT_X_SPEED': 0.05,
    'SLEFT_Z_SPEED': 0.5,
    'SRIGHT_X_SPEED': 0.05,
    'SRIGHT_Z_SPEED': -0.5,
    'AVOIDRIGHT_X_SPEED': 0.08,
    'AVOIDRIGHT_Z_SPEED': -0.5,
    'BLEFT_X_SPEED': -0.1,
    'BLEFT_Z_SPEED': 0.5,

    # robotDriver.py
    'MAX_TARGET_WALL_DISTANCE': 20.0,
    'MIN_TARGET_WALL_DISTANCE': 10.0,
    'MAX_TARGET_WALL_ANGLE': 120.0,
    'MIN_TARGET_WALL_ANGLE': 60.0,
    'FIND_WALL_SPIN_TARGET_WALL_DISTANCE': 15.0,
    'NO_WALL_RIGHT_TURN_DISTANCE': 20.0,
    'COLLISION_OBJECT_PASSED_DISTANCE': 20.0,
    'FIND_WALL_TICKS': 15,
    'FIND_WALL_SPIN_TICKS': 5,
    'RIGHT_TURN_TICKS': 3,
    'RIGHT_TURN_FORWARD_TICKS': 5,
    'COLLISION_BACK_TICKS': 2,
    'COLLISION_LEFT_TICKS': 5,
    'COLLISION_RETURN_MIN_TICKS': 6,
    'COLLISION_WALL_FOLLOW_TICKS': 40,
    'COLLISION_RETURN_RIGHT_TICKS': 2,
    'COLLISION_RETURN_FORWARD_TICKS': 4,
    'COLLISION_RETURN_WALL_FOLLOW_TICKS': 30,
    'GRAZE_SLEFT_TICKS': 3,
    'GRAZE_WALL_FOLLOW_TICKS': 3,
    'TIMER_PERIOD': 0.2,

    # captain.py
    'BEACON_OUTLIER_THRESHOLD': 7,

    # bumperSensor.py
    'MAX_BUMP_EVENT_PUBLISH_TICKS': 30
}

# ~~~~ Load overrides ~~~~
def loadNumberOverrides():
    magicNumbers = {}
    ROOT_DIR = os.getcwd()
    with open(f'{ROOT_DIR}/src/carleton-mail-delivery-robot/mail_delivery_robot/magicNumbers.csv') as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        for row in reader:
            magicNumbers[row[0]] = row[1]
    return magicNumbers
