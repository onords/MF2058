#!/usr/bin/env python
"""
Stepper motor control file using ROS
HK2020 Autoplant 1
Author: {}
Idea: Use ROS for node communication
Use RPI PIN to control stepper motor
TODO Initial code without microstepping, comment out microsteppingcode
RPi.GPIO is for Raspberry Pi PIN control (installed in Rpi OS)
rospy (idea is to use ROS for communication between subsystems)
"""

import time
import RPi.GPIO as GPIO
import rospy


class MoreThanOneTurn(RosException):
    pass


class RpiControl:
    def __init__(self):
        """
        Initialize all stepper motor control variables
        """
        self.dir_pin = 1
        self.step_pin = 2
        self.clockwise = 3
        self.counter_clockwise = 4
        self.steps = 200  # 360/1.8 = 200
        self.delay = 0.001  # A delay between voltage change to allow the stepper motor to move a step

        """
        Raspberry Pi PIN setup
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)

    def turn_clockwise(self, turns = 0, part_turn = 0):
        """
        Turns motor clockwise direction
        :param part_turn: How much of a lap should be turned
        :param turns: How many turns of motor user wants
        :return: Spins the motor set amount of turns
        """
        if part_turn > self.steps:
            raise MoreThanOneTurn()

        GPIO.output(self.dir_pin, self.clockwise)
        for y in range(turns):
            # Y turns of motor
            for x in range(part_turn):
                # performs full 360 degree rotation of motor
                GPIO.output(self.step_pin, GPIO.HIGH)
                time.sleep(self.delay)
                GPIO.output(self.step_pin, GPIO.LOW)

    def turn_counter_clockwise(self, turns = 0, part_turn = 0):
        """
        Turns motor counter clockwise direction
        :param part_turn: How much of a lap should be turned
        :param turns: How many turns of motor user wants
        :return: Spins the motor set amount of turns
        """
        if part_turn > self.steps:
            raise MoreThanOneTurn()

        GPIO.output(self.dir_pin, self.counter_clockwise)
        for y in range(turns):
            # Y turns of motor
            for x in range(part_turn):
                # performs full 360 degree rotation of motor
                GPIO.output(self.step_pin, GPIO.HIGH)
                time.sleep(self.delay)
                GPIO.output(self.step_pin, GPIO.LOW)

    def one_round_movement(self):
        """
        This function moves a tray from buffer position
        To dropping of plants
        And back to buffer
        :param in_position: Boolean
        :return: Boolean
        """
        in_buffer_position = True
        loaded = True
        # TODO Add listen to ros topics for above variables
        if loaded:
            self.turn_clockwise(1, 2)  # Add necessary amount of revolutions to go initial position
            plant_drop = 5
            for i in range(plant_drop):
                time.sleep(0.5)
                self.turn_counter(1, 2)
            while in_buffer_position:
                self.turn_counter_clockwise(1,2)
                # TODO Add a line for listening if in_buffer_position to rostopic


if __name__ == '__main__':
    # Some Test Code
    print("Some Test Code Is Not Added Yet")




