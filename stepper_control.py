#!/usr/bin/env python
"""
Stepper motor control file using ROS
HK2020 Autoplant 1
Author: {}
Idea: Use ROS for node communication
Use RPI PIN to control stepper motor
TODO Initial code without microstepping, comment out microsteppingcode
"""

from time import sleep
import RPi.GPIO as GPIO
import rospy

class RpiControl:
    def __init__(self):
        """
        Initialize all stepper motor control variables
        """
        self.dir_pin = 1
        self.step_pin = 2
        self.clockwise = 3
        self.counter_clockwise = 4
        self.steps = 200 # 360/1.8 = 200

        """
        Raspberry Pi PIN setup
        """
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)

    def turn_clockwise(self, turns = 0):
        """
        Turns motor clockwise direction
        :param turns: How many turns of motor user wants
        :return: Spins the motor set amount of turns
        """
        GPIO.output(self.dir_pin, self.clockwise)
        for y in range(turns):
            # Y turns of motor
            for x in range(self.steps):
                # performs full 360 degree rotation of motor
                GPIO.output(self.step_pin, GPIO.HIGH)
                sleep(delay)
                GPIO.output(self.step_pin, GPIO.LOW)

    def turn_counter_clockwise(self, turns = 0):
        """
        Turns motor counter clockwise direction
        :param turns: How many turns of motor user wants
        :return: Spins the motor set amount of turns
        """
        GPIO.output(self.dir_pin, self.counter_clockwise)
        for y in range(turns):
            # Y turns of motor
            for x in range(self.steps):
                # performs full 360 degree rotation of motor
                GPIO.output(self.step_pin, GPIO.HIGH)
                sleep(delay)
                GPIO.output(self.step_pin, GPIO.LOW)





