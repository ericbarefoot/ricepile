from picamzero import Camera
import serial
import RPi.GPIO as gpio

from time import sleep
import os

class Feeder(object):

	def __init__(self, feed_rate=None, calib_file=None, rps=None, ppr=800, PUL=17, DIR=27, ENA=22):

        self.PUL = PUL
        self.DIR = DIR
        self.ENA = ENA

        gpio.setmode(gpio.BCM)
		gpio.setup(self.PUL, gpio.OUT)
		gpio.setup(self.DIR, gpio.OUT)
		gpio.setup(self.ENA, gpio.OUT)

        ################################
        # TODO: in this block, specify a calibration file that links rps to feed rate of rice.


        ################################

		# need to do rev per pulse math to make this easier
        # whatever motor you have, there will be some pulse-per-rev setting
        if not rps:
            raise ValueError("must specify rotation frequency")
        self.rps = rps
        self.ppr = ppr
        self.calculate_delay()

        # thought: do I want to allow a rotary encoder? probably. This would be the ultimate goal for an exhibit.
        # may have to restructure code allow for gpio input and interruption. A start-stop switch or button would be nice too.
        # maybe switch for direction, and button for start/stop
        # small OLED display for displaying feed rate, the scale mass, and maybe the status of various things?
        # could also have some regular color LEDs for feedback.

    def cleanup(self):
        gpio.cleanup()

    def calculate_delay(self):
        # rps is angular velocity in hertz, so rotations per second
        # ppr is pulses per rotation as set on motor
        # want seconds per pulse
        self.pulse_delay = 1/(self.rps * self.ppr)

    def calculate_pulses(self, dur, rot):
        if dur:
            self.pulses = dur / self.pulse_delay
        elif rot:
            self.pulses = self.ppr / rot
        else:
            raise ValueError("specify either duration to rotate or rotations to execute")

	def enable(self, direction='cw'):
        gpio.output(self.ENA, gpio.HIGH)
        sleep(25e-6)
        if direction == 'cw':
            self.cw()
            sleep(125e-6)
        elif direction == 'ccw':
            self.ccw()
            sleep(125e-6)
        else:
            raise ValueError("what direction do you want to go anyway? there's only two options")

    def cw(self):
		gpio.output(self.DIR, gpio.LOW)

    def ccw(self):
        gpio.output(self.DIR, gpio.HIGH)

    def go_finite(self, dur=5, direction="cw", rot=None, rps=None):
        if dur:
            self.calculate_pulses(dur = dur)
        elif rot:
            self.calculate_pulses(rot = rot)
        else:
            raise ValueError("specify a finite number of rotations")

        self.enable(direction=direction)

        for p in range(self.pulses):
            gpio.output(self.PUL, gpio.HIGH)
            sleep(self.pulse_delay)
            gpio.output(self.PUL, gpio.LOW)
            sleep(self.pulse_delay)

class Balance(object):

	def __init__(self, dev=None, br=19200):
        if not dev:
            raise ValueError("specify path to serial device.")
		self.ser = serial.Serial(dev, baudrate=br)
		self.logging=False

	def start_logging(self):
		self.logging = True
		while self.logging:
			self.ser.read()

if __name__=="__main__":
	# cam = Camera()
    f = Feeder(rps=0.24)

    f.go_finite(dur=3, direction='cw')
    f.cleanup()
