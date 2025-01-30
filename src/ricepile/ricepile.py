from picamzero import Camera
import serial
import RPi.GPIO as gpio
import datetime

from time import sleep
import os

gpio.setmode(gpio.BCM)


class Feeder(object):

    def __init__(self, feed_rate=None, calib_file=None, rps=None, ppr=1600, PUL=17, DIR=27, ENA=22):

        self.PUL = PUL
        self.DIR = DIR
        self.ENA = ENA

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
        self.pulse_delay = 1/(self.rps * self.ppr * 2)

    def calculate_pulses(self, dur=None, rot=None):
        if dur:
            self.pulses = int(dur / self.pulse_delay / 2)
        elif rot:
            self.pulses = int(self.ppr * rot)
        else:
            raise ValueError("specify either duration to rotate or rotations to execute")

    def enable(self, direction='cw'):
        self.enabled = True
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

    def disable(self):
        gpio.output(self.ENA, gpio.LOW)
        sleep(25e-6)
        self.enabled = False

    def cw(self):
        gpio.output(self.DIR, gpio.LOW)

    def ccw(self):
        gpio.output(self.DIR, gpio.HIGH)

    def go_finite(self, dur=None, rot=None, direction="cw", rps=None):
        if dur:
            self.calculate_pulses(dur = dur)
        elif rot:
            self.calculate_pulses(rot = rot)
        else:
            raise ValueError("specify a finite number of rotations")

        self.enable(direction=direction)

        for p in range(self.pulses):
            sleep(self.pulse_delay)
            gpio.output(self.PUL, gpio.HIGH)
            sleep(self.pulse_delay)
            gpio.output(self.PUL, gpio.LOW)

        self.disable()

class Balance(object):

    # example parameters needed for MJ's scale.
    # s = serial.Serial('/dev/ttyUSB1', baudrate=1200, parity=serial.PARITY_ODD, stopbits=serial.STOPBITS_ONE, bytesize=serial.SEVENBITS)

    def __init__(self, dev=None, file=None, **scaleargs):
        if not dev:
            raise ValueError("specify path to serial device.")
        self.ser = serial.Serial(dev, **scaleargs)
        self.logging=False
        self.file = file
        if not os.path.exists(os.path.dirname(self.file)):
            raise ValueError('path to file directory does not exist')
        if os.path.isfile(self.file):
            self.dataexists = True
            #######
            # complementary stand-in segment to check-still-logging()
            with open(self.file, 'r') as f:
                self.datalines = len(f.readlines())
            #######
        elif os.path.basename(self.file) == "":
            raise ValueError("must provide valid file path")
        else:
            with open(self.file, "a") as f:
                _ = f.write("datetime, mass")
                self.datalines = 0

    def check_still_logging(self, thresh=100):
        # This is a stand-in for some other method of halting data collection via input.
        if self.datalines > thresh:
            self.logging = False
        self.datalines += 1

    def stop_logging(self):
        self.logging = False

    def start_logging(self):
        self.logging = True
        self.ser.reset_input_buffer()
        with open('massdata.txt', 'w+') as f:
            buffer = ''
            buffer += self.ser.read().decode()
            while '\n' not in buffer:
                buffer += s.read().decode()
            while self.logging:
                w = self.ser.read_until().decode()
                _ = f.write(datetime.datetime.now().strftime("%d-%m-%Y %H:%M:%S.%f") + ', ' + w[7:17] + '\n')
                self.check_still_logging()

if __name__=="__main__":

    # cam = Camera()
    today = datetime.datetime.now().strftime("%Y-%m-%d")
    mjscale = {"baudrate":1200, "parity":"serial.PARITY_ODD", "stopbits":"serial.STOPBITS_ONE", "bytesize":"serial.SEVENBITS"}

    f = Feeder(rps=0.24)

    b = Balance(dev="/dev/ttyUSB1", file=os.path.expanduser(f"~/Desktop/{today}_data.csv"), **mjscale)

    b.start_logging()

    f.go_finite(dur=3, direction='cw')
    f.cleanup()
