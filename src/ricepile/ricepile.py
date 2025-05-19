"""Rice pile experiment control module.

This module provides classes for controlling a rice pile experiment setup, including a stepper
motor feeder and digital scale interface.
"""

import datetime
import os
from multiprocessing import Process, Pipe
from tempfile import NamedTemporaryFile
from time import sleep, monotonic

import RPi.GPIO as gpio
import serial
from picamzero import Camera

# Set GPIO mode
gpio.setmode(gpio.BCM)


class Feeder:
    """A class to control a stepper motor for feeding rice at a specified rate.
    
    This class manages the stepper motor control through GPIO pins, allowing for
    precise control of rotation speed and direction. It's designed to work with
    a stepper motor driver that accepts pulse and direction signals.
    """

    def __init__(
        self,
        feed_rate=None,
        calib_file=None,
        rps=None,
        ppr=1600,
        PUL=17,
        DIR=27,
        ENA=22
    ):
        """Initialize the Feeder with motor control parameters.
        
        Args:
            feed_rate (float, optional): Target feed rate in units per second
            calib_file (str, optional): Path to calibration file linking rps to feed rate
            rps (float): Rotations per second for the motor
            ppr (int, optional): Pulses per revolution. Defaults to 1600
            PUL (int, optional): GPIO pin for pulse signal. Defaults to 17
            DIR (int, optional): GPIO pin for direction signal. Defaults to 27
            ENA (int, optional): GPIO pin for enable signal. Defaults to 22
        """
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

        self.running = False

        # thought: do I want to allow a rotary encoder? probably. This would be the 
        # ultimate goal for an exhibit.
        # may have to restructure code allow for gpio input and interruption. 
        # A start-stop switch or button would be nice too.
        # maybe switch for direction, and button for start/stop
        # small OLED display for displaying feed rate, the scale mass, and maybe 
        # the status of various things?
        # could also have some regular color LEDs for feedback.

    def cleanup(self):
        """Clean up GPIO resources when done with the feeder."""
        gpio.cleanup()

    def calculate_delay(self):
        """Calculate the delay between pulses based on desired rotation speed.
        
        The delay is calculated as: 1 / (rps * ppr * 2)
        where rps is rotations per second and ppr is pulses per revolution.
        """
        self.pulse_delay = 1 / (self.rps * self.ppr * 2)

    def calculate_pulses(self, dur=None, rot=None):
        """Calculate the number of pulses needed for a given duration or rotation.
        
        Args:
            dur (float, optional): Duration in seconds
            rot (float, optional): Number of rotations
            
        Raises:
            ValueError: If neither duration nor rotations are specified
        """
        if dur:
            self.pulses = int(dur / self.pulse_delay / 2)
        elif rot:
            self.pulses = int(self.ppr * rot)
        else:
            raise ValueError("specify either duration to rotate or rotations to execute")

    def enable(self, direction='cw'):
        """Enable the motor and set its direction.
        
        Args:
            direction (str, optional): Direction of rotation ('cw' or 'ccw'). Defaults to 'cw'
            
        Raises:
            ValueError: If direction is not 'cw' or 'ccw'
        """
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
            raise ValueError("direction must be either 'cw' or 'ccw'")

    def disable(self):
        """Disable the motor by setting the enable pin low."""
        gpio.output(self.ENA, gpio.LOW)
        sleep(25e-6)
        self.enabled = False

    def cw(self):
        """Set motor direction to clockwise."""
        gpio.output(self.DIR, gpio.LOW)

    def ccw(self):
        """Set motor direction to counter-clockwise."""
        gpio.output(self.DIR, gpio.HIGH)

    def start(self):
        """Start the motor and create a running state file."""
        self.enable(direction='cw')  # Fixed undefined direction variable
        self.running = True
        self.running_file = NamedTemporaryFile()

    def stop(self):
        """Stop the motor and clean up the running state file."""
        self.disable()
        self.running_file.close()

    def go_infinite(self, rps=None, direction="cw"):
        """Run the motor continuously until stopped.
        
        Args:
            rps (float, optional): Rotations per second
            direction (str, optional): Direction of rotation. Defaults to "cw"
        """
        self.start()
        self.calculate_delay()

        while os.path.exists(self.running_file):
            pass

        self.stop()

    def go_finite(self, dur=None, rot=None, direction="cw", rps=None, pipe=None):
        """Run the motor for a specific duration or number of rotations.
        
        Args:
            dur (float, optional): Duration in seconds
            rot (float, optional): Number of rotations
            direction (str, optional): Direction of rotation. Defaults to "cw"
            rps (float, optional): Rotations per second
            pipe (multiprocessing.Pipe, optional): Pipe for inter-process communication
            
        Raises:
            ValueError: If neither duration nor rotations are specified
        """
        if dur:
            self.calculate_pulses(dur=dur)
        elif rot:
            self.calculate_pulses(rot=rot)
        else:
            raise ValueError("specify a finite number of rotations")

        self.start()

        for p in range(self.pulses):
            sleep(self.pulse_delay)
            gpio.output(self.PUL, gpio.HIGH)
            sleep(self.pulse_delay)
            gpio.output(self.PUL, gpio.LOW)

        self.stop()

    def calibrate(self, balance, rps_list, dur, output_file, sample_interval=0.5):
        """Run calibration sequence varying RPS values and record mass data to create a calibration file."""
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        with open(output_file, 'w') as f:
            f.write('rps,total_mass,feed_rate\n')
            for rps in rps_list:
                self.rps = rps
                self.calculate_delay()
                process = Process(target=self.go_finite, kwargs={'dur': dur})
                process.start()
                total_mass = 0.0
                # sample mass until feeder process completes
                while process.is_alive():
                    try:
                        mass = balance.read_mass()
                        total_mass += mass
                    except Exception:
                        pass
                    sleep(sample_interval)
                process.join()
                feed_rate = total_mass / dur if dur else 0
                f.write(f'{rps},{total_mass},{feed_rate}\n')


class Balance:
    """A class to interface with a digital balance/scale and log mass measurements.
    
    This class handles serial communication with a digital scale and provides
    functionality to log mass measurements to a CSV file with timestamps.
    """

    # example parameters needed for MJ's scale.
    # s = serial.Serial('/dev/ttyUSB1', baudrate=1200, parity=serial.PARITY_ODD, 
    # stopbits=serial.STOPBITS_ONE, bytesize=serial.SEVENBITS)

    def __init__(self, dev=None, file=None, **scaleargs):
        """Initialize the Balance with serial device and logging parameters.
        
        Args:
            dev (str): Path to the serial device
            file (str): Path to the output CSV file
            **scaleargs: Additional serial port configuration parameters
            
        Raises:
            ValueError: If device path is not specified or file path is invalid
        """
        if not dev:
            raise ValueError("specify path to serial device.")
            
        self.ser = serial.Serial(dev, **scaleargs)
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
                _ = f.write("datetime, mass\n")
                self.datalines = 0
        self.ser.close()

    # def check_still_logging(self, thresh=100):
    #     # This is a stand-in for some other method of halting data collection via input.
    #     if self.datalines > thresh:
    #         self.logging = False
    #     self.datalines += 1

    # def stop_logging(self):
    #     self.logging = False

    def log_data(self):
        """Read mass data from the scale and log it to the CSV file with timestamp."""
        mass = self.read_mass()
        with open(self.file, 'a') as f:
            f.write(f"{datetime.datetime.now().strftime('%d-%m-%Y %H:%M:%S.%f')}, {mass}\n")

    def read_mass(self):
        """Read a single mass measurement from the scale and return it as a float.

        This method opens the serial connection to the digital scale, reads the mass data,
        and attempts to parse it into a float. It handles potential errors in parsing and
        ensures that the connection is closed after reading.

        Returns:
            float: The mass measurement read from the scale. If the reading fails, it returns 0.0.
        """
        self.ser.open()
        self.ser.reset_input_buffer()
        buffer = ''
        buffer += self.ser.read().decode()
        while '\n' not in buffer:
            buffer += self.ser.read().decode()
        w = self.ser.read_until().decode()
        self.ser.close()
        try:
            return float(w[7:17])
        except ValueError:
            try:
                return float(w.strip())
            except ValueError:
                return 0.0


class Pile:
    """A class to represent and manage a rice pile.
    
    This class is intended to handle the physical characteristics and behavior
    of the rice pile being created by the feeder.
    """

    def __init__(self):
        """Initialize the Pile object."""
        pass


if __name__ == "__main__":

    # cam = Camera()
    today = datetime.datetime.now().strftime("%Y-%m-%d")
    mjscale = {
        "baudrate": 19200,
        "parity": serial.PARITY_ODD,
        "stopbits": serial.STOPBITS_ONE,
        "bytesize": serial.SEVENBITS
    }

    f = Feeder(rps=0.24)

    b = Balance(dev="/dev/ttyUSB0", file=os.path.expanduser(f"~/Desktop/{today}_data.csv"), **mjscale)

    starttime = monotonic()
    fp = Process(target=f.go_finite, kwargs={'dur': 30})
    fp.start()

    while os.path.exists(f.running_file):
        b.log_data()
        sleep(0.5 - (monotonic() - starttime) % 0.5)

    fp.join()

    f.cleanup()
