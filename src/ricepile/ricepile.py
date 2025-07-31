"""Rice pile experiment control module.

This module provides classes for controlling a rice pile experiment setup, including a stepper
motor feeder and digital scale interface.
"""

import datetime
import os
import re
from multiprocessing import Process, Pipe
from tempfile import NamedTemporaryFile
from time import sleep, monotonic

# from distutils.util import strtobool

import serial
from picamzero import Camera # type: ignore

def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")

# import RPi.GPIO as gpio
# # Set GPIO mode
# gpio.setmode(gpio.BCM)

class riceCamera:
    """A class to control the camera for capturing images of the rice pile.
    
    This class uses the picamzero library to interface with a Raspberry Pi camera.
    It provides methods to capture images and manage camera settings.
    """

    def __init__(self):
        """Initialize the riceCamera."""
        self.camera = Camera()
        self.camera.start_preview()

    def capture_image(self, filename):
        """Capture an image and save it to the specified filename.
        
        Args:
            filename (str): The path where the image will be saved.
        """
        self.camera.capture(filename)

    def stop_preview(self):
        """Stop the camera preview."""
        self.camera.stop_preview()

class Feeder:
    """A class to control a stepper motor for feeding rice at a specified rate.
    
    This class manages the stepper motor control through GPIO pins, allowing for
    precise control of rotation speed and direction. It's designed to work with
    a stepper motor driver that accepts pulse and direction signals.
    """

    def __init__(
        self,
        port="/dev/ttyACM1",  # Serial port for the stepper motor controller
        dataport="/dev/ttyACM2",  # Serial port for the stepper motor controller
        feed_rate=None,
        calib_file=None,
        rps=0.5,  # Default rotations per second
        ppr=1600, # Pulses per revolution, adjust based on your motor and driver
        # dont have a good way to specify this yet, so this does nothing for now.
    ):
        """Initialize the Feeder with motor control parameters.
        
        Args:
            feed_rate (float, optional): Target feed rate in units per second
            calib_file (str, optional): Path to calibration file linking rps to feed rate
            rps (float): Rotations per second for the motor, defaults to 0.5 Hz.
            ppr (int, optional): Pulses per revolution. Defaults to 1600. Not implemented yet.
        """

        self.fp = serial.Serial(port)
        self.dp = serial.Serial(dataport)

        self.fp.write(b'\x03')
        sleep(2)  # Wait for the controller to exit
        self.fp.write(b'\x04')
        sleep(4)  # Wait for the controller to reboot

        self.calib_file = calib_file
        ################################
        # TODO: in this block, specify a calibration file that links rps to feed rate of rice.


        ################################

        # need to do rev per pulse math to make this easier
        # whatever motor you have, there will be some pulse-per-rev setting
        self.rps = rps
        self.ppr = ppr

        # use get statemem

        self.running = str2bool(self.get('running'))
        #  self.fp.write('get:running'.encode)self.dp.
        self.enabled = False
        self.direction = -1  # Default direction
        self.dirconv = lambda x: "CW" if x<0 else "CCW" if x>0 else "Problem..."

        # self.last_encoder_value = 0

        # thought: do I want to allow a rotary encoder? probably. This would be the 
        # ultimate goal for an exhibit.
        # may have to restructure code allow for gpio input and interruption. 
        # A start-stop switch or button would be nice too.
        # maybe switch for direction, and button for start/stop
        # small OLED display for displaying feed rate, the scale mass, and maybe 
        # the status of various things?
        # could also have some regular color LEDs for feedback.

    def get(self, statuscode):
        """Send a command to the motor controller and return the response.
        
        Args:
            command (str): The command to send to the motor controller.
        
        Returns:
            str: The response from the motor controller.
        """
        self.fp.write(f"get:{statuscode}\n".encode())
        return self.dp.readline().decode().strip()
    
    def speed(self, value):
        """Send a command to the motor controller and return the response.
        
        Args:
            command (str): The command to send to the motor controller.
        
        Returns:
            str: The response from the motor controller.
        """
        self.fp.write(f"speed={value}\n".encode())
        # return self.dp.readline().decode().strip()

    def enable(self):
        """Enable the motor.
        """
        self.enabled = True
        self.fp.write("enable\n".encode())

    def disable(self):
        """Disable the motor by setting the enable pin low."""
        self.enabled = False
        self.fp.write("disable\n".encode())
    
    def change_direction(self):
        self.fp.write("switch_direction\n".encode())
        self.direction *= -1  # Toggle direction
    
    def get_direction(self):
        """Get the current direction of the motor.
        
        Returns:
            str: "CW" for clockwise, "CCW" for counter-clockwise, or an error message.
        """
        return self.dirconv(self.direction)

    def start(self):
        """Start the motor, even if disabled."""
        self.enable()
        if not self.running: 
            self.fp.write('toggle\n'.encode())
            self.running = str2bool(self.get('running'))
        else:
            pass  # already running, do nothing

    def toggle(self):
        """Toggle motor on-off."""
        self.fp.write('toggle\n'.encode())
        self.running = str2bool(self.get('running'))

    def stop(self):
        """Stop the motor and disable it."""
        self.disable()
        self.fp.write("stop\n".encode())

    def reboot_controller(self):
        """Reboot the motor controller."""
        self.fp.write(b'\x03')
        sleep(2)  # Wait for the controller to exit
        self.fp.write(b'\x04')
        sleep(4)  # Wait for the controller to reboot

    # def read_encoder(self):
    #     """Read the rotary encoder value and update the rps."""
    #     current_value = gpio.input(self.encoder_a)  # Read the encoder value
    #     if current_value != self.last_encoder_value:
    #         # Update rps based on encoder value
    #         self.rps = self.calculate_rps_from_encoder()  # Implement this method based on your encoder logic
    #     self.last_encoder_value = current_value

    def calibrate(self, balance, rps_list, dur, output_file, sample_interval=0.5):
        """Run calibration sequence varying RPS values and record mass data to create a calibration file."""
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        with open(output_file, 'w') as f:
            f.write('rps,total_mass,feed_rate\n')
            for rps in rps_list:
                self.speed(rps)
                process = Process(target=self, kwargs={'dur': dur})
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

    def __init__(self, dev=None, file='', **scaleargs):
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
        ww = re.findall(r"\d+\.\d+", w)
        try:
            return float(ww[0])
        #try:
        #    return float(ww.strip())
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
