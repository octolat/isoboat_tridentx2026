"""A MARLIN robot with tank drive controls, servo control, and sensor reading printouts."""

from __future__ import annotations
from dataclasses import dataclass, field
import time
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np  
import yaml
from marlin import Robot, Thruster, Servo, IMU, RadioBeacon

import os

# Get and print the current working directory
cwd = os.getcwd()
print(cwd)

# defines
XYZ_ARR = ["x", "y", "z"]

# per users
TRIDENTXPATH = "C:/Users/tansi/Documents/projects/tridentx"

# Brain port definitions
LEFT_THRUST = "M8" 
RIGHT_THRUST = "M1"
LEFT_THRUST_SECONDARY = "M7" 
RIGHT_THRUST_SECONDARY = "M6" 

ARM = "M2"

IMU_PORT = "S4"
RADIOBEACON_PORT = "S6"




# create our robot object depending on what we have
robot = Robot()
robot.configure(
    sensors={
        IMU_PORT: IMU, 
        RADIOBEACON_PORT: RadioBeacon
    },
    effectors={
        LEFT_THRUST: Thruster, 
        RIGHT_THRUST: Thruster, 
        LEFT_THRUST_SECONDARY: Thruster, 
        RIGHT_THRUST_SECONDARY: Thruster, 
        ARM: Servo
    },
)


def initialize(robot):
    time.sleep(0.5)
    print("Battery:", robot.battery_voltage, "V")

def driver(robot):
    while robot.running:
        # Drive forward with joystick ONE and THREE.
        left = robot.controller.joystick("ONE").value
        right = robot.controller.joystick("THREE").value

        robot.thruster(LEFT_THRUST).set_duty(left)
        robot.thruster(RIGHT_THRUST).set_duty(right)

        # Set Servo speed to 180 deg/s
        robot.servo(ARM).set_speed(180)

        if robot.controller.button("LEFT_TRIGGER").is_down:
            robot.servo(ARM).angle(120)
        else:
            robot.servo(ARM).angle(0)

        # print IMU readings
        imu_reading = robot.imu(IMU_PORT).read()
        if imu_reading.ok:
            print(imu_reading.mag_x)
            # imu_reading.gyro()

        # print RadioBeacon readings
        radiobeacon_reading = robot.radiobeacon(RADIOBEACON_PORT).read()
        if radiobeacon_reading.ok:
            print(radiobeacon_reading)

def autonomous(robot):
    while robot.running:
        pass

class SettingsManager:
    def __init__(self, ignore_list=[]):
        self.ignore_list = ignore_list

    def importData(self, path):
        with open(path, "r") as file:
            data = yaml.safe_load(file)
            for key in dir(self):
                if key in data:
                    if isinstance(data[key], list):
                        v = np.array(data[key])
                    else:
                        v = data[key]
                    setattr(self, key, v)

    def exportData(self, path):
        data = {}
        for key in dir(self):
            if key[0] != '_':
                if not callable(getattr(self, key)) and key != "ignore_list":
                    skip = False
                    for type in self.ignore_list:
                        if isinstance(getattr(self, key), type):
                            skip = True
                            break
                    if not skip:
                        v = getattr(self, key)
                        if isinstance(v, np.ndarray):
                            v = v.tolist()
                        data[key] = v
        with open(path, "w") as file:
            yaml.dump(data, file)

class ImuCalibration(SettingsManager):
    # "imu_calib": {
    #     'x': {'max': 342, 'min': -201, 'count': 912259}, 
    #     'y': {'max': 558, 'min': -30, 'count': 843805}, 
    #     'z': {'max': 320, 'min': -238, 'count': 943009}
    # }
    def __init__(
            self, 
            sensing,
            load_from_path=None,
            mag_min=np.full(3, np.nan),
            mag_max=np.full(3, np.nan),
            mag_count=np.full(3, np.nan),
        ):
        super().__init__(ignore_list=[Sensing])

        self.sensing = sensing

        self.mag_min = mag_min
        self.mag_max = mag_max
        self.mag_count = mag_count


        if not load_from_path is None:
            self.importData(load_from_path) 

        # check if calibration is done
        if np.isnan(self.mag_min).any() or np.isnan(self.mag_max).any() or np.isnan(self.mag_count).any():
            print("One or more mag calibrations are not set. Starting manual calibration")
            self.calibrateMag()

        self.exportData(f"{TRIDENTXPATH}/imu_calibration.yaml")

    # bounds of each axis is needed to calculate yaw with magnometer
    def calibrateMag(self, print_results = True, duration = 10): 
        for i, axis in enumerate(XYZ_ARR):
            self.mag_count[i] = 0
            print(f"rotate along the {axis} axis. send input when ready.")
            input()
            tstart = time.time()
            values_list = {
                'x': [],
                'y': [],
                'z': [],
            }
            while (tstart + duration > time.time() and self.sensing.robot.running):
                result = self.sensing._readRaw()
                if result == False:
                    print("reading failed, trying again.")
                    continue
                reading = self.sensing.raw_mag[i]
                print(reading)
                if np.isnan(self.mag_max[i]) or self.mag_max[i] < reading:
                    self.mag_max[i] = reading
                if np.isnan(self.mag_min[i]) or self.mag_min[i] > reading:
                    self.mag_min[i] = reading
                self.mag_count[i] += 1
                values_list[axis].append(reading)

        if (print_results):
            print("Pastable Results:\n")
            print(f"mag_max= np.array({self.mag_max}),")
            print(f"mag_min= np.array({self.mag_min}),")
            print(f"mag_count= np.array({self.mag_count}),")

            self.plotMag(mode="provided", x=values_list["x"], y=values_list["y"])

    def plotMag(self, mode = "raw", duration = 15, x=[], y=[]):
        print("Plotting starting. When ready, input and rotate imu on a flat surface.")
        input()
        tstart = time.time()
        if mode != "provided":
            while (tstart + duration > time.time() and self.sensing.robot.running):
                if mode =="raw":
                    self.sensing._readRaw()
                    x.append(self.sensing.raw_mag[0])
                    y.append(self.sensing.raw_mag[1])
                if mode =="norm":
                    self.sensing.getMagHeading()
                    x.append(self.sensing.mag[0])
                    y.append(self.sensing.mag[1])
        plt.scatter(x, y, color='red', marker='o', s=10)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()
        




class Sensing(SettingsManager):
    def __init__(
            self, 
            robot,
            load_from_path=None,
        ):
        super().__init__(ignore_list=[Robot])

        self.robot = robot
        self.imu_calib = ImuCalibration(
            self,
            load_from_path=load_from_path,
        )

        # initalise readings
        self.raw_accel = None
        self.raw_gyro = None
        self.raw_mag = None
        self.raw_timestamp = None
        self._readRaw()

        self.heading_offset = 0

        self.mag_vec = None
        self.mag_angle = None
        self.getMagHeading()


    def getMagHeading(self):
        # normalise the mags
        self._readRaw()
        if self.raw_timestamp is None:
            return None
        ranges = self.imu_calib.mag_max - self.imu_calib.mag_min 
        centres = self.imu_calib.mag_min + ranges / 2
        scalings = np.zeros(3)
        for i, curr in enumerate(XYZ_ARR):
            # compare to everything but itself
            # comparisions = [i for i, x in enumerate(XYZ_ARR) if x != curr]
            # scalings[i] = (ranges[i] / min(ranges[i], ranges[comparisions[0]])) * (ranges[i] / min(ranges[i], ranges[comparisions[1]]))
            scalings[i] = ranges[i] / min(ranges)
        mag_vec = (self.raw_mag - centres) / scalings / min(ranges)
        self.mag = mag_vec
        self.mag_angle = (math.degrees(math.atan2(mag_vec[0], mag_vec[1])) - self.heading_offset +180)%360-180
        return self.mag_angle

    def _readRaw(self):
        # returns RAW values in a dictionary with keys accel gyro mag timestamp
        # in each value is a np array in format xyz, except timestamp
        read = self.robot.imu(IMU_PORT).read()
        if not read.ok:
            print("imu read failed!")
            return False
        readings = {}
        row = ["accel", "gyro", "mag"]
        for type in row:
            vector = np.zeros(3) #implicitly a xyz format okay 
            for i, axis in enumerate(XYZ_ARR):
                vector[i] = getattr(read, f"{type}_{axis}")
            setattr(self, f"raw_{type}", vector.copy())
        self.raw_timestamp = read.timestamp
        return True

    def setHeadingOffset(self, offset):
        # one method is to set the starting heading to heading 0
        # sensing.setHeadingOffset(sensing.getMagHeading())

        # HOWEVER my prefered method is to measure one time before the match and manually set the heading
        # sensing.setHeadingOffset(-34)
        # so when the match starts and we drift / never place properly its all okay
        self.heading_offset = offset

class Plotter: # work in progress
    def __init__(self):
        # plt.ion()
        fig, axes = plt.subplots(nrows=2, ncols=2)
        
        self.windows = {
            "Heading": (0,0),
        }

        # set heading window
        ax = axes[*self.windows["Heading"]]
        ax.set_title("Heading")
        ax.set_aspect('equal', adjustable='box') 
        ax.set_xlim(-1,1)
        ax.set_ylim(-1,1)
        ax.scatter(0,0, s=10, c='b')

        plt.tight_layout()
        plt.show()

class Motion(SettingsManager):
    def __init__(self, robot, sensing, 
                load_from_path,
                turning_kp=None,
                turning_ki=None,
                turning_kd=None,
                move_turning_kp=None,
                move_turning_ki=None,
                move_turning_kd=None,
                ):
        super().__init__(ignore_list=[Robot, Sensing])
        self.robot = robot
        self.sensing = sensing

        self.turning_kp = turning_kp
        self.turning_ki = turning_ki
        self.turning_kd = turning_kd
        self.move_turning_kp = move_turning_kp
        self.move_turning_ki = move_turning_ki
        self.move_turning_kd = move_turning_kd

        self.importData(load_from_path)

    def turnTo(self, target_heading, speed=50, timeout=5, threshold = 5):
        #everythins in degrees
        #target_heading is after offset btw
        start_t = time.time()
        while self.robot.running:
            #turn robot
            reading = self.sensing.getMagHeading()
            if reading == None:
                print("WARNGING: imu failed! using old values")
            error = (target_heading - self.sensing.mag_angle + 180) % 360 - 180
            offset = (error*self.turning_kp)
            self.move(offset, -offset)
            print(f"turnTo: current_heading: {reading} error:{error}, speeds:+-{offset}")

            # check for end condition
            time_length = time.time() - start_t
            if time_length > timeout: break
            if abs(error) < threshold: break


    def moveTo(self, unit, value, target_heading, speed = 80, timeout=8):
        # only unit == "time" implemented. time in seconds.
        # put seconds in value

        #i REALLY recomend u to put target heading manually. 
        # this will make sure that even during transition, if the robot gets hit, 
        # we have a better chance of going where we want to. 
        # like DO NOT not manually put a target heading in the real auton 
        # but for testing, if you want the robot to just go straight, put None

        # for timeout, if the function does not accomplish the goal by the set amount of time, 
        # it quits and moves on. helps when u hit a wall or something.
        if target_heading is None:
            num = 10
            target_heading = 0
            for i in range(num):
                while not robot.running:
                    pass
                target_heading += self.sensing.getMagHeading() * 1/num

        start_t = time.time()
        while self.robot.running:
            # check for end condition
            time_length = time.time() - start_t
            if time_length > timeout: break
            if unit == "time" and time_length > value: break

            # move robot while compensating for heading
            reading = self.sensing.getMagHeading()
            if reading == None:
                print("WARNGING: imu failed! using old values")
            error = (target_heading - self.sensing.mag_angle + 180) % 360 - 180
            offset = (error*self.turning_kp)
            self.move(speed+offset, speed-offset)
            print(f"turnTo: current_heading: {reading} error:{error}, speeds:{speed}+-{offset}")

    def move(self, left, right):
        # left and right is out of 100
        left = min(100, max(-100, left))
        right = min(100, max(-100, right))

        self.robot.thruster(LEFT_THRUST).set_duty(left)
        self.robot.thruster(LEFT_THRUST_SECONDARY).set_duty(left)

        self.robot.thruster(RIGHT_THRUST).set_duty(right)
        self.robot.thruster(RIGHT_THRUST_SECONDARY).set_duty(right)





def testing(robot):
    # create the classes
    sensing = Sensing(robot, load_from_path=f"{TRIDENTXPATH}/imu_calibration.yaml")
    motion = Motion(robot, sensing, load_from_path=f"{TRIDENTXPATH}/motion_tuning.yaml")

    # motion functions
    motion.moveTo(mode="time", value=5, target_heading=90) #also has speed and timeout arguments
    motion.turnTo(target_heading=90) #also has speed, timeout, and threshold arguments

    # tune the constants of the PID in motion_tuning.yaml
    # to go backward just put negative speed



robot.run(
    initialize=initialize,
    autonomous=autonomous,
    driver=testing,
)

print(robot)