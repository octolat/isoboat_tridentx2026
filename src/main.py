"""A MARLIN robot with tank drive controls, servo control, and sensor reading printouts."""

from ipaddress import collapse_addresses
import time
from marlin import Robot, Thruster, Servo, IMU, RadioBeacon

# defines
XYZ_ARR = ["x", "y", "z"]

# Brain port definitions
LEFT_THRUST = "M8" 
RIGHT_THRUST = "M1"
LEFT_THRUST_SECONDARY = "M7" 
RIGHT_THRUST_SECONDARY = "M6" 

ARM = "M2"

IMU_PORT = "S4"
RADIOBEACON_PORT = "S6"

# settings and values for the auton (is like this so i dont have to global everytihbn in the funcctions)
atn_set = {
    "imu_calib": {
        'x': {'max': 342, 'min': -201, 'count': 912259}, 
        'y': {'max': 558, 'min': 3, 'count': 843805}, 
        'z': {'max': 320, 'min': -238, 'count': 943009}
        }

}

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







def testing(robot):
    if atn_set["imu_calib"] == None or True: 
        atn_set["imu_calib"] = imu_calibrateMag()
        print(f"imu calib results: {atn_set["imu_calib"]}")

    time.sleep(5)

    getMagHeading(atn_set["imu_calib"])


# --- robot functions --- #

# bounds of each axis is needed to calculate yaw with magnometer
def imu_calibrateMag(duration = 5): 
    results = {}
    for i, axis in enumerate(XYZ_ARR):
        print(f"rotate along the {axis} axis. send input when ready.")
        input()
        tstart = time.time()
        result = {"max": None, "min": None, "count": 0}
        while (tstart + duration > time.time()):
            reading = imu_read()["mag"][i]
            if result["max"] is None or result["max"] < reading:
                result["max"] = reading
            if result["min"] is None or result["min"] > reading:
                result["min"] = reading
            result["count"] += 1

        print(result)
        results[axis] = result

    return results

def getMagHeading(calibration):
    mag_vec = imu_read()["mag"]

def imu_read():
    readings = {}
    row = ["accel", "gyro", "mag"]
    for type in row:
        vector = [0]*3 #implicitly a xyz format okay 
        for i, axis in enumerate(XYZ_ARR):
            vector[i] = getattr(robot.imu(IMU_PORT).read(), f"{type}_{axis}")
        readings[type] = vector

    return readings


robot.run(
    initialize=initialize,
    autonomous=autonomous,
    driver=testing,
)