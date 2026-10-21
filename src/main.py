"""A MARLIN robot with tank drive controls, servo control, and sensor reading printouts."""

import time
from marlin import Robot, Thruster, Servo, IMU, RadioBeacon

# Brain port definitions (false to disable)
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


def autonomous(robot):
    while robot.running:
        pass


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


robot.run(
    initialize=initialize,
    autonomous=autonomous,
    driver=driver,
)



# --- robot functions --- #

# bounds of each axis is needed to calculate yaw with magnometer
def imu_calibrateMag(time = 5): 
    results = {}
    axises = ["x", "y", "z",]
    for axis in axises:
        pass

