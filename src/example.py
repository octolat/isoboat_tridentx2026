"""A MARLIN robot with tank drive controls, servo control, and sensor reading printouts."""

import time
from marlin import Robot, Thruster, Servo, IMU, RadioBeacon

# Brain port definitions
IMU_PORT = "S1"
RADIOBEACON_PORT = "S6"
LEFT = "M8"
RIGHT = "M1"
ARM = "M2"

robot = Robot()
robot.configure(
    sensors={IMU_PORT: IMU, RADIOBEACON_PORT: RadioBeacon},
    effectors={LEFT: Thruster, RIGHT: Thruster, ARM: Servo},
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

        robot.thruster(LEFT).set_duty(left)
        robot.thruster(RIGHT).set_duty(right)

        # Set Servo speed to 180 deg/s
        robot.servo(ARM).set_speed(180)

        if robot.controller.button("LEFT_TRIGGER").is_down:
            robot.servo(ARM).angle(120)
        else:
            robot.servo(ARM).angle(0)

        # print IMU readings
        imu_reading = robot.imu(IMU_PORT).read()
        if imu_reading.ok:
            print(imu_reading)

        # print RadioBeacon readings
        radiobeacon_reading = robot.radiobeacon(RADIOBEACON_PORT).read()
        if radiobeacon_reading.ok:
            print(radiobeacon_reading)


robot.run(
    initialize=initialize,
    autonomous=autonomous,
    driver=driver,
)
