# connecting to the robot
There will be multiple ports, and occasionally it may say connected even though its on the wrong port (connected to your mouse or something.) make sure the firmware is not unknown.  
![](documentation_media\got_firmware.png)


# running the script
1. open the `tridentx` folder in vsc. It should have the project.marlin file inside. 
2. you need `numpy` and `pyyaml` and `matplotlib` installed on your host python enviroment (sorry). you can do this by `py -m pip install numpy` or `python -m pip install numpy` in your terminal.
3. as well as for pyyaml and matplotlib

# quick api hints
!!!Change the value of `TRIDENTXPATH` to your own path of the tridentx folder!!!
the `motion_tuning.yaml` and `imu_calibration.yaml` files should be inside.  
the `Motion` and `Sensing` classes rely on those config files respectively.  
look at testing() for how to use the functions
``

dev notes:
marlin module code is at `marlin_src` for reference (og was at `~\\.vscode\\extensions\\polymatepteltd.marlin-controller-1.0.3\\dist\\marlin\\` )   
[api reference](https://marlin.polymate.tech/api-reference/)  

marlin quirks
1. any loop you do HAVE to call robot.running somewhere. like while (condition) and robot.running. it includes some wait delay thing that makes all the sensors and whatnot work. without it, the sensors/motors might not return at all.
2. you can do closing actions AFTER the robot.run (which is blocking). robot.run will release when e-stop is pressed. so you could export settings or what not lah

todos:
1. firgure out why it occasionally disconnects like that
3. auton api
4. for yaw ->
    - compensate the magnometer for tilt
        - but highkey if the boat keeps moving like that idk how to find the down vector
    - complementary filter to include accel or velocity for the heading? kalman?
    - profit
5. for position tracking ->
    - extended kalman filter????? pain and death
    - youll have to measure various reactions of the boat
    - and retune everytime the boat cahnges
    - watch a youtube video
    - why is the math so hard
6. for ui
    -  matplotlib dashboard (interactive mode)
    - ez plant value tuning
    - motor speed ah battery ah errors ah etc
    - estimated position and heading