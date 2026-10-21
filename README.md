# connecting to the robot
There will be multiple ports, and occasionally it may say connected even though its on the wrong port (connected to your mouse or something.) make sure the firmware is not unknown.  
![](documentation_media\got_firmware.png)




dev notes:
marlin module code is at `marlin_src` for reference (og was at `~\\.vscode\\extensions\\polymatepteltd.marlin-controller-1.0.3\\dist\\marlin\\` )   
[api reference](https://marlin.polymate.tech/api-reference/)  

todos:
1. firgure out why it occasionally disconnects like that
3. auton api
4. for yaw ->
    - calabrate xyz
    - project xyz vector into the ground plane by checking the accel
    - do normal trigo to find angle (the magnitude litterly dosen matter thats why you can jut projct even though the manitiude dies)
    - profit
    - maybe add gyro
    - maybe add acell
5. for position tracking ->
    - extended kalman filter
    - youll have to measure various reactions of the boat
    - and retune everytime the boat cahnges
    - watch a youtube video
6. for ui
    -  matplotlib dashboard
    - ez plant value tuning
    - motor speed ah battery ah errors ah etc
    - estimated position and heading