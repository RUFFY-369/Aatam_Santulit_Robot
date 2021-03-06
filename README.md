# SelfBalancingRobot

A two wheeled self-balancing robot based on Arduino atmega 2560.The dynamics of the bot are similar to an inverted pendulum on which a certain torque is applied along pivot to keep it balanced.

### Programming
A PID regulator is used to stabilize this system and minimize the steady state error.

### Electronics
A 6 DOF MPU6050 IMU is used to keep track of the various motion related parameters of the bot like orientation, acceleration, etc.. It is mounted on a 400 point breadboard.

200 rpm BO motors are used at the base with the wheels

The L298N Motor Driver Module is to manipulate the DC motors.

The connection of the whole circuitry is done using jumper wires.

The circuitry is powered by a Duracell 9v alkaline battery(got it cheap✨ but wasn't able to operate the robot for a long time which caused some problems with gains tuning)
(Lipo or Li-ion battery wasn't available)

### Fabrication
To manage the cost of the project I utilised the waste hardboard, cardboard, and some pens✨

The robot is built on three layers of hardboard/cardboard which are spaced 65mm (approx., due to bending of the cardboard) apart with cylindrical part of the pen's body.

The layers are a rectangle cutout of size 160X80 mm.

The bottom layer has slits to fit the motors. The mid layer has the microcontroller, breadboard and an imu. And the top layer has the battery and a motor driver module.

![](https://github.com/RUFFY-369/Aatam_Santulit_Robot/blob/master/Self_balancing_robot_img.jpg)

#### Lets see the robot in action through a GIF
![](https://github.com/RUFFY-369/Aatam_Santulit_Robot/blob/master/Self_balancing_robot.gif)

##### You can also checkout the full video available in the repo.
