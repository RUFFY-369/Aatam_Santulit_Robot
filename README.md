# SelfBalancingBot

A two wheel balancing robot based on Arduino atmega 2560.The dynamics of the bot is similar to an inverted pendulum on which a certain torque is applied along pivot to keep it balanced.

### Programming
A PID regulator is used to stabilise this system and minimise the steady state error.

### Electronics
A 6 DOF MPU6050 IMU is used to keep track of the various motion related parameters of the bot like orientation, acceleration, etc.. It is mounted on a 400 point breadboard.

200 rpm BO motors are used at the base with the wheels

The L298N Motor Driver Module is to manipulate the DC motors.

The connection of the whole circuitry is done using jumper wires.

The circuitry is powered by a Duracell 9v alkaline battery(got it cheap✨)

### Fabrication
To manage the cost of the project I utilised the waste hardboard, cardboard, and some pens✨

The robot is built on three layers of hardboard/cardboard which are spaced 65mm (approx., due to bending of the cardboard) apart with cylindrical part of the pen's body.

The bottom layer has slits to fit the motors. The mid layer has the microcontroller, breadboard and an imu. And the top layer has the battery and a motor driver module.
