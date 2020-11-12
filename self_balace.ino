#include "I2Cdev.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

MPU6050 mpu;

#define enA 10
#define in1 8
#define in2 9
#define enB 5
#define in3 6
#define in4 7
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion vector to get orientation and rotation of the bot
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw,pitch,roll container 


double setpoint= 182; //Setpoint around which to balance or the point where the bot is perpendicular to the surface 
/***************************Pid gains to tune for your bot*****************/
double Kp = 50; //Set this first(Usually this is done first when tuning a PID regulator for a system to reduce the steady state error)
double Kd = 2.2; //Set this secound(Then the differential gain is tuned to decrease the overshoot/oscillations and settling time)
double Ki =180;//Finally set this(And then to completely eliminate the steady state error,we tweak the integral gain)


double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);

  // initialize device
    Serial.println(F("Initializing the I2C devices....."));
    mpu.initialize();

     // verifying the connection
    Serial.println(F("Testing the device connections....."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    
    // can supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688); 

      // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

//Initialising the motor output pins
    pinMode (enB, OUTPUT);
    pinMode (in3, OUTPUT);
    pinMode (in4, OUTPUT);
    pinMode (enA, OUTPUT);
    pinMode (in1, OUTPUT);
    pinMode (in2, OUTPUT);

//Turning off both the motors(default)
    analogWrite(enA,0);
    analogWrite(enB,0);
    digitalWrite(7,LOW);
    digitalWrite(6,LOW);
    digitalWrite(8,LOW);
    digitalWrite(9,LOW);
}

void loop() {
 
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {   Serial.print("sup ");
    Serial.print(input);
    Serial.print("t");
        //no mpu data - performing PID calculations and output to motors     
        pid.Compute();   
        
        //Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(input); Serial.print(" ==>"); Serial.println(output);
               
        if (input>140 && input<240){//If the Bot is falling 
          
        if (output>0) //Falling towards front 
        Forward(); //Then rotating the wheels forward 
        else if (output<0) //Falling towards back
        Reverse(); //Then rotating the wheels backward 
        }
        else //If Bot not falling
        Stop(); //Hold the wheels still
        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

        input = ypr[1] * 180/M_PI + 180;

   }
}

void Forward() //Code to rotate the wheel forward 
{    Serial.print("Forward...");
    analogWrite(enB,output);
   digitalWrite(in3,LOW);
   digitalWrite(in4,HIGH);
    analogWrite(enA,output);
  digitalWrite(in1,LOW);
   digitalWrite(in2,HIGH);
     //Debugging information 
}

void Reverse() //Code to rotate the wheel Backward  
{    Serial.print("Backward...");
   digitalWrite(in4,LOW);
   digitalWrite(in3,HIGH);
    analogWrite(enB,output*-1);
digitalWrite(in2,LOW);
   digitalWrite(in1,HIGH);
    analogWrite(enA,output*-1); 
    
}

void Stop() //Code to stop both the wheels
{
  
   digitalWrite(in4,LOW);
   digitalWrite(in3,LOW);
    analogWrite(enB,0);
    digitalWrite(in2,HIGH);
   digitalWrite(in1,HIGH);
    analogWrite(enA,0);
   
    Serial.print("Stopping...");
}
