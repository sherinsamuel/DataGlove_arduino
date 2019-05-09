// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <SoftwareSerial.h>

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu1;
MPU6050 mpu2(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL
// Define the data transmit/receive pins in Arduino
#define TxD 5
#define RxD 6

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus1; 
uint8_t devStatus2;     // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int flexSensorPin = A0;
int flexSensorReading;
SoftwareSerial mySerial(RxD, TxD); // RX, TX for Bluetooth


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(57600);
    mySerial.begin(57600); // For Bluetooth
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    mySerial.println(F("Initializing I2C devices..."));
    mpu1.initialize();
    mpu2.initialize();
    
    // verify connection
    mySerial.println(F("Testing device connections..."));
    mySerial.println(mpu1.testConnection() ? F("MPU6050 1 connection successful") : F("MPU6050 1 connection failed"));
    mySerial.println(mpu2.testConnection() ? F("MPU6050 2 connection successful") : F("MPU6050 2 connection failed"));

    // wait for ready
    mySerial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (mySerial.available() && mySerial.read()); // empty buffer
    while (!mySerial.available());                 // wait for data
    while (mySerial.available() && mySerial.read()); // empty buffer again

    // load and configure the DMP
    mySerial.println(F("Initializing DMP..."));
    devStatus1 = mpu1.dmpInitialize();
    devStatus2 = mpu2.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu1.setXGyroOffset(220);
    mpu1.setYGyroOffset(76);
    mpu1.setZGyroOffset(-85);
    mpu1.setZAccelOffset(1788); 
    mpu2.setXGyroOffset(220);
    mpu2.setYGyroOffset(76);
    mpu2.setZGyroOffset(-85);
    mpu2.setZAccelOffset(1788);// 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus1 == 0 && devStatus2 == 0) {
        // turn on the DMP, now that it's ready
        mySerial.println(F("Enabling DMP..."));
        mpu1.setDMPEnabled(true);
        mpu2.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //mySerial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu1.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //mySerial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu1.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
       mySerial.print(F("DMP Initialization failed (code "));
        mySerial.print(devStatus1);
        mySerial.print(devStatus2);
        mySerial.println(F(")"));
    }

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
//  currentMillis = millis();
  GetMpuValue(mpu1,1);
  GetMpuValue(mpu2,2);
 }

void GetMpuValue(const MPU6050 mpu, int flag){
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else{
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            if(flag == 1)
            {
              mySerial.print("<");
              mySerial.print(ypr[0] * 180/M_PI);
              mySerial.print(",");
              mySerial.print(ypr[1] * 180/M_PI);
              mySerial.print(",");
              mySerial.print(ypr[2] * 180/M_PI);
              mySerial.print(",");
            }
            else
            {
              mySerial.print(ypr[0] * 180/M_PI);
              mySerial.print(",");
              mySerial.print(ypr[1] * 180/M_PI);
              mySerial.print(",");
              mySerial.print(ypr[2] * 180/M_PI);
              mySerial.print(",");
              mySerial.print(analogRead(flexSensorPin));
              mySerial.print(">");
              mySerial.println("");
            }
        #endif
    }

}

