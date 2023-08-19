#include <iterator>
#include "HardwareSerial.h"
// IMPORTANT NOTE : 
// "If it works don't touch it."

#include "MPU6050_6Axis_MotionApps20.h"
#include "BluetoothSerial.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)


bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector


// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

void initMPU(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  /*// wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again*/

  // load and configure the DMP
  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  //mpu.setXGyroOffset(121);
  //mpu.setYGyroOffset(-50);
  //mpu.setZGyroOffset(-15);
  //mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(30);
      mpu.CalibrateGyro(30);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT); 
}

void displayYPR(float* ypr) {
  Serial.print("ypr\t");
  Serial.print(ypr[0]);
  Serial.print(" deg \t");
  Serial.print(ypr[1]);
  Serial.print(" deg \t");
  Serial.print(ypr[2]);
  Serial.println(" deg");
}

void displayAaReal(float* xyz) {
  Serial.print("xyz\t");
  Serial.print(xyz[0]);
  Serial.print("m.s^-2\t");
  Serial.print(xyz[1]);
  Serial.print("m.s^-2\t");
  Serial.print(xyz[2]);
  Serial.println("m.s^-2");
}

// Test after to get better code 

void getYPRdata(float* ypr) {
  float ypr_tmp[3];
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Get YPR (yam pitch roll) in rad
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    // Convert ypr from rad to deg
    ypr[0] = ypr[0] * 180/PI;
    ypr[1] = ypr[1] * 180/PI;
    ypr[2] = ypr[2] * 180/PI;

    // Notice that the MPU6050 is propely axed : yaw = -pitch, pitch = -roll, roll = yaw
    // Patch the YPR data
    ypr_tmp[0] = -ypr[1];
    ypr_tmp[1] = -ypr[2];
    ypr_tmp[2] = ypr[0];

    ypr[0] = ypr_tmp[0];
    ypr[1] = ypr_tmp[1];
    ypr[2] = ypr_tmp[2];
  } else {
    Serial.println("Error while reading ypr.");
  }
}

void EMA(float* yprEMA, float* ypr) {
  float alpha = 0.998;

  yprEMA[0] = alpha*ypr[0] + (1 - alpha)*yprEMA[0];
  yprEMA[1] = alpha*ypr[1] + (1 - alpha)*yprEMA[1];
  yprEMA[2] = alpha*ypr[2] + (1 - alpha)*yprEMA[2];
}

void displayH(float* ph, float* rh) {
  Serial.println(sizeof(ph));
  Serial.println(sizeof(rh));
  
  for(int i = 0; i < sizeof(ph); i++) {
    Serial.print(ph[i]);
    Serial.print("  ");
  }

  Serial.println();

  for(int j = 0; j < sizeof(rh); j++) {
    Serial.print(rh[j]);
    Serial.print("  ");
  }

  Serial.println();
}

void updatePH(float* ypr, float* ph) {
  for(int i = 0; i < sizeof(ph) - 1; i++) {
    ph[i] = ph[i + 1]; // Moving the array to the left
  }

  ph[sizeof(ph) - 1] = ypr[1];
}

void updateRH(float* ypr, float* rh) {
  for(int i = 0; i < sizeof(rh) - 1; i++) {
    rh[i] = rh[i + 1]; // Moving the array to the left
  }

  rh[sizeof(rh) - 1] = ypr[2];
}