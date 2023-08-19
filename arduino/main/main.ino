// Quick note : 0 -> left ;  1 -> right (always in this order)

#include "BluetoothSerial.h"
#include "serv.h"
#include "gyro.h"
#include "controller.h"
#include "pilot.h"
#include <ESP32Servo.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


BluetoothSerial SerialBT;

// By default
bool enable_auto = false;

float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];       // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float xyz[3];
float yprEMA[3] = {0, 0, 0};        //YPR corrected to avoid noises
float ypr_tmp[3];
float xyz_tmp[3];
int servo_angle[2] = {80, 80};
float correction[2] = {0, 0};
float ph[10];
float rh[10]; 

#define SERV_L 34 // Direct pins for servos input (G34, G35)
#define SERV_R 35 

#define ENABLE_AUTO_CH 33 // Check on channel G33

int m_ang[2] = {20, 160}; // max range for servos

int live_angle[2]= {80, 80}; // By default neutral

// Declaring servos
Servo servo_l;
Servo servo_r;

// Add some new functions 
Serv serv_l(servo_l, m_ang);
Serv serv_r(servo_r, m_ang);

void PRINTEUH(String a) {
  uint8_t buf[a.length()];
  memcpy(buf,a.c_str(),a.length());
  SerialBT.write(buf,a.length());
}

void sendData(float* xyz, float* ypr, int* servo_angle) {
  // Return flying data through bluetooth
  String tmp = String(millis());

  // Acceleration dat a(m.s^-2)
  String accel_data = tmp + ";A;" + String(xyz[0]) + ";" + String(xyz[1]) + ";" + String(xyz[1]);
  PRINTEUH(accel_data);
  SerialBT.println();

  // Gyro (deg)  
  String gyro_data = tmp + ";I;" + String(ypr[0]) + ";" + String(ypr[1]) + ";" + String(ypr[2]);
  PRINTEUH(gyro_data);
  SerialBT.println();

  String G_data = tmp + ";G;" + String(servo_angle[0]);
  PRINTEUH(G_data);
  SerialBT.println();

  String D_data = tmp + ";D;" + String(servo_angle[1]);
  PRINTEUH(D_data);
  SerialBT.println();
}

void setup(){
  Serial.begin(115200); // Enable feedback
  SerialBT.begin("ESP32");

  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  // Setting servos to port 25 and 26
	servo_l.setPeriodHertz(50);    // standard 50 hz servo
	servo_l.attach(25, 500, 2400); // attaches the servo on pin 18 to the servo object
  serv_l.checkup();

  
	servo_r.setPeriodHertz(50);    // standard 50 hz servo
	servo_r.attach(26, 500, 2400); // attaches the servo on pin 18 to the servo object
  serv_r.checkup();


  // Setup intput
  pinMode(SERV_L, INPUT);
  pinMode(SERV_R, INPUT);
  pinMode(ENABLE_AUTO_CH, INPUT);

  SerialBT.print("Starting ...");
  SerialBT.print("\n");
  initMPU();
}

void loop() {
  //getYPRdata(ypr);


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

    // Patch the YPR data
    ypr_tmp[0] = -ypr[0]; 
    ypr_tmp[1] = -ypr[2];
    ypr_tmp[2] = ypr[1];

    ypr[0] = ypr_tmp[0];
    ypr[1] = ypr_tmp[1];
    ypr[2] = ypr_tmp[2];

    xyz[0] = aaReal.x/16384.0;
    xyz[1] = aaReal.x/16384.0;
    xyz[2] = aaReal.x/16384.0;

    xyz_tmp[0] = -xyz[0];
    xyz_tmp[1] = -xyz[1];
    xyz_tmp[2] = xyz[2];

    xyz[0] = xyz_tmp[0];
    xyz[1] = xyz_tmp[1];
    xyz[2] = xyz_tmp[2];
 
    displayAaReal(xyz);
    
    //displayH(ph, rh); 
    updatePH(ypr, ph);
    updatePH(ypr, rh);
    //EMA(yprEMA, ypr);
  }
  
  // Get 2 directs angles 
  live_angle[0] = readChannel(SERV_L, 0, 160, 20);
  live_angle[1] = readChannel(SERV_R, 0, 160, 20);
  

  enable_auto = getBoolChannel(ENABLE_AUTO_CH, 0, 100, 0, 50);
  
  if(enable_auto) { // enable autopilot
    // Cruse - takeoff - land  
    displayYPR(ypr);

    int correction_p = getPitchCorrection(ypr, ph);
    int correction_r = getRollCorrection(ypr, rh);

    if(-40 < correction_p + correction[0] && 40 > correction_p + correction[0]) {
      correction[0] = correction_p + correction[0];
    } else {
      correction[0] = signOf(correction_p + correction[0]) * 40;
    }
    if( -30 < correction_r + correction[1] && 30 > correction_r + correction[1]) {
      correction[1] = correction_r + correction[1];
    } else {
      correction[1] = signOf(correction_p + correction[1]) * 30;
    }

    servo_angle[0] = correct_l(correction);
    servo_angle[1] = correct_r(correction);

    Serial.print(servo_angle[0]);
    Serial.print(servo_angle[1]);
    
    serv_l.moveTo(servo_angle[0]);
    serv_r.moveTo(servo_angle[1]);
    
  } else { // Take control by hands 
    
    Serial.println(live_angle[0]);
    Serial.println(live_angle[1]);

    serv_l.moveTo(live_angle[0] + 10); 
    serv_r.moveTo(180 - live_angle[1]); 

    servo_angle[0] = live_angle[0];
    servo_angle[1] = live_angle[1];

    //serv_l.moveTo(90); 
    //serv_r.moveTo(90);
  }

  sendData(xyz, ypr, servo_angle);
}

/*
Servo servo_r;
Servo servo_l;

float max_angle[2] = {20, 160};
float max_pitch_control = 0.7;

float servo_angle[2] = {90, 90}; // left - right

bool take_off = false;
bool land = false;

SimpleKalmanFilter anglefilter(1, 0.25, 0);
//SimpleKalmanFilter accfilter(50, 0.5, 0);

//======================================================================================


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

// Setup
void setup() {
  // Setup MPU6050
  initMPU();

    servo_r.attach(25);
    servo_l.attach(26);

    check_servos();
}

// Utils 
void check_servos() {
  
  servo_angle[0] = max_angle[0];
  servo_angle[1] = max_angle[0];
  update_servos(true);

  delay(500);
  
  for (float i = max_angle[0]; i <= max_angle[1]; i++) {
    servo_angle[0] = i;
    servo_angle[1] = i;
    update_servos(true);
    
    delay(20);
  }

  servo_angle[0] = 90.0;
  servo_angle[1] = 90.0;
  update_servos(true);
}

void auto_pitch(float to_angle) {
  
  // Max variation from 90
  float max_var = max_angle[1] - 90.0;
  // Max percentage allowed to pitch control

  float delta_p = ypr[1] - to_angle;
  float tau = 20;
  
  //Serial.print("  |  ");
  //Serial.println(90 + p_angle_control + r_angle_control);

  float p_angle_control;
  
  if(delta_p >= 0) {
    p_angle_control = max_pitch_control * max_var * (1 - exp(-delta_p/tau));     
  } else {
    p_angle_control = -max_pitch_control * max_var * (1 - exp(delta_p/tau)); 
  }

  
  //Serial.print(90 + p_angle_control - r_angle_control);
  //Serial.print("  |  ");
  //Serial.println(90 + p_angle_control + r_angle_control);

  servo_angle[0] = 90 + p_angle_control;
  servo_angle[1] = 90 + p_angle_control;
}

void auto_roll(float to_angle) {

  float max_var = max_angle[1] - 90.0;  

  float delta_p = ypr[2] - to_angle;
  float tau = 20;

  float r_angle_control;
  
  if(delta_p >= 0) {
    r_angle_control = (1 - max_pitch_control) * max_var * (1 - exp(-delta_p/tau));     
  } else {
    r_angle_control = -(1 - max_pitch_control) * max_var * (1 - exp(delta_p/tau)); 
  }
  
  //Serial.print(90 + p_angle_control - r_angle_control);
  //Serial.print("  |  ");
  //Serial.println(90 + p_angle_control + r_angle_control);

  servo_angle[0] += r_angle_control;
  servo_angle[1] -= r_angle_control;
}

int get_real_angle(float angle) {
  float real_angle;
  if(max_angle[0] <= angle <= max_angle[1]) {
    real_angle = angle;
  }

  else {
    if(angle - 90.0 < 0) {
      real_angle = max_angle[0];      
    } else {
      real_angle = max_angle[1];
    }
  }

  int a = real_angle;
  return a; 
}

void update_servos(bool display) {   
  int left = get_real_angle(servo_angle[0]);
  int right = get_real_angle(servo_angle[1]);

  if(display) {
    Serial.print(servo_angle[0]);
    Serial.print("left, ");
    Serial.print(servo_angle[1]);
    Serial.print("right\n");
  }
  

  servo_l.write(left);
  servo_r.write(right);
}

// Main
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
  ypr = readYPR();

  delay(20);
}*/