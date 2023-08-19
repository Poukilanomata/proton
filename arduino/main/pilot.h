#include "HardwareSerial.h"
// Note that YPR should already be converted to the "true" data considering the plane configuration

float target_p = 20.0;
float target_r = 0.0;

float offset_l = 90.0;
float offset_r = 80.0;

int signOf(float a) {
  if (a < 0) {
    return -1;
  } else if (a > 0) {
    return 1;
  }

  return 0;
} 

/*
// Pilot 1.0
int correct_l(float* ypr) {
  //int correction = 60 * (exp((ypr[1] - target_p + ypr[2] - target_r)/20) - 1);
  int correction_p = signOf(ypr[1] - target_p) * 20.0 * min((exp(abs(ypr[1] - target_p)/40) - 1), 25.0f);
  int correction_r = -signOf(ypr[2] - target_r) * 25.0 * min((exp(abs(ypr[2] - target_r)/30) - 1), 35.0f);

  int correction = correction_p + correction_r;

  //Serial.println(offset_l + correction_p + correction_r);
  //return offset_l + correction_p + correction_r;

  
  return 180 - (offset_l + correction);
  //return 90;
}

int correct_r(float* ypr) {
  //int correction = 60 * (exp((ypr[1] - target_p + ypr[2] - target_r)/20) - 1);
  
  int correction_p = signOf(ypr[1] - target_p) * 20.0 * min((exp(abs(ypr[1] - target_p)/40) - 1), 25.0f);
  int correction_r = signOf(ypr[2] - target_r) * 25.0 * min((exp(abs(ypr[2] - target_r)/30) - 1), 35.0f);

  int correction = correction_p + correction_r;

  //Serial.println(offset_l + correction_p + correction_r);
  //return 180 - offset_l + correction_p + correction_r;
 
  return offset_r + correction;
}*/

// Pilot 2.0
float derivation(float* hist) {
  return (hist[sizeof(hist) - 1] - hist[0])/sizeof(hist);
}

int getPitchCorrection(float* ypr, float* ph) {
  float theorie_p = ypr[1] + derivation(ph) * 10;  
  int correction_p = 7 * signOf(theorie_p - target_p) * pow((abs(theorie_p - target_p)/30), 2.3) + 3 * signOf(ypr[1] - target_p) * pow((abs(theorie_p - target_p)/30), 2.3);
  Serial.print("Correction pitch ");
  Serial.println(correction_p);

  return correction_p;
}

int getRollCorrection(float* ypr, float* rh) {
  float theorie_r = ypr[2] + derivation(rh) * 10;
  float correction_r = 7 * signOf(theorie_r - target_r) * pow((abs(theorie_r - target_r)/20), 2.3) + 3 * signOf(ypr[2] - target_r) * pow((abs(theorie_r - target_r)/20), 2.3);

  Serial.print("Correction roll ");
  Serial.println(correction_r);

  return correction_r;
}

int correct_l(float* correction) {
  int correct = correction[0] + correction[1];

  return 180 - (offset_l + correct);
}

int correct_r(float* correction) {
  int correct = correction[0] + correction[1];

  return offset_r + correct;
}

