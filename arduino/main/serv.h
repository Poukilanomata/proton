#include <ESP32Servo.h>

class Serv {
  public:
    Serv(Servo& servoObj, int m_ang[2]); // constructor with a Servo object parameter and an integer array of size 2
    void moveTo(int angle);
    void checkup();
  private:
    Servo* _servo; // pointer to a Servo object
    int _m_ang[2];
};

Serv::Serv(Servo& servoObj, int m_ang[2]) {
  _servo = &servoObj; // store a pointer to the passed Servo object
  _m_ang[0] = m_ang[0];
  _m_ang[1] = m_ang[1];
}

// Visual checkup
void Serv::checkup() {
  for (int i = _m_ang[0]; i <= _m_ang[1]; i++) {
    this->moveTo(i);
    delay(10);
  }

  this->moveTo(90);
}

// Move the servo making sure it's not passing the limit
void Serv::moveTo(int angle) {
  if (_m_ang[0] <= angle && angle <= _m_ang[1]) {
    _servo->write(angle);
  } else {
    if (angle - 90.0 < 0) {
      _servo->write(_m_ang[0]);
    } else {
      _servo->write(_m_ang[1]);
    }
  }
}
