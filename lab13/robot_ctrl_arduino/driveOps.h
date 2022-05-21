#include "sharedUtils.h"

void stunt(int* i)
{
  if (tx_tof_f_value > 2000)
  {
    drive(FORWARD, 120);
    (*i)++;
    Serial.print("Driving forward as front TOF: ");
    Serial.println(tx_tof_f_value);
  }
  else
  {
//    Serial.print(tx_tof_f_value);
//    Serial.println(" < 500; Jump!");
//    drive(BACKWARD, 255);
//    delay(1000);
//    drive(ASTOP, 0);
//    delay(500);
//    drive(BACKWARD, 200);
//    delay(1000);
    drive(ASTOP, 0);
    stunt_running = false;
    *i = 0;
  }
//  for (int i = 0; i < 10; i++) {
//    drive(RIGHT, 200);
//    delay(50);
//  }
//  drive(ASTOP, 0);
//  stunt_running = false;
}

// May require testing to adjust values to do an actual 360
void spin360(int* i, int initSpeed)
{
  float angV = myICM.gyrZ();
  Serial.println(angV);

  // PID
  float kp = 1.0;
  float err = angV - 20;
  int speed = (int)(initSpeed - kp * err);

  if (*i < 18)
  {
    drive(RIGHT, speed);
    delay(200);
    drive(STOP, 0);
    delay(300);
    (*i)++;
  }
  else
  {
    drive(STOP, 0);
    spin_running = false;
    *i = 0;
  }
}
