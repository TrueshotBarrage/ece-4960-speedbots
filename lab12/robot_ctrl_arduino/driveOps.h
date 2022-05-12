#include "sharedUtils.h"

void stunt()
{
  if (tx_tof_f_value > 500)
  {
//    drive(FORWARD, 100);
    Serial.print("Driving forward as front TOF: ");
    Serial.println(tx_tof_f_value);
  }
  else
  {
    Serial.println("Jump!");
//    drive(BACKWARD, 100);
//    delay(50);
//    drive(STOP, 0);
//    delay(200);
//    drive(FORWARD, 100);
//    delay(100);
//    drive(STOP, 0);
    stunt_running = false;
  }
}

// May require testing to adjust values to do an actual 360
void spin360(int* i)
{
  float angV = myICM.gyrZ();

  // PID
  float kp = 1.0;
  float err = angV - 15;
  int speed = (int)(108 - kp * err);
  
  if (*i < 14)
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
