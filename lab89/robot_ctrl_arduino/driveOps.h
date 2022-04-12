#include "sharedUtils.h"

double input;

void stunt()
{
  input = tx_tof_f_value; // Front TOF sensor reading (mm)
  if (input > 500)
  {
    drive(FORWARD, 200);
    stunt();
  }
  else
  {
    drive(BACKWARD, 200);
    delay(50);
    drive(STOP, 0);
    delay(200);
    drive(FORWARD, 200);
    delay(100);
    drive(STOP, 0);
  }
}

// May require testing to adjust values to do an actual 360
void spin360()
{
  for (int i = 0; i < 14; i++)
  {
    drive(RIGHT, 100);
    delay(200);
    drive(STOP, 0);
    delay(500);
  }
}