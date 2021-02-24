#include <mbed.h>
#include <Servo.h>

Servo servo(PB_6);

int main() {

    // enable servo and set position to -90 degrees
    servo.attach(SERVO_POS_MIN);

    while(1) {
      // Sweep across whole servo range from -90 degrees to +90 degrees
      for (int i = SERVO_POS_MIN; i <= SERVO_POS_MAX; i++)
      {
        servo.write(i);
        wait_us(10000);
      }

      // Sweep back from 90 degrees to -90 degrees
      for (int i = SERVO_POS_MAX; i >= SERVO_POS_MIN; i--)
      {
        servo.write(i);
        wait_us(10000);
      }

      wait_us(1000000);
    }
  }