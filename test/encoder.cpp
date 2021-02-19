#include <mbed.h>

#define WHEEL_RADIUS 0.09
#define GEAR_REDUCTION_RATION 90
#define BASIC_PULSE_NUMBER 17
#define PULSES_PER_REVOLUTION GEAR_REDUCTION_RATION*BASIC_PULSE_NUMBER

Serial pc(PA_9, PA_10);

DigitalOut M1_INA(PB_4);
DigitalOut M1_INB(PB_3);
DigitalOut M1_EN(PB_15);
PwmOut M1_PWM(PA_7);
AnalogIn M1_CS(PB_1);
InterruptIn M1_C1(PB_13);
DigitalIn M1_C2(PB_12);

int m1_pulses = 0;
int m1_revolutions = 0;

void callback(){
  if (M1_C2.read()) m1_pulses++;
  else m1_pulses--;

  if (m1_pulses >= PULSES_PER_REVOLUTION) {
    m1_pulses = 0;
    m1_revolutions ++;
  }
  else if (m1_pulses <= -PULSES_PER_REVOLUTION) {
    m1_pulses = 0;
    m1_revolutions--;
  }

}

int main() {

  M1_C1.rise(&callback);

  M1_INA = 1;
  M1_INB = 0;
  M1_EN = 1;
  M1_PWM = 1;

  while (1) {
    pc.printf("%i \n", m1_revolutions);
    wait_ms(100);
  }
}

