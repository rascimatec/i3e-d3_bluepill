#include <mbed.h>
#include "ros.h"
#include "std_msgs/Float32.h"

// Utilizar um .yaml
#define PI 3.14159265359
#define TWO_PI 2*PI
#define WHEEL_RADIUS 0.09
#define GEAR_REDUCTION_RATIO 90
#define BASIC_PULSE_NUMBER 17
#define PULSES_PER_REVOLUTION GEAR_REDUCTION_RATIO*BASIC_PULSE_NUMBER
#define PULSE_DISTANCE TWO_PI/PULSES_PER_REVOLUTION

DigitalOut M1_INA(PB_4);
DigitalOut M1_INB(PB_3);
DigitalOut M1_EN(PB_15);
PwmOut M1_PWM(PA_7);
InterruptIn M1_C1(PB_13);
DigitalIn M1_C2(PB_12);
AnalogIn M1_CS(PB_1);

DigitalOut M2_INA(PB_9);
DigitalOut M2_INB(PB_8);
DigitalOut M2_EN(PB_5);
PwmOut M2_PWM(PA_11);
InterruptIn M2_C1(PB_14);
DigitalIn M2_C2(PA_3);

int m1_pulses = 0;
bool m1_reverse = false;

ros::NodeHandle nh;

std_msgs::Float32 float_msg;
ros::Publisher publisher("publisher", &float_msg);


void callback(){

  m1_reverse = M1_C2.read();
  if (m1_reverse) m1_pulses++;
  else m1_pulses--;

}

int main() {

  nh.initNode();
  nh.advertise(publisher);

  M1_C1.rise(&callback);
  
  M1_INA = 1;
  M1_INB = 0;
  M1_EN = 1;
  M1_PWM = 1;

  ros::Time current_time = nh.now(); 
  ros::Time last_time = current_time;

  float delta_ang;
  float dt;

  while(1) {
    current_time = nh.now();
    dt = current_time.toSec() - last_time.toSec();
    delta_ang = m1_pulses*PULSE_DISTANCE;
    m1_pulses = 0;
    float_msg.data = delta_ang/dt;
    publisher.publish(&float_msg);
    last_time = current_time;
    nh.spinOnce();
    wait_ms(50);
  }
}
