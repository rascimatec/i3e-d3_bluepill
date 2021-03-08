#include <mbed.h>
#include "ros.h"
#include "geometry_msgs/Twist.h"

// Utilizar um .yaml
#define PI 3.14159265359
#define TWO_PI 2*PI
#define WHEEL_RADIUS 0.09
#define WHEEL_SEPARATION 0.3
#define GEAR_REDUCTION_RATIO 90
#define BASIC_PULSE_NUMBER 17
#define PULSES_PER_REVOLUTION GEAR_REDUCTION_RATIO*BASIC_PULSE_NUMBER
#define PULSE_DISTANCE TWO_PI/(PULSES_PER_REVOLUTION)
#define MAX_LINEAR_SPEED 1.5
#define MAX_ANGULAR_SPEED MAX_LINEAR_SPEED/WHEEL_RADIUS
#define MAX_LINEAR_ACCEL 2.0
#define MAX_ANGULAR_ACCEL MAX_LINEAR_ACCEL/WHEEL_RADIUS 

void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel);
void ml_encoder_callback();
void mr_encoder_callback();
void calculate_speed(double delta_time);
void limit_differential_speed(double &diff_speed_left, double &diff_speed_right);
void set_motors(double wl, double wr);
void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel);
geometry_msgs::Twist get_twist_vel(double diff_speed_left, double diff_speed_right);


AnalogIn ML_CS(PA_0);
DigitalOut ML_EN(PB_3);
DigitalOut ML_INA(PB_4);
DigitalOut ML_INB(PB_5);
InterruptIn ML_C1(PB_8);
DigitalIn ML_C2(PB_9);
PwmOut ML_PWM(PB_14);

AnalogIn MR_CS(PA_1);
DigitalOut MR_EN(PA_11);
DigitalOut MR_INA(PA_12);
DigitalOut MR_INB(PA_15);
InterruptIn MR_C1(PB_10);
DigitalIn MR_C2(PB_11);
PwmOut MR_PWM(PB_15);

int ml_pulses = 0;
bool ml_reverse = false;
int mr_pulses = 0;
bool mr_reverse = false;

double vel_ang, vel_lin;
double wl_last_speed=0, wr_last_speed=0;
double wl_desired_speed, wr_desired_speed;
double wl_current_speed, wr_current_speed;
double wl_error, wr_error;

double kp, ki, kd;

ros::Time current_time; 
ros::Time last_time;
double dt;

ros::NodeHandle nh;

geometry_msgs::Twist twist_odom_msg;
ros::Publisher twist_odom_pub("twist_odom", &twist_odom_msg);

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_callback);

void ml_encoder_callback(){
  ml_reverse = ML_C2.read();
  if (ml_reverse) ml_pulses++;
  else ml_pulses--;
}

void mr_encoder_callback(){
  mr_reverse = MR_C2.read();
  if (mr_reverse) mr_pulses++;
  else mr_pulses--;
}

void calculate_speed(double delta_time) {
  double delta_ang;

  delta_ang = ml_pulses*PULSE_DISTANCE;
  ml_pulses = 0;
  wl_current_speed = WHEEL_RADIUS*delta_ang/delta_time;
  
  delta_ang = mr_pulses*PULSE_DISTANCE;
  mr_pulses = 0;
  wr_current_speed = WHEEL_RADIUS*delta_ang/delta_time;
}

void limit_differential_speed(double &diff_speed_left, double &diff_speed_right) {

  double wl_accel = diff_speed_left - wl_last_speed;
  double wr_accel = diff_speed_right - wr_last_speed;

  double large_accel = ( std::max(std::abs(wl_accel), std::abs(wr_accel)) ) / dt;
  if (large_accel > MAX_ANGULAR_ACCEL) {
    diff_speed_left = wl_last_speed + MAX_ANGULAR_ACCEL * (large_accel/wl_accel);
    diff_speed_right = wr_last_speed + MAX_ANGULAR_ACCEL * (large_accel/wr_accel);
  }

  double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));
  if (large_speed > MAX_ANGULAR_SPEED) {
    diff_speed_left *= MAX_ANGULAR_SPEED / large_speed;
    diff_speed_left *= MAX_ANGULAR_SPEED / large_speed;
  }

  wl_last_speed = diff_speed_left;
  wr_last_speed = diff_speed_right;
}

void set_motors(double wl, double wr) {

  if (wl > 0) { ML_INA  = 0; ML_INB = 1; }
  else { ML_INA = 1; ML_INB = 0;}
  ML_PWM = 1;

  if (wr > 0) { MR_INA  = 0; MR_INB = 1; }
  else { MR_INA = 1; MR_INB = 0;}
  MR_PWM = 1;  

}

void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel){
  vel_lin = cmd_vel.linear.x;
  vel_ang = cmd_vel.angular.z;
  wr_desired_speed = (vel_lin/WHEEL_RADIUS) + ((vel_ang*WHEEL_SEPARATION)/(2.0*WHEEL_RADIUS));
  wl_desired_speed = (vel_lin/WHEEL_RADIUS) - ((vel_ang*WHEEL_SEPARATION)/(2.0*WHEEL_RADIUS));
  limit_differential_speed(wl_desired_speed, wr_desired_speed);
  set_motors(wl_desired_speed, wr_desired_speed);
}

geometry_msgs::Twist get_twist_vel(double diff_speed_left, double diff_speed_right) {
  geometry_msgs::Twist twist;
  twist.linear.x = WHEEL_RADIUS*(diff_speed_right - diff_speed_left) / 2;
  twist.angular.z = WHEEL_RADIUS*(diff_speed_right - diff_speed_left) / WHEEL_SEPARATION;
  return twist;
}

int main() {

  nh.initNode();
  nh.advertise(twist_odom_pub);
  nh.subscribe(cmd_vel_sub);

  ML_C1.rise(&ml_encoder_callback);
  MR_C1.rise(&mr_encoder_callback);
  
  ML_EN = 1;
  MR_EN = 1;

  last_time = nh.now();

  while(1) {
    current_time = nh.now();
    dt = current_time.toSec() - last_time.toSec();

    calculate_speed(dt);

    twist_odom_msg = get_twist_vel(wl_current_speed, wr_current_speed);
    twist_odom_pub.publish(&twist_odom_msg);

    last_time = current_time;
    nh.spinOnce();
    wait_ms(50);
  }
}
