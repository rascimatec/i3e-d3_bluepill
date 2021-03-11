#include <mbed.h>
#include "ros.h"
#include "geometry_msgs/Twist.h"

// Utilizar um .yaml
#define PI 3.14159265359
#define TWO_PI 2*PI
#define CONTROL_FREQUENCY 10
#define WHEEL_RADIUS 0.09
#define WHEEL_SEPARATION 0.3
#define GEAR_REDUCTION_RATIO 90
#define BASIC_PULSE_NUMBER 17
#define PULSES_PER_REVOLUTION (GEAR_REDUCTION_RATIO*BASIC_PULSE_NUMBER)
#define PULSE_DISTANCE (TWO_PI/PULSES_PER_REVOLUTION)
#define MAX_LINEAR_SPEED 1.6
#define MAX_ANGULAR_SPEED (MAX_LINEAR_SPEED/WHEEL_RADIUS)
#define MAX_LINEAR_ACCEL 4.0
#define MAX_ANGULAR_ACCEL (MAX_LINEAR_ACCEL/WHEEL_RADIUS)
#define MAX_PID (3.0*TWO_PI)

void ticker_millis();
void ml_encoder_callback();
void mr_encoder_callback();
void calculate_speed();
void limit_differential_speed();
void process_pid();
void set_motors();
void constrain_abs(double &var, double max);
void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel);
geometry_msgs::Twist get_twist_vel(double wl_desired_speed, double wr_desired_speed);

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

Ticker ticker;

int ml_pulses = 0;
bool ml_reverse = false;
int mr_pulses = 0;
bool mr_reverse = false;

double wl_last_speed = 0.0, wr_last_speed = 0.0;
double wl_desired_speed = 0.0, wr_desired_speed = 0.0;
double wl_current_speed = 0.0, wr_current_speed = 0.0;
double wl_error = 0.0, wr_error = 0.0;
double wl_last_error = 0.0, wr_last_error = 0.0;
double wl_error_sum = 0.0;
double wr_error_sum = 0.0;
double wl_pid = 0.0, wr_pid = 0.0;
double wl_pwm = 0.0, wr_pwm = 0.0;

double kp = 1.0,
       ki = 1.8,
       kd = 0.0;

unsigned long int millis = 0;
double dt;
double current_time; 
double last_time;

ros::NodeHandle nh;

geometry_msgs::Twist twist_odom_msg;
ros::Publisher twist_odom_pub("twist_odom", &twist_odom_msg);

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_callback);


void ticker_millis(){
  millis++;
}

void ml_encoder_callback(){
  ml_reverse = !ML_C2.read();
  ml_reverse ? ml_pulses++ : ml_pulses--;
}

void mr_encoder_callback(){
  mr_reverse = !MR_C2.read();
  mr_reverse ? mr_pulses++ : mr_pulses--;
}

void calculate_speed() {
  double delta_ang;

  delta_ang = ml_pulses*PULSE_DISTANCE;
  ml_pulses = 0;
  wl_current_speed = delta_ang/0.1;
  
  delta_ang = mr_pulses*PULSE_DISTANCE;
  mr_pulses = 0;
  wr_current_speed = delta_ang/0.1;
}

void limit_differential_speed() {

  double wl_accel = wl_desired_speed - wl_last_speed;
  double wr_accel = wr_desired_speed - wr_last_speed;

  double large_accel = ( std::max(std::abs(wl_accel), std::abs(wr_accel)) ) / 0.1;
  if (large_accel > MAX_ANGULAR_ACCEL) {
    wl_desired_speed = wl_last_speed + MAX_ANGULAR_ACCEL * (wl_accel/large_accel);
    wr_desired_speed = wr_last_speed + MAX_ANGULAR_ACCEL * (wr_accel/large_accel);
  }

  double large_speed = std::max(std::abs(wl_desired_speed), std::abs(wr_desired_speed));
  if (large_speed > MAX_ANGULAR_SPEED) {
    wl_desired_speed *= MAX_ANGULAR_SPEED / large_speed;
    wl_desired_speed *= MAX_ANGULAR_SPEED / large_speed;
  }

  wl_last_speed = wl_desired_speed;
  wr_last_speed = wr_desired_speed;
}

void process_pid() {
  wl_error = wl_desired_speed - wl_current_speed;
  wr_error = wr_desired_speed - wr_current_speed;

  wl_error_sum += wl_error;
  wr_error_sum += wr_error;
  constrain_abs(wl_error_sum, 5*MAX_PID);
  constrain_abs(wr_error_sum, 5*MAX_PID);

  wl_pid = kp * wl_error +
           ki * wl_error_sum * 0.1 + 
           kd * (wl_error - wl_last_error) / 0.01;
  wr_pid = kp * wr_error + 
           ki * wr_error_sum * 0.1 + 
           kd * (wr_error - wr_last_error) / 0.01;

  wl_last_error = wl_error;
  wr_last_error = wr_error;

  constrain_abs(wl_pid, MAX_PID);
  constrain_abs(wr_pid, MAX_PID);

  wl_pwm = wl_pid/MAX_PID + wl_desired_speed/MAX_PID;
  wr_pwm = wr_pid/MAX_PID + wr_desired_speed/MAX_PID;
  constrain_abs(wl_pwm, 1.0);
  constrain_abs(wr_pwm, 1.0);

  if (wl_desired_speed > 0 && wl_pwm < 0.0) wl_pwm = 0.01;
  if (wl_desired_speed < 0 && wl_pwm > 0.0) wl_pwm = -0.01;
  if (wr_desired_speed < 0 && wr_pwm > 0.0) wr_pwm = 0.01;
  if (wr_desired_speed < 0 && wr_pwm > 0.0) wr_pwm = -0.01;

  char array[8];
  sprintf(array, "%f", wl_pid);
  nh.loginfo(array);

}

void set_motors() {

  if (wl_pwm > 0) { ML_INA  = 0; ML_INB = 1; }
  else { ML_INA = 1; ML_INB = 0;}
  ML_PWM = std::abs(wl_pwm);  

  if (wr_pwm > 0) { MR_INA  = 0; MR_INB = 1; }
  else { MR_INA = 1; MR_INB = 0;}
  MR_PWM = std::abs(wr_pwm);  

}

void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel){
  double vel_lin = cmd_vel.linear.x;
  double vel_ang = cmd_vel.angular.z;
  wr_desired_speed = (vel_lin/WHEEL_RADIUS) + ((vel_ang*WHEEL_SEPARATION)/(2.0*WHEEL_RADIUS));
  wl_desired_speed = (vel_lin/WHEEL_RADIUS) - ((vel_ang*WHEEL_SEPARATION)/(2.0*WHEEL_RADIUS));
  limit_differential_speed();
}

geometry_msgs::Twist get_twist_vel(double wl_desired_speed, double wr_desired_speed) {
  geometry_msgs::Twist twist;
  twist.linear.x = WHEEL_RADIUS*(wr_desired_speed + wl_desired_speed) / 2;
  twist.angular.z = WHEEL_RADIUS*(wr_desired_speed - wl_desired_speed) / WHEEL_SEPARATION;
  return twist;
}

void constrain_abs(double &var, double max_abs){
  if (var < -max_abs) var = -max_abs;
  else if (var > max_abs) var = max_abs;
}


int main() {

  ticker.attach(&ticker_millis, 0.001);

  nh.initNode();
  nh.advertise(twist_odom_pub);
  nh.subscribe(cmd_vel_sub);

  ML_C1.rise(&ml_encoder_callback);
  MR_C1.rise(&mr_encoder_callback);
  
  ML_EN = 1;
  MR_EN = 1;

  last_time = millis;

  while(1) {
    
    current_time = millis;
    dt = current_time - last_time;
    last_time = current_time;

    calculate_speed();
    twist_odom_msg = get_twist_vel(wl_current_speed, wr_current_speed);
    //twist_odom_msg.linear.x = wl_current_speed;
    //twist_odom_msg.linear.y = wl_desired_speed;
    //twist_odom_msg.linear.z = wl_error_sum;
    //twist_odom_msg.angular.x = wr_current_speed;
    //twist_odom_msg.angular.y = wr_desired_speed;
    //twist_odom_msg.angular.z = wr_error_sum;
    twist_odom_pub.publish(&twist_odom_msg);

    process_pid();
    set_motors();
    
    nh.spinOnce();
    wait_ms(1000/CONTROL_FREQUENCY);
  }
}
