#include <mbed.h>
#include "ros.h"
#include "geometry_msgs/Twist.h"

// Utilizar um .yaml
#define PI 3.14159265359
#define TWO_PI 2*PI
#define CONTROL_FREQUENCY 20
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
#define MAX_RPS 3.0

void ml_encoder_callback();
void mr_encoder_callback();
void calculate_speed();
void limit_differential_speed();
void process_pid();
void set_motors();
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

int ml_pulses = 0;
bool ml_reverse = false;
int mr_pulses = 0;
bool mr_reverse = false;

double wl_last_speed = 0.0, wr_last_speed = 0.0;
double wl_desired_speed = 0.0, wr_desired_speed = 0.0;
double wl_current_speed = 0.0, wr_current_speed = 0.0;
double wl_error = 0.0, wr_error = 0.0;
double wl_last_error = 0.0, wr_last_error = 0.0;
double wl_error_sum = 0.0, wr_error_sum = 0.0;
double wl_pid = 0.0, wr_pid = 0.0;
double wl_pwm = 0.0, wr_pwm = 0.0;

double kp = 1000.0,
       ki = 0.0,
       kd = 0.0;

double dt;
ros::Time current_time; 
ros::Time last_time;

ros::NodeHandle nh;

geometry_msgs::Twist twist_odom_msg;
ros::Publisher twist_odom_pub("twist_odom", &twist_odom_msg);

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &cmd_vel_callback);


void ml_encoder_callback(){
  ml_reverse = ML_C2.read();
  ml_reverse ? ml_pulses++ : ml_pulses--;
}

void mr_encoder_callback(){
  mr_reverse = MR_C2.read();
  mr_reverse ? mr_pulses++ : mr_pulses--;
}

void calculate_speed() {
  double delta_ang;

  delta_ang = ml_pulses*PULSE_DISTANCE;
  ml_pulses = 0;
  wl_current_speed = WHEEL_RADIUS*delta_ang/dt;
  
  delta_ang = mr_pulses*PULSE_DISTANCE;
  mr_pulses = 0;
  wr_current_speed = WHEEL_RADIUS*delta_ang/dt;
}

void limit_differential_speed() {

  double wl_accel = wl_desired_speed - wl_last_speed;
  double wr_accel = wr_desired_speed - wr_last_speed;

  double large_accel = ( std::max(std::abs(wl_accel), std::abs(wr_accel)) ) / dt;
  if (large_accel > MAX_ANGULAR_ACCEL) {
    wl_desired_speed = wl_last_speed + MAX_ANGULAR_ACCEL * (large_accel/wl_accel);
    wr_desired_speed = wr_last_speed + MAX_ANGULAR_ACCEL * (large_accel/wr_accel);
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

  char array[8];
  sprintf(array, "%f", wr_current_speed);
  nh.loginfo(array);

  wl_error_sum += wl_error;
  wr_error_sum += wr_error;

  wl_pid = kp * wl_error +
           ki * wl_error_sum * dt + 
           kd * (wl_error - wl_last_error) / dt;
  wr_pid = kp * wr_error + 
           ki * wr_error_sum*dt + 
           kd * (wr_error - wr_last_error) / dt;

  wl_last_error = wl_error;
  wr_last_error = wr_error;

  if (wl_pid > MAX_RPS) wl_pid = MAX_RPS;
  else if (wl_pid < -MAX_RPS) wl_pid = -MAX_RPS;
  if (wr_pid > MAX_RPS) wr_pid = MAX_RPS;
  else if (wr_pid < -MAX_RPS) wr_pid = -MAX_RPS;

  wl_pwm = wl_pid/MAX_RPS;
  wr_pwm = wr_pid/MAX_RPS;

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
}

geometry_msgs::Twist get_twist_vel(double wl_desired_speed, double wr_desired_speed) {
  geometry_msgs::Twist twist;
  twist.linear.x = WHEEL_RADIUS*(wr_desired_speed - wl_desired_speed) / 2;
  twist.angular.z = WHEEL_RADIUS*(wr_desired_speed - wl_desired_speed) / WHEEL_SEPARATION;
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
    last_time = current_time;

    calculate_speed();
    twist_odom_msg = get_twist_vel(wl_current_speed, wr_current_speed);
    twist_odom_pub.publish(&twist_odom_msg);

    limit_differential_speed();
    process_pid();
    set_motors();
    
    nh.spinOnce();
    wait_ms(1000/CONTROL_FREQUENCY);
  }
}
