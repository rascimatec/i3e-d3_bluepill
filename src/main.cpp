#include <mbed.h>

#define PI 3.14159265359
#define TWO_PI 2*PI
#define CONTROL_FREQUENCY 10
#define WHEEL_RADIUS 0.09
#define WHEEL_SEPARATION 0.38
#define GEAR_REDUCTION_RATIO 90
#define BASIC_PULSE_NUMBER 17
#define PULSES_PER_REVOLUTION (GEAR_REDUCTION_RATIO*BASIC_PULSE_NUMBER)
#define PULSE_DISTANCE (TWO_PI/PULSES_PER_REVOLUTION)
#define MAX_LINEAR_SPEED 1.6
#define MAX_ANGULAR_SPEED (MAX_LINEAR_SPEED/WHEEL_RADIUS)
#define MAX_LINEAR_ACCEL 4.0
#define MAX_ANGULAR_ACCEL (MAX_LINEAR_ACCEL/WHEEL_RADIUS)
#define MAX_PID (3.0*TWO_PI)
#define DT 1/CONTROL_FREQUENCY

struct vel_vector {
  double lin = 0.0;
  double ang = 0.0;
};

struct vel_wheels {
  double wr = 0.0;
  double wl = 0.0;
};


vel_wheels get_wheels_velocity();
vel_vector get_odom(vel_wheels wheels_speed);

void ml_encoder_callback();
void mr_encoder_callback();

void constrain_abs(double &var, double max_abs);

void limit_differential_speed();
void process_pid();
void set_motors();


Serial rasp(PA_9, PA_10, 115200);

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


vel_wheels wheels_last_speed;
vel_wheels wheels_current_speed;
vel_wheels wheels_desired_speed;
vel_wheels wheels_error;
vel_wheels wheels_last_error;
vel_wheels wheels_error_sum;
vel_wheels wheels_pid;
vel_wheels wheels_pwm;

vel_vector odom_vel;

double kp = 1.0,
       ki = 1.8,
       kd = 0.0;


void constrain_abs(double &var, double max_abs){
  if (var < -max_abs) var = -max_abs;
  else if (var > max_abs) var = max_abs;
}


void ml_encoder_callback(){
  ml_reverse = ML_C2.read();
  ml_reverse ? ml_pulses++ : ml_pulses--;
}

void mr_encoder_callback(){
  mr_reverse = MR_C2.read();
  mr_reverse ? mr_pulses++ : mr_pulses--;
}


vel_wheels get_wheels_velocity(){
  double delta_ang;
  vel_wheels wv;

  delta_ang = ml_pulses*PULSE_DISTANCE;
  ml_pulses = 0;
  wv.wl = delta_ang/DT;
  
  delta_ang = mr_pulses*PULSE_DISTANCE;
  mr_pulses = 0;
  wv.wr = delta_ang/DT;

  return wv;
}


vel_vector get_odom(vel_wheels wheels_speed) {
  vel_vector odom;
  odom.lin = WHEEL_RADIUS*(wheels_speed.wr + wheels_speed.wl) / 2;
  odom.ang = WHEEL_RADIUS*(wheels_speed.wr - wheels_speed.wl) / WHEEL_SEPARATION;
  return odom;
}


void limit_differential_speed() {
  vel_wheels w_accel;
  w_accel.wl = wheels_desired_speed.wl - wheels_last_speed.wl;
  w_accel.wr = wheels_desired_speed.wr - wheels_last_speed.wr;
  
  double large_accel = ( std::max(std::abs(w_accel.wl), std::abs(w_accel.wr)) ) / DT;
  if (large_accel > MAX_ANGULAR_ACCEL) {
    wheels_desired_speed.wl = wheels_last_speed.wl + MAX_ANGULAR_ACCEL * (w_accel.wl/large_accel);
    wheels_desired_speed.wr = wheels_last_speed.wr + MAX_ANGULAR_ACCEL * (w_accel.wr/large_accel);
  }

  double large_speed = std::max(std::abs(wheels_desired_speed.wl), std::abs(wheels_desired_speed.wr));
  if (large_speed > MAX_ANGULAR_SPEED) {
    wheels_desired_speed.wl *= MAX_ANGULAR_SPEED / large_speed;
    wheels_desired_speed.wr *= MAX_ANGULAR_SPEED / large_speed;
  }

  wheels_last_speed.wl = wheels_desired_speed.wl;
  wheels_last_speed.wr = wheels_desired_speed.wr;

}


void process_pid() {
  wheels_error.wl = wheels_desired_speed.wl - wheels_current_speed.wl;
  wheels_error.wr = wheels_desired_speed.wr - wheels_current_speed.wr;

  wheels_error_sum.wl += wheels_error.wl;
  wheels_error_sum.wr += wheels_error.wr;

  constrain_abs(wheels_error_sum.wl, 5*MAX_PID);
  constrain_abs(wheels_error_sum.wr, 5*MAX_PID);

  wheels_pid.wl = kp * wheels_error.wl +
                  ki * wheels_error_sum.wl * DT + 
                  kd * (wheels_error.wl - wheels_last_error.wl) / DT;
  wheels_pid.wr = kp * wheels_error.wr +
                  ki * wheels_error_sum.wr * DT + 
                  kd * (wheels_error.wr - wheels_last_error.wr) / DT;

  wheels_last_error.wl = wheels_error.wl;
  wheels_last_error.wr = wheels_error.wr;

  constrain_abs(wheels_pid.wl, MAX_PID);
  constrain_abs(wheels_pid.wr, MAX_PID);

  wheels_pwm.wl = wheels_pid.wl/MAX_PID + wheels_desired_speed.wl/MAX_PID;
  wheels_pwm.wr = wheels_pid.wr/MAX_PID + wheels_desired_speed.wr/MAX_PID;
  constrain_abs(wheels_pwm.wl, 1.0);
  constrain_abs(wheels_pwm.wr, 1.0);

  if (wheels_desired_speed.wl > 0 && wheels_pwm.wl < 0.0) wheels_pwm.wl = 0.01;
  if (wheels_desired_speed.wl < 0 && wheels_pwm.wl > 0.0) wheels_pwm.wl = -0.01;
  if (wheels_desired_speed.wr < 0 && wheels_pwm.wr > 0.0) wheels_pwm.wr = 0.01;
  if (wheels_desired_speed.wr < 0 && wheels_pwm.wr > 0.0) wheels_pwm.wr = -0.01;

}


void set_motors() {

  if (wheels_pwm.wl > 0) { ML_INA  = 0; ML_INB = 1; }
  else { ML_INA = 1; ML_INB = 0;}
  // ML_PWM = std::abs(wheels_pwm.wl);  

  if (wheels_pwm.wr > 0) { MR_INA  = 0; MR_INB = 1; }
  else { MR_INA = 1; MR_INB = 0;}
  // MR_PWM = std::abs(wheels_pwm.wr);  

}


int main() {


  ML_C1.rise(&ml_encoder_callback);
  MR_C1.rise(&mr_encoder_callback);
  
  ML_EN = 1;
  MR_EN = 1;

  char coming_data[6];
  vel_vector coming_vel, desired_vel;
  coming_vel.lin = 0.0;
  coming_vel.ang = 0.0;
  desired_vel.lin = 0.0;
  desired_vel.ang = 0.0;

  vel_wheels desired_vels;


  while (1) {
    
    // Get coming velocity data
    if (rasp.readable()) {
      if (rasp.getc() == 'v') {
        
        for(int i=0; i<6; i++)
          coming_data[i] = rasp.getc();

        coming_vel.lin = (coming_data[0] - '0')*100 + (coming_data[1] - '0')*10 + (coming_data[2] - '0');
        coming_vel.ang = (coming_data[3] - '0')*100 + (coming_data[4] - '0')*10 + (coming_data[5] - '0');
        
        wheels_desired_speed.wr = (coming_vel.lin/WHEEL_RADIUS) + ((coming_vel.ang*WHEEL_SEPARATION)/(2.0*WHEEL_RADIUS));
        wheels_desired_speed.wl = (coming_vel.lin/WHEEL_RADIUS) - ((coming_vel.ang*WHEEL_SEPARATION)/(2.0*WHEEL_RADIUS));
        limit_differential_speed();
      }
    }
    //

    wheels_current_speed = get_wheels_velocity();
    odom_vel = get_odom(wheels_current_speed);
    rasp.printf("v%03d%03d", odom_vel.lin, odom_vel.ang);
    
    process_pid();




    wait_ms(1000*DT);

  }
}
