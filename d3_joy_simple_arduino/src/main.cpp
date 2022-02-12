#include <Arduino.h>

const int MR_EN  = PA11;
const int MR_INA = PA12;
const int MR_INB = PA15;
const int MR_PWM = PB15;

const int ML_EN  = PB3;
const int ML_INA = PB4;
const int ML_INB = PB5;
const int ML_PWM = PB14;

String value;

String angle;
String strength;
String button;

float vx, vy;

uint8_t map_pwm_max_val;

uint8_t pwm_l;
uint8_t pwm_r;


float getVx(int strength, int angle);
float getVy(int strength, int angle);

void setMotors(int vx, int vy);
void stopMotors();

void setup()
{
    Serial1.begin(9600);
    
    pinMode(MR_EN, OUTPUT);
    pinMode(MR_INA, OUTPUT);
    pinMode(MR_INB, OUTPUT);
    pinMode(MR_PWM, OUTPUT);

    pinMode(ML_EN, OUTPUT);
    pinMode(ML_INA, OUTPUT);
    pinMode(ML_INB, OUTPUT);
    pinMode(ML_PWM, OUTPUT);

    digitalWrite(ML_EN, HIGH);

}

void loop()
{
    if (Serial1.available() > 0)
    {
        value = Serial1.readStringUntil('#');
        if (value.length() == 7)
        {
            angle = value.substring(0,3);
            strength = value.substring(3,6);
            button = value.substring(6,8);

            if (button.toInt() == 0) map_pwm_max_val = 75;
            else if (button.toInt() % 2) map_pwm_max_val = 120;
            else map_pwm_max_val = 0;

            vx = getVx(strength.toInt(), angle.toInt());
            vy = getVy(strength.toInt(), angle.toInt());

            setMotors(vx, vy);

            Serial1.print("Button: "); Serial1.println(button);
            
            Serial1.flush();
            value = "";
        }
    }
}

float getVx(int strength, int angle)
{
    return strength * cos(3.14*angle/180);
}

float getVy(int strength, int angle)
{
    return strength * sin(3.14*angle/180);
}

void setMotors(int vx, int vy)
{
    pwm_l = map(vx, -100, 100, -map_pwm_max_val, map_pwm_max_val) 
          + map(vy, -100, 100, -map_pwm_max_val, map_pwm_max_val)/3;
    pwm_r = map(vx, -100, 100, -map_pwm_max_val, map_pwm_max_val) 
          - map(vy, -100, 100, -map_pwm_max_val, map_pwm_max_val)/3;

    if (pwm_l > 0)
    {
        digitalWrite(ML_INA, LOW);
        digitalWrite(ML_INB, HIGH);
    }
    else
    {
        digitalWrite(ML_INB, LOW);
        digitalWrite(ML_INA, HIGH);
    }
    if (abs(pwm_l) > 5) analogWrite(ML_PWM, abs(pwm_l));
    else analogWrite(ML_PWM, 0);

    if (pwm_r > 0)
    {
        digitalWrite(MR_INB, LOW);
        digitalWrite(MR_INA, HIGH);
    }
    else
    {
        digitalWrite(MR_INA, LOW);
        digitalWrite(MR_INB, HIGH);
    }
    if (abs(pwm_r) > 5) analogWrite(MR_PWM, abs(pwm_r));
    else analogWrite(MR_PWM, 0);
}

void stopMotors()
{
    analogWrite(ML_PWM, 0);
    analogWrite(MR_PWM, 0);
}
