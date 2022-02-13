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
int angle, strength, button;

int pwmr, pwml;
float multiplier = 1.0;

float vx, vy;
float getVx(int strength, int angle);
float getVy(int strength, int angle);


void move(int pwmr, int pwml);

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
    digitalWrite(MR_EN, HIGH);

}

void loop()
{
    if(Serial1.available() > 0)
    {
        value = Serial1.readStringUntil('#');
        if (value.length() == 7)
        {
            angle = value.substring(0,3).toInt();
            strength = value.substring(3,6).toInt();
            button = value.substring(6,8).toInt();

            if (strength < 40 && strength > -40) strength = 0;

            if (angle > 345 || angle < 15) angle = 0;
            else if (angle > 165 && angle < 195) angle = 180;
            else if (angle > 75 && angle < 105) angle = 90;
            else if (angle > 255 || angle < 285) angle = 270;

            if (button % 2) multiplier = 1.5;
            else if (button != 0) multiplier = 0.0;
            else multiplier = 1.0;
        }
    }

    vx = getVx(strength, angle);
    vy = getVy(strength, angle);

    pwmr = map(vx, -100, 100, -50, 50);
    pwml = pwmr;

    pwmr -= map(vy, -100, 100, -30, 30);
    pwml += map(vy, -100, 100, -30, 30);

    pwmr *= multiplier;
    pwml *= multiplier;

    move(pwmr, pwml);

}

void move(int pwmr, int pwml)
{
    bool dirr = pwmr > 0;
    bool dirl = pwml > 0;

    digitalWrite(MR_INA, dirr);
    digitalWrite(MR_INB, !dirr);

    digitalWrite(ML_INA, !dirl);
    digitalWrite(ML_INB, dirl);
    
    analogWrite(MR_PWM, abs(pwmr));
    analogWrite(ML_PWM, abs(pwml));
}

float getVx(int strength, int angle)
{
    return strength * cos(3.14*angle/180);
}

float getVy(int strength, int angle)
{
    return strength * sin(3.14*angle/180);
}
