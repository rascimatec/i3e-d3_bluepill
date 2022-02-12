#include <Arduino.h>

const int MR_EN  = PA_11;
const int MR_INA = PA_12;
const int MR_INB = PA_15;
const int MR_PWM = PB_15;

const int ML_EN  = PB_3;
const int ML_INA = PB_4;
const int ML_INB = PB_5;
const int ML_PWM = PB_14;

String value;

String angle;
String strength;
String button;

float vx, vy;

float getVx(int strength, int angle);
float getVy(int strength, int angle);

void setup()
{
    Serial.begin(9600);
    
    pinMode(MR_EN, OUTPUT);
    pinMode(MR_INA, OUTPUT);
    pinMode(MR_INB, OUTPUT);
    pinMode(MR_PWM, OUTPUT);

    pinMode(ML_EN, OUTPUT);
    pinMode(ML_INA, OUTPUT);
    pinMode(ML_INB, OUTPUT);
    pinMode(ML_PWM, OUTPUT);
}

void loop()
{
    if (Serial.available() > 0)
    {
        value = Serial.readStringUntil('#');
        if (value.length() == 7)
        {
            angle = value.substring(0,3);
            strength = value.substring(3,6);
            button = value.substring(6,8);

            vx = getVx(strength.toInt(), angle.toInt());
            vy = getVy(strength.toInt(), angle.toInt());

            switch (button.toInt())
            {
            case 1:
                digitalWrite(MR_EN, HIGH);
                digitalWrite(MR_INA, LOW);
                digitalWrite(MR_INB, HIGH);
                analogWrite(MR_PWM, 255);

                digitalWrite(ML_EN, HIGH);
                digitalWrite(ML_INA, LOW);
                digitalWrite(ML_INB, HIGH);
                analogWrite(ML_PWM, 255);
                break;
            
            case 2:
                digitalWrite(MR_EN, HIGH);
                digitalWrite(MR_INA, HIGH);
                digitalWrite(MR_INB, LOW);
                analogWrite(MR_PWM, 255);

                digitalWrite(ML_EN, HIGH);
                digitalWrite(ML_INA, LOW);
                digitalWrite(ML_INB, HIGH);
                analogWrite(ML_PWM, 255);
                break;

            case 3:
                digitalWrite(MR_EN, HIGH);
                digitalWrite(MR_INA, HIGH);
                digitalWrite(MR_INB, LOW);
                analogWrite(MR_PWM, 255);

                digitalWrite(ML_EN, HIGH);
                digitalWrite(ML_INA, HIGH);
                digitalWrite(ML_INB, LOW);
                analogWrite(ML_PWM, 255);
                break;
            
            case 4:
                digitalWrite(MR_EN, HIGH);
                digitalWrite(MR_INA, LOW);
                digitalWrite(MR_INB, HIGH);
                analogWrite(MR_PWM, 255);

                digitalWrite(ML_EN, HIGH);
                digitalWrite(ML_INA, HIGH);
                digitalWrite(ML_INB, LOW);
                analogWrite(ML_PWM, 255);
                break;

            default:
                digitalWrite(MR_EN,  LOW);
                digitalWrite(MR_INA, LOW);
                digitalWrite(MR_INB, LOW);

                digitalWrite(ML_EN,  LOW);
                digitalWrite(ML_INA, LOW);
                digitalWrite(ML_INB, LOW);
                break;
            }

            Serial.println();
            
            Serial.flush();
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
