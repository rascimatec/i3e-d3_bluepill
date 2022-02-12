#include <Arduino.h>

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

            Serial.print("Angle: "); Serial.println(angle);
            Serial.print("Strength: "); Serial.println(strength);
            Serial.print("Button: "); Serial.println(button);

            Serial.print("Vx: "); Serial.println(vx);
            Serial.print("Vy: "); Serial.println(vy);

            
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
