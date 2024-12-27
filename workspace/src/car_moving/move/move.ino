#include "MecanumDriver.h"

MecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);

int speed[4] = {0, 0, 0, 0};
String command;

void setup()
{
    mecanum.begin();
    Serial.begin(115200);
}

void loop()
{
    while (Serial.available() > 0)
    {
        char temp = char(Serial.read());
        if ((temp >= '0' && temp <= '9') || (temp >= 'a' && temp <= 'z'))
        {
            command += temp;
        }
    }

    if (command.length() > 0)
    {
        int begin = message.indexOf('<');
        int end = message.indexOf('>') + 1;
        if (begin != -1 && end != -1 && begin < end)
        {
            sscanf(message.substring(begin, end).c_str(), "<%d,%d,%d,%d>", &speed[0], &speed[1], &speed[2], &speed[3]);
        }
    }

    mecanum.setDutyCycle(speed[0], speed[1], speed[2], speed[3]);
    delay(20);
}