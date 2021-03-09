#include <servo.h>
#include <functions.h>

void setup()
{
    Serial.being(9600);
}

void loop()
{

    // put your main code here, to run repeatedly:
    static STATE machine_state = INITIALISING; // start from the sate INITIALIING
    switch (machine_state)
    {
    case INITIALISING:
        machine_state = initialising();
        break;
    case RUNNING:
        machine_state = running();
        break;
    case STOPPED:
        machine_state = stopped();
        break;
    }
}