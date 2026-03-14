#include "ServoMotorController.h"
#include <Math.h>

ServoMotorController::ServoMotorController(int pin, int min, int max)
{
    this->pin = pin;
    this->min = min;
    this->max = max;
    this->angle = 0.0;
    
    // start the pin
    this->setup_servo_pin();

    // calibration servo params
    this->set_servo_params();
}


// Servo positioning methods
int ServoMotorController::convert_angular_to_time(int angle)
{
    // check if angle is in range of motion interval
    if (abs(angle) < 90) {
        return (int) (angle * this->m) + this->b;
    }
    return -1;   
}

int ServoMotorController::set_angular_pos(int angle)
{
    int time = this->convert_angular_to_time(angle);
    
    if (time != -1)
    {
        this->angle = angle;
        this->servo.writeMicroseconds(time);
        return 0;
    }
    return -1;
}

float ServoMotorController::get_angular_position()
{
    return this->angle;
}

// Config Methods
void ServoMotorController::set_servo_params()
{
    // m * angle + b = microsecs in servo
    this->b = (int) ((this->max + this->min)/ 2);
    this->m = (float) (this->max - this->min) / 180;
}

void ServoMotorController::setup_servo_pin()
{
    this->servo.attach(this->pin, SERVO_MIN, SERVO_MAX);
}