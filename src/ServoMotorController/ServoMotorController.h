#include <Servo.h>

// constant variables
#define SERVO_MIN 400
#define SERVO_MAX 2600

class ServoMotorController 
{
    public:
        ServoMotorController(int pin, int min, int max);
        int set_angular_pos(int angle);
        float get_angular_position();

    private:
        void set_servo_params();
        void setup_servo_pin();
        int convert_angular_to_time(int angle);
        float m, angle;
        int b, min, max, pin;
        Servo servo;
};