#include "ServoMotorController/ServoMotorController.h"
#include <Math.h>
#include <vector>

// initial coordinates in cylindrical coordinates
#define R_0 5//4.5
#define PHI_0 0
#define Z_0 6

#define GRIPPER_ANGLE_OPEN -25
#define GRIPPER_ANGLE_CLOSED 25

struct Coordinates
{
    float r;
    float phi;
    float z;
};

extern Coordinates POS_SAFE;
extern Coordinates pos_red;
extern Coordinates pos_green;
extern Coordinates pos_blue;
extern Coordinates pos_yellow;
extern Coordinates pos_color_sensor;
extern Coordinates matrix_pos[9];
extern float phi_min_tof, phi_max_tof;
extern std::vector<float> distances;
extern std::vector<float> angles;
extern std::vector<int> minima_indices;
extern std::vector<float> filtered_distances;
extern int num_steps;   

class InverseKinematics 
{
    public:
        InverseKinematics(ServoMotorController& servo1, ServoMotorController& servo2, ServoMotorController& servo4);
        int move_to_position(Coordinates pos);
        Coordinates get_curr_pos();
        void set_limit_coords(Coordinates pos_min, Coordinates pos_max);
        int check_if_curr_position_is_safe();
        int check_if_aligned(float phi);
        void set_vel_angular(float vel);
        void set_vel_linear(float vel);
        float get_vel_angular();
        float get_vel_linear();
        
    private:
        ServoMotorController* servo1;
        ServoMotorController* servo2;
        ServoMotorController* servo4;
        float q1, q2, q3;
        float vel_linear, vel_angular;
        Coordinates pos_min, pos_max;
        Coordinates curr_pos;
};