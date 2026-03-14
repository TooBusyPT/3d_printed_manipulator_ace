#include "InverseKinematics/InverseKinematics.h"

// min and max coordinates in cylindrical coordinates
Coordinates POS_MIN_0 = {4.5, -89, -3};
Coordinates POS_MAX_0 = {15, 89, 6};
Coordinates POS_SAFE = {5, 0, 6};
Coordinates pos_red = {11, 89, -1};
Coordinates pos_green = {12.5, 89, -1};
Coordinates pos_blue = {11, 73, -1};
Coordinates pos_yellow = {13, 72, -1};
Coordinates pos_color_sensor = {13, -88, 1};
float phi_min_tof = -45;
float phi_max_tof = 45;

// predefined values for matrix positions
Coordinates matrix_pos[9] = {
    {8.75, -20, -1.75},
    {8.5, 0.5, -1.75},
    {9.25, 20, -2},
    {12, -15, -2},
    {11.5, 3.5, -1.75},
    {12, 19.0, -2},
    {14.25, -14, -1.75},
    {14.0, 2.0, -1.75},
    {14.5, 17.5, -1.75}
};

// Assuming phi_min_tof and phi_max_tof are defined somewhere
int num_steps = phi_max_tof - phi_min_tof + 1;

// Create a vector to store the distances
std::vector<float> distances(num_steps);

// Create a vector to store the angles
std::vector<float> angles(num_steps);

// Create a vector to store the indexes of the minima
std::vector<int> minima_indices;

// Create a vector to store the filtered distances
std::vector<float> filtered_distances;

InverseKinematics::InverseKinematics(ServoMotorController& servo1, ServoMotorController& servo2, ServoMotorController& servo4)
{
    this->servo1 = &servo1;
    this->servo2 = &servo2;
    this->servo4 = &servo4;

    // save initial position
    this->curr_pos.r = R_0;
    this->curr_pos.phi = PHI_0;
    this->curr_pos.z = Z_0;

    // set initial velocities
    this->set_vel_linear(0.027); // cm/ms
    this->set_vel_angular(0.058); // deg/ms

    // save initial limits
    this->set_limit_coords(POS_MIN_0, POS_MAX_0);

    // move to initial position
    this->move_to_position(this->curr_pos);
}

void InverseKinematics::set_limit_coords(Coordinates pos_min, Coordinates pos_max)
{
    this->pos_max = pos_max;
    this->pos_min = pos_min;
}

int InverseKinematics::move_to_position(Coordinates pos)
{
    float theta1, theta2, theta3;
    float q1_aux, q2_aux, q3_aux;

    // check if position is within limits
    if (pos.r < this->pos_min.r || pos.phi < this->pos_min.phi || pos.phi > this->pos_max.phi || pos.z < this->pos_min.z || pos.z > this->pos_max.z)
    {
        return-1;
    }

    if (pos.r > this->pos_max.r )
    {
        pos.r = this->pos_max.r;
    }

    // calculate joint angles
    q1_aux = pos.phi;
    q3_aux = -2*atan(sqrt((16*16/(pos.r*pos.r + pos.z*pos.z) - 1)));;
    q2_aux = atan2(pos.z, pos.r) - atan2(8*sin(q3_aux), 8*(1 + cos(q3_aux))) - PI/2;

    // calculate servo angles
    theta1 = q1_aux;
    theta2 = -q2_aux;

    if (theta2 >= 0)
    {
        theta3 = (PI - fabs(theta2) - fabs(q3_aux));
    }
    else
    {
        theta3 = (PI + fabs(theta2) - fabs(q3_aux));
    }

    theta1 = theta1;
    theta2 = theta2/PI*180;
    theta3 = theta3/PI*180;

    // check angle within servo limits
    if (fabs(theta1) > 90 || fabs(theta2) > 90 || fabs(theta3) > 90)
    {
        return -1;
    }

    // save new position
    this->curr_pos.r = pos.r;
    this->curr_pos.phi = pos.phi;
    this->curr_pos.z = pos.z;
    this->q1 = q1_aux;
    this->q2 = q2_aux;
    this->q3 = q3_aux;

    // according to the servo numbers
    this->servo1->set_angular_pos(theta3);                
    this->servo2->set_angular_pos(theta2);
    this->servo4->set_angular_pos(theta1);
    
    return 0;
}

Coordinates InverseKinematics::get_curr_pos()
{
    return this->curr_pos;
}

int InverseKinematics::check_if_curr_position_is_safe()
{
    if (this->curr_pos.r <= POS_SAFE.r && this->curr_pos.z >= POS_SAFE.z)
    {
        return 1;
    }
    return 0;
}

int InverseKinematics::check_if_aligned(float phi)
{
    if (fabs(this->curr_pos.phi - phi) < 0.1)
    {
        return 1;
    }
    return 0;
}

void InverseKinematics::set_vel_angular(float vel)
{
    this->vel_angular = vel;
}

void InverseKinematics::set_vel_linear(float vel)
{
    this->vel_linear = vel;
}

float InverseKinematics::get_vel_angular()
{
    return this->vel_angular;
}

float InverseKinematics::get_vel_linear()
{
    return this->vel_linear;
}