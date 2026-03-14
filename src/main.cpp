#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <EEPROM.h>

#include <SPI.h>
#include <Wire.h>
#include <ColorConverterLib.h>
#include "Adafruit_TCS34725.h"

// TOF DEBUG
#include <Wire.h>
#include <VL53L0X.h>
// TOF DEBUG

#include "InverseKinematics/InverseKinematics.h"
#include "FiniteStateMachines/FiniteStateMachine.h"
#include "Commands/commands.h"
#include "Data/algorithms.h"

ServoMotorController* servo1;
ServoMotorController* servo2;
ServoMotorController* servo3;
ServoMotorController* servo4;

InverseKinematics* ik;
Coordinates target_pos, initial_pos, intermediate_pos, curr_pos;

//Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_120MS, TCS34725_GAIN_1X);

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_120MS, TCS34725_GAIN_4X);

commands_t serial_commands;

fsm_t fsm_move_to_position;
fsm_t fsm_main_control;
fsm_t fsm_pick_drop;
fsm_t fsm_search_tof;
fsm_t fsm_matrix;
fsm_t* fsm_array[FSM_COUNT] = {&fsm_move_to_position, &fsm_main_control, &fsm_pick_drop, &fsm_search_tof, &fsm_matrix};

float Dr, Dz, Dphi, Dt;
float vr, vz, vphi;

// flags
int flag_smooth_move, flag_pick_drop, flag_search_tof, flag_matrix;
int flag_config, flag_pick_drop_start, flag_search_tof_start, flag_matrix_start;
int flag_stop_tof, flag_matrix_stop;
int measured_color;

unsigned long now, interval, last_cycle;

// TOF
VL53L0X tof;
float distance;
int flag_measure_distance;
int num_cyc = 5; // do 5 cycles of 40 ms to measure distance for TOF
int count_distances;
int flag_add_distance;;
int delivered;
float distance_med;

// matrix
int count_matrix;


void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}

void get_color_data(float *r, float *g, float *b, float *sum)
{
	uint16_t red, green, blue, c;
	getRawData_noDelay(&red, &green, &blue, &c);

	*sum = c;
	*r = (float)red / *sum * 255.0;
	*g = (float)green / *sum * 255.0;
	*b = (float)blue / *sum * 255.0;
}

int measure_color(int print)
{
	// red 1, yellow 2, blue 3, green 4
	float r, g, b, c;
	int res;
	get_color_data(&r, &g, &b, &c);

	if ((r/g)>=1.25) 
	{
		res = 1;
	}
	else if((r/g)<1.25 && r>g)
	{
		res = 2;
	}
	else if (b > g)
	{
		res = 3;
	}
	else if (g>r && g>b)
	{
		res = 4;
	}
	else
	{
		res = 3;
	}

	if (print)
	{
		Serial.print("Red: ");
		Serial.println(r);
		Serial.print("Green: ");
		Serial.println(g);
		Serial.print("Blue: ");
		Serial.println(b);
		Serial.print("Result: ");
		Serial.println(res);
		switch (res)
		{
		case 1:
			Serial.println("RED");
			break;

		case 2:
			Serial.println("YELLOW");
		break;

		case 3:
			Serial.println("BLUE");
		break;

		case 4:
			Serial.println("GREEN");
		break;
		}
	}

	return res;
}

void process_command(char command, float value)
{
	// Movement commands
	if (command == 'q')
	{
		target_pos.r = value;
	}
	else if (command == 'w')
	{
		target_pos.phi = value;
	}
	else if (command == 'e')
	{
		target_pos.z = value;
	}
	else if (command == 'r')
	{
		Serial.print("\nMoving to position: r = ");
		Serial.print(target_pos.r);
		Serial.print(", phi = ");
		Serial.print(target_pos.phi);
		Serial.print(", z = ");
		Serial.println(target_pos.z);

		flag_smooth_move = 1;
	}
	else if (command == 'v')
	{
		Serial.print("\nCurrent position: r = ");
		Serial.print(curr_pos.r);
		Serial.print(", phi = ");
		Serial.print(curr_pos.phi);
		Serial.print(", z = ");
		Serial.println(curr_pos.z);
	}
	// Main Control Commands
	else if (fsm_main_control.state == 0)
	{
		if (command == 'c')
		{
			// event to enter the config mode
			flag_config = 1;
		}
		else if (command == 'z')
		{
			// event to start the pick and drop fsm
			flag_pick_drop_start = 1;
		}
		else if (command == 'x')
		{
			// event to start the search fsm
			flag_search_tof_start = 1;
			flag_stop_tof = 0;
		}
		else if (command == 'l')
		{
			// event to start the matrix fsm
			flag_matrix_start = 1;
		}
	}
	else if (fsm_main_control.state == 1)
	{
		// save the positions
		if (command == 'c')
		{
			// event to exit the config mode
			flag_config = 1;
		}
		else if (command == 'a')
		{
			// green deposit
			pos_green.r = curr_pos.r;
			pos_green.phi = curr_pos.phi;
			pos_green.z = curr_pos.z;
			Serial.print("Green deposit position stored: r = ");
			Serial.print(pos_green.r);
			Serial.print(", phi = ");
			Serial.print(pos_green.phi);
			Serial.print(", z = ");
			Serial.println(pos_green.z);

			// save to eeprom
			int addr = 0;
			addr += 2*sizeof(phi_min_tof);
			EEPROM.put(addr, pos_green);
		}
		else if (command == 's')
		{
			// red deposit
			pos_red.r = curr_pos.r;
			pos_red.phi = curr_pos.phi;
			pos_red.z = curr_pos.z;
			Serial.print("Red deposit position stored: r = ");
			Serial.print(pos_red.r);
			Serial.print(", phi = ");
			Serial.print(pos_red.phi);
			Serial.print(", z = ");
			Serial.println(pos_red.z);

			// save to eeprom
			int addr = 0;
			addr += 2*sizeof(phi_min_tof);
			addr += sizeof(pos_green);
			EEPROM.put(addr, pos_red);
		}
		else if (command == 'd')
		{
			// blue deposit
			pos_blue.r = curr_pos.r;
			pos_blue.phi = curr_pos.phi;
			pos_blue.z = curr_pos.z;
			Serial.print("Blue deposit position stored: r = ");
			Serial.print(pos_blue.r);
			Serial.print(", phi = ");
			Serial.print(pos_blue.phi);
			Serial.print(", z = ");
			Serial.println(pos_blue.z);

			// save to eeprom
			int addr = 0;
			addr += 2*sizeof(phi_min_tof);
			addr += 2*sizeof(pos_green);
			EEPROM.put(addr, pos_blue);
		}
		else if (command == 'f')
		{
			// yellow deposit
			pos_yellow.r = curr_pos.r;
			pos_yellow.phi = curr_pos.phi;
			pos_yellow.z = curr_pos.z;
			Serial.print("Yellow deposit position stored: r = ");
			Serial.print(pos_yellow.r);
			Serial.print(", phi = ");
			Serial.print(pos_yellow.phi);
			Serial.print(", z = ");
			Serial.println(pos_yellow.z);

			// save to eeprom
			int addr = 0;
			addr += 2*sizeof(phi_min_tof);
			addr += 3*sizeof(pos_green);
			EEPROM.put(addr, pos_yellow);
		}
		else if (command == 'g')
		{
			// color sensor position
			pos_color_sensor.r = curr_pos.r;
			pos_color_sensor.phi = curr_pos.phi;
			pos_color_sensor.z = curr_pos.z;
			Serial.print("Color sensor position stored: r = ");
			Serial.print(pos_color_sensor.r);
			Serial.print(", phi = ");
			Serial.print(pos_color_sensor.phi);
			Serial.print(", z = ");
			Serial.println(pos_color_sensor.z);

			// save to eeprom
			int addr = 0;
			addr += 2*sizeof(phi_min_tof);
			addr += 4*sizeof(pos_green);
			EEPROM.put(addr, pos_color_sensor);
		}
		else if (command == 'z')
		{
			// min angle tof
			phi_min_tof = value;
			Serial.print("Minimum angle for TOF search stored: ");
			Serial.println(phi_min_tof);

			// save to eeprom
			int addr = 0;
			EEPROM.put(addr, phi_min_tof);
		}
		else if (command == 'x')
		{
			// max angle tof
			phi_max_tof = value;
			Serial.print("Maximum angle for TOF search stored: ");
			Serial.println(phi_max_tof);

			// save to eeprom
			int addr = 0;
			addr += sizeof(phi_min_tof);
			EEPROM.put(addr, phi_max_tof);
		}
		else if (command == 'l')
		{
			if (0 < value && value < 10)
			{
				int ind = (int) value;
				ind -= 1;

				matrix_pos[ind].r = curr_pos.r;
				matrix_pos[ind].phi = curr_pos.phi;
				matrix_pos[ind].z = curr_pos.z;
				Serial.print("Matrix position ");
				Serial.print(ind + 1);
				Serial.print(" stored: r = ");
				Serial.print(matrix_pos[ind].r);
				Serial.print(", phi = ");
				Serial.print(matrix_pos[ind].phi);
				Serial.print(", z = ");
				Serial.println(matrix_pos[ind].z);

				// save to eeprom
				int addr = 0;
				addr += 2*sizeof(phi_min_tof);
				addr += 5*sizeof(pos_green);
				addr += ind*sizeof(matrix_pos[ind]);
				EEPROM.put(addr, matrix_pos[ind]);
			}
		}
	}
	else if (fsm_main_control.state == 3)
	{
		if (command == 'b')
		{
			flag_stop_tof = 1;
		}
	}
	else if (fsm_main_control.state == 4)
	{
		if (command == 'b')
		{
			flag_matrix_stop = 1;
		}
	}
}

void read_eeprom()
{
	int addr = 0;
	EEPROM.get(addr, phi_min_tof);
	addr += sizeof(phi_min_tof);
	EEPROM.get(addr, phi_max_tof);
	addr += sizeof(phi_max_tof);
	EEPROM.get(addr, pos_green);
	addr += sizeof(pos_green);
	EEPROM.get(addr, pos_red);
	addr += sizeof(pos_red);
	EEPROM.get(addr, pos_blue);
	addr += sizeof(pos_blue);
	EEPROM.get(addr, pos_yellow);
	addr += sizeof(pos_yellow);
	EEPROM.get(addr, pos_color_sensor);
	addr += sizeof(pos_color_sensor);
	for (int i = 0; i < 9; i++)
	{
		EEPROM.get(addr, matrix_pos[i]);
		addr += sizeof(matrix_pos[i]);
	}
}

void setup() 
{
	servo1 = new ServoMotorController(0, 490, 2460);
	servo2 = new ServoMotorController(1, 500, 2440);
	servo4 = new ServoMotorController(2, 530, 2505);
	servo3 = new ServoMotorController(3, 520, 2480);

	servo3->set_angular_pos(GRIPPER_ANGLE_OPEN);

	ik = new InverseKinematics(*servo1, *servo2, *servo4);
	Serial.begin(115200);

	interval = 40; //ms

	target_pos.r = 15;
	target_pos.z = 0;
	target_pos.phi = 0;
	flag_smooth_move = 0;

	serial_commands.init(process_command);
	
	// Section for the TCS34725
	Wire1.setSDA(6);  // Connect TCS34725 SDA to gpio 8
  	Wire1.setSCL(7);

	Wire1.begin();

	while (!tcs.begin(TCS34725_ADDRESS, &Wire1))
	{
		Serial.println("No TCS34725 found ... check your connections");
	}

	// TOF DEBUG
	Wire.setSDA(8);
  	Wire.setSCL(9);

  	Wire.begin();
  	tof.setTimeout(2000);

  	while (!tof.init()) {
    	Serial.println(F("Failed to detect and initialize VL53L0X!"));
  	}  
  	
	tof.setMeasurementTimingBudget(30000);

  	tof.startReadRangeMillimeters();

	EEPROM.begin(512);
	read_eeprom();
}

void loop() 
{
	uint8_t b;
    if (Serial.available())
	{  // Only do this if there is serial data to be read  
		b = Serial.read();
		char command = (char) b;
		Coordinates aux = ik->get_curr_pos();

		if (command == 't')
		{
			// subtract to r
			aux.r -= 0.25;
			ik->move_to_position(aux);
		}
		else if (command == 'y')
		{
			aux.r += 0.25;
			ik->move_to_position(aux);
		}
		else if (command == 'u')
		{
			aux.phi -= 0.5;
			ik->move_to_position(aux);
		}
		else if (command == 'i')
		{
			aux.phi += 0.5;
			ik->move_to_position(aux);
		}
		else if (command == 'o')
		{
			aux.z -= 0.25;
			ik->move_to_position(aux);
		}
		else if (command == 'p')
		{
			aux.z += 0.25;
			ik->move_to_position(aux);
		}  
		else if (command == 'n')
		{
			servo3->set_angular_pos(GRIPPER_ANGLE_OPEN);
		}
		else if (command == 'm')
		{
			servo3->set_angular_pos(GRIPPER_ANGLE_CLOSED);
		}
		else if (command == 'h')
		{
			// do a TOF reading and print the value
			Serial.print("d: ");
			Serial.println(distance);
		}
		else if (command == 'j')
		{
			measure_color(1);
		}
		else 
		{
			serial_commands.process_char(b);
			Serial.print((char) b);
		}
		
    }  	

	now = millis();

	if (now - last_cycle > interval)
	{
		// read inputs if there are inputs to read
		curr_pos = ik->get_curr_pos();

		// update current instant of time in all fsm
		update_all_fsm_tis(now);

		// read TOF sensor
		if (tof.readRangeAvailable())
		{
			distance = tof.readRangeMillimeters();
			tof.startReadRangeMillimeters();
		}

		// calculate transitions of all fsm
		// need to define the flag(event) that starts this machine
		if (fsm_move_to_position.state == 0 && flag_smooth_move)
		{
			int res1 = ik->check_if_aligned(target_pos.phi);
			int res2 = ik->check_if_curr_position_is_safe();
			// consume move event
			flag_smooth_move = 0;

			if (res1)
			{
				initial_pos = curr_pos;
				intermediate_pos = initial_pos;
				fsm_move_to_position.new_state = 3;
				Dr = target_pos.r - curr_pos.r;
				Dz = target_pos.z - curr_pos.z;
				Dt = sqrt(pow(Dr, 2) + pow(Dz, 2)) / ik->get_vel_linear();
				vr = Dr / sqrt(pow(Dr, 2) + pow(Dz, 2)) * ik->get_vel_linear();
				vz = Dz / sqrt(pow(Dr, 2) + pow(Dz, 2)) * ik->get_vel_linear();
			}
			else if (!res1 && !res2)
			{
				initial_pos = curr_pos;
				intermediate_pos = initial_pos;
				fsm_move_to_position.new_state = 1;
				Dr = POS_SAFE.r - curr_pos.r;
				Dz = POS_SAFE.z - curr_pos.z;
				Dt = sqrt(pow(Dr, 2) + pow(Dz, 2)) / ik->get_vel_linear();
				vr = Dr / sqrt(pow(Dr, 2) + pow(Dz, 2)) * ik->get_vel_linear();
				vz = Dz / sqrt(pow(Dr, 2) + pow(Dz, 2)) * ik->get_vel_linear();
			}
			else if (!res1 && res2)
			{
				initial_pos = curr_pos;
				intermediate_pos = initial_pos;
				fsm_move_to_position.new_state = 2;
				Dphi = target_pos.phi - curr_pos.phi;
				Dt = fabs(Dphi) / ik->get_vel_angular();
				vphi = Dphi / Dt;
			}
		}
		else if (fsm_move_to_position.state == 1)
		{
			if (fsm_move_to_position.tis > Dt)
			{
				fsm_move_to_position.new_state = 2;
				// to make sure that we end in the safe position with the correct angle phi
				intermediate_pos.r = POS_SAFE.r;
				intermediate_pos.z = POS_SAFE.z;
				intermediate_pos.phi = curr_pos.phi;
				ik->move_to_position(intermediate_pos);

				initial_pos = curr_pos;
				intermediate_pos = initial_pos;
				fsm_move_to_position.new_state = 2;
				Dphi = target_pos.phi - curr_pos.phi;
				Dt = fabs(Dphi) / ik->get_vel_angular();
				vphi = Dphi / Dt;
			}
		}
		else if (fsm_move_to_position.state == 2)
		{
			if (fsm_move_to_position.tis > Dt)
			{
				fsm_move_to_position.new_state = 3;
				// to make sure that we end in the position
				ik->move_to_position(target_pos);
				initial_pos = curr_pos;
				intermediate_pos = initial_pos;
				Dr = target_pos.r - curr_pos.r;
				Dz = target_pos.z - curr_pos.z;
				Dt = sqrt(pow(Dr, 2) + pow(Dz, 2)) / ik->get_vel_linear();
				vr = Dr / sqrt(pow(Dr, 2) + pow(Dz, 2)) * ik->get_vel_linear();
				vz = Dz / sqrt(pow(Dr, 2) + pow(Dz, 2)) * ik->get_vel_linear();
			}
		}
		else if (fsm_move_to_position.state == 3)
		{
			if (fsm_move_to_position.tis > Dt)
			{
				fsm_move_to_position.new_state = 0;
				// to make sure that we end in the position
				ik->move_to_position(target_pos);
			}
		}



		if (fsm_main_control.state == 0)
		{
			if (flag_config)
			{
				fsm_main_control.new_state = 1;
				print_config();
			}
			else if (flag_pick_drop_start)
			{
				fsm_main_control.new_state = 2;
				flag_pick_drop = 1;
			}
			else if (flag_search_tof_start)
			{
				fsm_main_control.new_state = 3;
				flag_search_tof = 1;
			}
			else if (flag_matrix_start)
			{
				fsm_main_control.new_state = 4;
				flag_matrix = 1;
			}
			flag_config = 0;
			flag_pick_drop_start = 0;
			flag_search_tof_start = 0;
			flag_matrix_start = 0;
		}
		else if (fsm_main_control.state == 1)
		{
			if (flag_config)
			{
				fsm_main_control.new_state = 0;
				flag_config = 0;
				Serial.println("Config Mode Exited");
				EEPROM.commit();
				print_main_control();
			}
		}
		else if (fsm_main_control.state == 2 && !fsm_pick_drop.state && fsm_main_control.tis > 3*interval)
		{
			fsm_main_control.new_state = 0;
		}
		else if (fsm_main_control.state == 3 && !fsm_search_tof.state && fsm_main_control.tis > 3*interval)
		{
			fsm_main_control.new_state = 0;
		}
		else if (fsm_main_control.state == 4 && !fsm_matrix.state && fsm_main_control.tis > 3*interval)
		{
			fsm_main_control.new_state = 0;
		}
		


		if (fsm_pick_drop.state == 0 && flag_pick_drop)
		{
			flag_pick_drop = 0;
			fsm_pick_drop.new_state = 1;
			// close gripper
			servo3->set_angular_pos(GRIPPER_ANGLE_CLOSED);
		}
		else if (fsm_pick_drop.state == 1 && fsm_pick_drop.tis > 250)
		{
			fsm_pick_drop.new_state = 2;
			// move to color sensor
			target_pos = pos_color_sensor;
			flag_smooth_move = 1;
		}
		else if (fsm_pick_drop.state == 2 && fsm_move_to_position.state == 0 && fsm_pick_drop.tis > 500 && fsm_move_to_position.tis > 500)
		{
			fsm_pick_drop.new_state = 3;
			// color measurement is ordered here
			measured_color = measure_color(1);
		}
		else if (fsm_pick_drop.state == 3 && fsm_pick_drop.tis > 500)
		{
			// we need to define a time for the measurement and according to that
			// we check the time we spent in this state
			fsm_pick_drop.new_state = 4;
			// the measurement is made and we have the color
			// for the moment we will just go to the red deposit
			switch (measured_color)
			{
			case 1:
				target_pos = pos_red;
				break;
			case 2:
				target_pos = pos_yellow;
				break;
			case 3:
				target_pos = pos_blue;
				break;
			case 4:
				target_pos = pos_green;
				break;
			}
			measured_color = 0;
			flag_smooth_move = 1;
		}
		else if (fsm_pick_drop.state == 4 && fsm_move_to_position.state == 0 && fsm_pick_drop.tis > 3*interval)
		{
			fsm_pick_drop.new_state = 5;
			
			// open the gripper
			servo3->set_angular_pos(GRIPPER_ANGLE_OPEN);
		}
		else if (fsm_pick_drop.state == 5 && fsm_pick_drop.tis > 250)
		{
			fsm_pick_drop.new_state = 6;
			// move to safe position
			target_pos.r = POS_SAFE.r;
			target_pos.z = POS_SAFE.z;
			target_pos.phi = curr_pos.phi;
			flag_smooth_move = 1;
		}
		else if (fsm_pick_drop.state == 6 && fsm_move_to_position.state == 0 && fsm_pick_drop.tis > 3*interval)
		{
			fsm_pick_drop.new_state = 0;
		}



		if (fsm_search_tof.state == 0 && flag_search_tof)
		{
			flag_search_tof = 0;
			flag_stop_tof = 0;
			fsm_search_tof.new_state = 1;
			
			// do movement to inital angle in a safe position position
			target_pos = POS_SAFE;
			target_pos.phi = phi_min_tof;
			flag_smooth_move = 1;
		}
		else if (fsm_search_tof.state != 0 && flag_stop_tof)
		{
			// emergency stop
			fsm_search_tof.new_state = 0;
			flag_stop_tof = 0;
		}
		else if (fsm_search_tof.state == 1 && !fsm_move_to_position.state && fsm_search_tof.tis > 3*interval)
		{
			fsm_search_tof.new_state = 2;

			// recalculate num_steps in case we changed the limits of search
			num_steps = phi_max_tof - phi_min_tof + 1;

			flag_add_distance = num_cyc;
		}
		else if (fsm_search_tof.state == 2 && count_distances == num_steps)
		{
			// reset the counter
			count_distances = 0;
			
			filtered_distances = moving_average_filter(distances, 3);

			minima_indices = find_local_minima(filtered_distances);


			if (minima_indices.size())
			{
				fsm_search_tof.new_state = 3;
				delivered = 0;
			}
			else
			{
				fsm_search_tof.new_state = 6;
				target_pos = POS_SAFE;
				delivered = 0;
				flag_smooth_move = 1;
			}
		}
		else if (fsm_search_tof.state == 3)
		{
			if (delivered < minima_indices.size())
			{
				// move to the next minimum
				target_pos.phi = angles[minima_indices[delivered]] + 1;
				target_pos.r = filtered_distances[minima_indices[delivered]]/10 + 0.5;
				Serial.print("Moving to position: r: ");
				Serial.print(target_pos.r);
				Serial.print("  D_f: ");
				Serial.print(filtered_distances[minima_indices[delivered]]/10);
				Serial.print("  D: ");
				Serial.println(distances[minima_indices[delivered]]/10);
				target_pos.z = -2;
				flag_smooth_move = 1;
				fsm_search_tof.new_state = 4;
			}
			else
			{
				Serial.println("All pieces delivered");
				Serial.print("Num pieces delivered: ");
				Serial.println(delivered);
				fsm_search_tof.new_state = 1;

				// do movement to inital angle in a safe position position
				target_pos = POS_SAFE;
				target_pos.phi = phi_min_tof;
				flag_smooth_move = 1;
			}
		}
		else if (fsm_search_tof.state == 4 && !fsm_move_to_position.state && fsm_search_tof.tis > 3*interval)
		{
			fsm_search_tof.new_state = 5;
			// open the gripper
			servo3->set_angular_pos(GRIPPER_ANGLE_OPEN);
			// do pick and drop
			flag_pick_drop = 1;
		}
		else if (fsm_search_tof.state == 5 && !fsm_pick_drop.state && fsm_search_tof.tis > 3*interval)
		{
			delivered += 1;
			fsm_search_tof.new_state = 3;
		}
		else if (fsm_search_tof.state == 6 && !fsm_move_to_position.state && fsm_search_tof.tis > 3*interval)
		{	
			// movement ended
			fsm_search_tof.new_state = 0;
			// open the gripper
			servo3->set_angular_pos(GRIPPER_ANGLE_OPEN);
		}



		if (fsm_matrix.state == 0 && flag_matrix)
		{
			flag_matrix = 0;
			flag_matrix_stop = 0;
			fsm_matrix.new_state = 1;
			count_matrix = 0;
		}
		else if (fsm_matrix.state && flag_matrix_stop)
		{
			fsm_matrix.new_state = 0;
			flag_matrix_stop = 0;
		}
		else if (fsm_matrix.state == 1)
		{
			if (count_matrix < 9)
			{
				target_pos = matrix_pos[count_matrix];
				flag_smooth_move = 1;
				fsm_matrix.new_state = 2;
			}
			else
			{
				fsm_matrix.new_state = 4;
				target_pos = POS_SAFE;
				flag_smooth_move = 1;
			}
		}
		else if (fsm_matrix.state == 2 && !fsm_move_to_position.state && fsm_matrix.tis > 3*interval)
		{
			fsm_matrix.new_state = 3;
			// do pick and drop
			flag_pick_drop = 1;
		}
		else if (fsm_matrix.state == 3 && !fsm_pick_drop.state && fsm_matrix.tis > 3*interval)
		{
			fsm_matrix.new_state = 1;
			count_matrix += 1;
		}
		else if (fsm_matrix.state == 4 && !fsm_move_to_position.state && fsm_matrix.tis > 3*interval)
		{
			fsm_matrix.new_state = 0;
		}



		// do the transitions to all the fsm
		set_fsm_state(fsm_move_to_position, fsm_move_to_position.new_state);
		set_fsm_state(fsm_main_control, fsm_main_control.new_state);
		set_fsm_state(fsm_pick_drop, fsm_pick_drop.new_state);
		set_fsm_state(fsm_search_tof, fsm_search_tof.new_state);
		set_fsm_state(fsm_matrix, fsm_matrix.new_state);


		// do the actions of all fsm
		if (fsm_move_to_position.state == 1 || fsm_move_to_position.state == 3)
		{
			// linear motion
			intermediate_pos.r = initial_pos.r + vr * fsm_move_to_position.tis;
			intermediate_pos.z = initial_pos.z + vz * fsm_move_to_position.tis;
			ik->move_to_position(intermediate_pos);
		}
		else if (fsm_move_to_position.state == 2)
		{
			// angular motion
			intermediate_pos.phi = initial_pos.phi + vphi * fsm_move_to_position.tis;
			ik->move_to_position(intermediate_pos);
		} 

		if (fsm_search_tof.state == 2)
		{
			// store distance
			if (flag_add_distance  < num_cyc - 2)
			{
				distance_med += distance;
			}

			if (!flag_add_distance)
			{
				// save distance in array for the distances
				distances[count_distances] = distance_med / (num_cyc - 2);
				distance_med = 0;

				// DEBUG PRINT
				//Serial.println(distances[count_distances]);
				
				// save angle in array for the angles
				angles[count_distances] = curr_pos.phi;

				// move to the next angle				
				target_pos = curr_pos;
				target_pos.phi += 1;
				ik->move_to_position(target_pos);

				count_distances += 1;
				flag_add_distance = num_cyc;
			}
			flag_add_distance -=  1;
		}

		// save the current instant to be used in next cycle as last cycle instant
		last_cycle = now;
	}
}