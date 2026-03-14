/* Copyright (c) 2022  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include <Arduino.h>
#include "commands.h"

commands_t::commands_t()
{
  count = 0;
  state = cs_wait_for_command;
  command = ' ';
  value = 0;
  process_command = NULL;

}


void commands_t::init(void (*process_command_function)(char command, float value))
{
  process_command = process_command_function;
}


void commands_t::process_char(char b)
{
  if (state == cs_wait_for_command && isalpha(b))  { // A command is allways a letter
    state = cs_reading_value;
    command = b;

  } else if (state == cs_reading_value && b == 0x0A)  { // LF (enter key received)
    // Now we can process the buffer
    if (count != 0) {
      data[count] = 0; // Null terminate the string
      value = atof((const char*)data);
    } 

    if (process_command)                  // If "process_command" is not null
      (*process_command)(command, value); // Do something with the pair (command, value)

    command = ' '; // Clear current command
    value = 0;     // Default value for "value"
    count = 0;     // Clear Buffer 
    state = cs_wait_for_command;

  } else if (state == cs_reading_value && count < COMMANDS_BUF_IN_SIZE - 1)  { // A new digit can be read
    data[count] = b;  // Store byte in the buffer
    count++;

  }
}

void print_main_control()
{
	Serial.println("\n##### Gripper 2000 Ultra Low Cost #####");
	Serial.println("Choose mode:");
	Serial.println(" - Collect Piece and Deposit According to color: press 'z'");
	Serial.println(" - Do a TOF search for the pieces: press 'x'");
  Serial.println(" - Do Matrix search: press 'l'");
	Serial.println(" - Enter the config mode: press 'c'");
	Serial.println("You can always do a manual movement with the movement commands");
	Serial.println("Commands:");
	Serial.println(" - 'q num' to define the r coordinate");
	Serial.println(" - 'w num' to define the phi coordinate");
	Serial.println(" - 'e num' to define the z coordinate");
	Serial.println(" - 'r' to move to the defined position");
	Serial.println(" - 't'/'y' to decrease/increase r");
	Serial.println(" - 'u'/'i' to decrease/increase phi");
	Serial.println(" - 'o'/'p' to decrease/increase z");
	Serial.println(" - 'n' to open the gripper");
	Serial.println(" - 'm' to close the gripper");
	Serial.println(" - 'h' to do a TOF measurement");
  Serial.println(" - 'b' to stop Matrix search and TOF measurement");
	Serial.println(" - 'v' get the current position");
}

void print_config()
{
	Serial.println("\nConfig Mode Entered");
	Serial.println("Press 'a' to save green deposit position");
	Serial.println("Press 's' to save red deposit position");
	Serial.println("Press 'd' to save blue deposit position");
	Serial.println("Press 'f' to save yellow deposit position");
	Serial.println("Press 'g' to save color sensor position");
	Serial.println("Write 'z num' to define minimum angle of tof search (predefined is -45)");
	Serial.println("Write 'x num' to define maximum angle of tof search (predefined is 45)");
  Serial.println("Write 'l num' where num is the matrix index to store the position");
}
