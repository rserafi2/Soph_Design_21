/*
Project: Sophomore Design Project Milestone 1
Authors: Brian Ly, Ryan Serafin
Description: This code was to make the Polulu 3pi into a programmable robot that
took the paramaters of movement such as Forward, reverse, clockwise and
counter-clockwise, speeds of 3 different settings, and time incremented by 0.1.
*/

#include <stdint.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/io.h>
#include "lcd_driver.h"
#include "port_macros.h"
#define PWM_TOP 100
#define TIME_CONSTANT 13500
#define SPEED_SLOW 20
#define SPEED_MED 40
#define SPEED_FAST 60
#define DEFAULT_DURATION 10

// Prints message on top row of the LCD
void printTopLCD(char message[8]) {
  LCD_move_cursor_to_col_row(0, 0);
  LCD_print_String(message);
}

// Prints message on bottom row of the LCD
void printBotLCD(char message[8]) {
  LCD_move_cursor_to_col_row(0, 1);
  LCD_print_String(message);
}

// Makes left motor wheel stop
void stopLeftMotor() {
  PORTD |= (1 << 5);
  PORTD |= (1 << 6);
}

// Makes right motor wheel stop
void stopRightMotor() {
  PORTD |= (1 << 3);
  PORTB |= (1 << 3);
}

// Configures left motor to the direction specified
void configLeftMotor(int direction) {
  switch (direction) {
  // Counterclockwise
  case 0:
    PORTD |= (1 << 5);
    PORTD &= ~(1 << 6);
    break;
  // Clockwise
  case 1:
    PORTD &= ~(1 << 5);
    PORTD |= (1 << 6);
    break;
  // Forward
  case 2:
    PORTD &= ~(1 << 5);
    PORTD |= (1 << 6);
    break;
  // Reverse
  case 3:
    PORTD |= (1 << 5);
    PORTD &= ~(1 << 6);
    break;
  // No Movement
  default:
    PORTD |= (1 << 5);
    PORTD |= (1 << 6);
  }
}

// Configures right motor to the direction specified
void configRightMotor(int direction) {
  switch (direction) {
  // Counterclockwise
  case 0:
    PORTD &= ~(1 << 3);
    PORTB |= (1 << 3);
    break;
  // Clockwise
  case 1:
    PORTD |= (1 << 3);
    PORTB &= ~(1 << 3);
    break;
    // Forward
  case 2:
    PORTD &= ~(1 << 3);
    PORTB |= (1 << 3);
    break;
  // Reverse
  case 3:
    PORTD |= (1 << 3);
    PORTB &= ~(1 << 3);
    break;
  // No Movement
  default:
    PORTD |= (1 << 3);
    PORTD |= (1 << 3);
  }
}

/*
Makes the robot move according to the instructions specified.
int movement = the direction specified
  0: Counter-Clockwise
  1: Clockwise
  2: Forward
  3: Reverse
  Other: No movement (Pause)
int speed = speed of robot from 0-100 (not moving - maximum speed of motor)
int time = how long the movement should last in tenths of a second
*/
void runInstruction(int movement, int speed, int time) {
  int move_time = 0;
  int pwm_counter = 0;
  int wheel_offset = 0;
  int time_count = 0;

  // Special case when direction is reverse, there is drift correction
  if (movement == 3) {
    wheel_offset = 1;
  }
  for (time_count = 0; time_count < time; time_count++) {
    // The move_time loop runs for exactly 0.1 seconds. TIME_CONSTANT was calculated by testing the motors
    for (move_time = 0; move_time < TIME_CONSTANT; move_time++) {
      //pwm control
      pwm_counter = pwm_counter + 1;
      if (pwm_counter >= PWM_TOP) {
        pwm_counter = 0;
      }
      // Speed of each motor is based on pwm
      // wheel_offset affects the motor that was slightly fast
      if (pwm_counter < speed - wheel_offset) {
        configLeftMotor(movement);
      } else {
        stopLeftMotor();
      }
      if (pwm_counter < speed) {
        configRightMotor(movement);
      } else {
        stopRightMotor();
      }
      _delay_us(10);  // Delay to slow down loop
    }
  }
  stopLeftMotor();
  stopRightMotor();
}

int main() {
  // Variables used for button states
  unsigned int last_left_button_state = (PINB & (1 << 1));
  unsigned int left_button_pressed = 0;
  unsigned int last_right_button_state = (PINB & (1 << 5));
  unsigned int right_button_pressed = 0;
  unsigned int last_middle_button_state = (PINB & (1 << 4));
  unsigned int middle_button_pressed = 0;

  // These arrays hold the instruction information for each step
  int movement[4];
  int speed[4];
  int time[4];

  int interface_stage = 0;
  int instruction_step = 0;
  int command_max = 1;
  int command_index = 0;
  int move_type = 0;
  int duration = DEFAULT_DURATION;
  char command_string[8];
  char duration_string[8];
  char blanks[8] = "        ";

  // Configure Motor pins and ports
  DDRD |= (1 << 5) | (1 << 6) | (1 << 3);
  DDRB |= (1 << 3);
  // Configure middle push-button
  DDRB &= ~(1 << 4);
  PORTB |= (1 << 4);
  // Configure left push-button
  DDRB &= ~(1 << 1);
  PORTB |= (1 << 1);
  // Configure right push-button
  DDRB &= ~(1 << 5);
  PORTB |= (1 << 5);

  initialize_LCD_driver();
  LCD_execute_command(TURN_ON_DISPLAY);
  LCD_execute_command(CLEAR_DISPLAY);

  //Interface part of program (Stages 0-4)
  while (interface_stage < 5) {
    // Pulser for middle button
    if ((PINB & (1 << 4)) != last_middle_button_state) {
      if ((PINB & (1 << 4)) == 0) {
        middle_button_pressed = 1;
      }
      last_middle_button_state = (PINB & (1 << 4));
    } else {
      middle_button_pressed = 0;
    }
    // Pulser for left button
    if ((PINB & (1 << 1)) != last_left_button_state) {
      if ((PINB & (1 << 1)) == 0) {
        left_button_pressed = 1;
      }
      last_left_button_state = (PINB & (1 << 1));
    } else {
      left_button_pressed = 0;
    }
    // Pulser for right button
    if ((PINB & (1 << 5)) != last_right_button_state) {
      if ((PINB & (1 << 5)) == 0) {
        right_button_pressed = 1;
      }
      last_right_button_state = (PINB & (1 << 5));
    } else {
      right_button_pressed = 0;
    }

    // Interface control
    switch(interface_stage) {
      case 0: // Stage 0: Prompting user the number of input commands
        printTopLCD("Commands");
        itoa(command_max, command_string, 10); // Convert int to string
        printBotLCD(command_string);

        if (left_button_pressed == 1) {
          command_max--; // Decrease max commands when left button pressed
          if (command_max < 1) {
            command_max = 1; // Prevent max commands from going below 1
          }
        }
        if (right_button_pressed == 1) {
          command_max++; // Increase max commands when right button pressed
          if (command_max > 4) {
            command_max = 4; // Prevent command number from going above 4
          }
        }
        if (middle_button_pressed == 1) {
          interface_stage++; // Middle button used as select
          LCD_execute_command(CLEAR_DISPLAY);
        }
        break;
    case 1: // Stage 1: User selects movement
    /*
    Move_Type Legend:
    0: Counter-Clockwise (CCW)
    1: Clockwise (CW)
    2: Forward
    3: Reverse
    */
      printTopLCD("Movement");
      switch(move_type) {
        case 0:
        printBotLCD("CCW     ");
        break;
        case 1:
        printBotLCD("CW      ");
        break;
        case 2:
        printBotLCD("FORWARD ");
        break;
        case 3:
        printBotLCD("REVERSE ");
        break;
      }
      // Allows users to cycle options
      if (left_button_pressed == 1) {
        move_type--;
        if (move_type < 0) {
          move_type = 3;
        }
      }
      if (right_button_pressed == 1) {
        move_type++;
        if (move_type > 3) {
          move_type = 0;
        }
      }
      if (middle_button_pressed == 1) {
        LCD_execute_command(CLEAR_DISPLAY);
        movement[command_index] = move_type; // Saves movement type
        interface_stage++; // Middle button used as Select
      }
      break;
    case 2: // Stage 2: User selects speed
        printTopLCD("Speed");
        printBotLCD("S   M  F"); //Display all three settings on LCD
        /*
        All three speed settings are displayed on LCD
        Left Button: Slow Setting
        Middle Button: Medium Setting
        Right Button: High Setting
        After button is selected, code proceeds to next stage.
        */
        if (left_button_pressed == 1) {
          speed[command_index] = SPEED_SLOW;
          interface_stage++;
          LCD_execute_command(CLEAR_DISPLAY);
        }
        if (middle_button_pressed == 1) {
          speed[command_index] = SPEED_MED;
          interface_stage++;
          LCD_execute_command(CLEAR_DISPLAY);
        }
        if (right_button_pressed == 1) {
          speed[command_index] = SPEED_FAST;
          LCD_execute_command(CLEAR_DISPLAY);
          interface_stage++;
        }
        break;
    case 3:
      printTopLCD("Time");
      itoa(duration, duration_string, 10); // Convert integer into string
      printBotLCD(duration_string);
      LCD_print_String(blanks); // Print blank after duration for no remaining 0's

      if (left_button_pressed == 1) {
        duration--; // Decreases duration when left button pressed
        if (duration < 0) {
          duration = 0; // Prevents duration from going below 0
        }
      }
      if (right_button_pressed == 1) {
        duration++; // Increase duration when right button pressed
      }
      if (middle_button_pressed == 1) {
        time[command_index] = duration; // Select current duration
        duration = DEFAULT_DURATION;
        LCD_execute_command(CLEAR_DISPLAY);
        interface_stage++;
      }
      break;
    case 4:
    /*
    Stage 4: Code checks if user inputted enough paramaters for the amount of
    commands the user chose. Robot starts the commands after a delay.
    */
      command_index++;
      if (command_index < command_max) {
        interface_stage = 1;
      } else {
        interface_stage++;
      }
      break;
    }
    _delay_us(10); // Small delay to slow down main loop
  }

  runInstruction(4, 0, 10); // Robot pauses for 1 second

  // Robot movement part of the program
  for (instruction_step = 0; instruction_step < command_max; instruction_step++) {
    runInstruction(movement[instruction_step], speed[instruction_step], time[instruction_step]);
    runInstruction(4, 0, 5); // Pause for 0.5 seconds
  }
  return 0;
}
