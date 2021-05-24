/*
Project: Sophomore Design Project Milestone 1
Authors: Brian Ly, Ryan Serafin
Description: This code was to make the Polulu 3pi into a programmable robot that
took the paramaters of movement such as Forward, reverse, clockwise and
counter-clockwise, speeds of 3 different settings, and time incremented by 0.1.
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/io.h>
#include "lcd_driver.h"
#include "port_macros.h"
#define PWM_TOP 100
#define TIME_CONSTANT 1000
#define SPEED_SLOW 10
#define SPEED_MED 25
#define SPEED_FAST 35
#define BUZZER_VOL 85
#define L_MOTOR_1 5
#define L_MOTOR_2 6
#define R_MOTOR_1 3
#define R_MOTOR_2 3
#define M_BUTTON 4
#define L_BUTTON 1
#define R_BUTTON 5
#define SENSOR_0 0
#define SENSOR_1 1
#define SENSOR_2 2
#define SENSOR_3 3
#define SENSOR_4 4
#define BUZZER 2

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

void printAppendLCD(char message[8]) {
  LCD_print_String(message);
}
// Configures left motor to the direction specified
void configMotors(int direction) {
  switch (direction) {
  // Forward
  case 0:
    PORTD &= ~(1 << L_MOTOR_1);
    PORTD |= (1 << L_MOTOR_2);
    PORTD &= ~(1 << R_MOTOR_1);
    PORTB |= (1 << R_MOTOR_2);
    break;
  // Backward
  case 1:
    PORTD |= (1 << L_MOTOR_1);
    PORTD &= ~(1 << L_MOTOR_2);
    PORTD |= (1 << R_MOTOR_1);
    PORTB &= ~(1 << R_MOTOR_2);
    break;
  // Clockwise
  case 2:
    PORTD &= ~(1 << L_MOTOR_1);
    PORTD |= (1 << L_MOTOR_2);
    PORTD |= (1 << R_MOTOR_1);
    PORTB &= ~(1 << R_MOTOR_2);
    break;
  // Counterclockwise
  case 3:
    PORTD |= (1 << L_MOTOR_1);
    PORTD &= ~(1 << L_MOTOR_2);
    PORTD &= ~(1 << R_MOTOR_1);
    PORTB |= (1 << R_MOTOR_2);
    break;
  // No Movement
  default:
    PORTD |= (1 << L_MOTOR_1);
    PORTD |= (1 << L_MOTOR_2);
    PORTD |= (1 << R_MOTOR_1);
    PORTB |= (1 << R_MOTOR_2);
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
void runInstruction(int movement, int time, int speed) {
  int move_time = 0;
  int pwm_counter = 0;
  int time_count = 0;

  for (time_count = 0; time_count < time; time_count++) {
    for (move_time = 0; move_time < TIME_CONSTANT; move_time++) {
      //pwm control
      pwm_counter = pwm_counter + 1;
      if (pwm_counter >= PWM_TOP) {
        pwm_counter = 0;
      }
      // Speed of each motor is based on pwm
      // wheel_offset affects the motor that was slightly fast
      if (pwm_counter < speed) {
        configMotors(movement);
      } else {
        configMotors(4);
      }
      _delay_us(10);  // Delay to slow down loop
    }
  }
}

void moveForward(int time, int speed){
  runInstruction(0,time,speed);
}

void moveBackward(int time, int speed){
  runInstruction(1,time,speed);
}

void moveTurnRight(int time, int speed){
  runInstruction(2,time,speed);
}

void moveTurnLeft(int time, int speed){
  runInstruction(3,time,speed);
}

void moveStop(int time){
  runInstruction(4,time,2);
}

void setBuzzer(int time){
  int buzz_time = 0;
  int pwm_counter = 0;
  for (buzz_time = 0; buzz_time < time; buzz_time++) {
    //pwm control
    pwm_counter = pwm_counter + 1;
    if (pwm_counter >= PWM_TOP) {
      pwm_counter = 0;
    }
    // Speed of each motor is based on pwm
    if (pwm_counter < BUZZER_VOL) {
      PORTB |= (1 << BUZZER);
    } else {
      PORTB &= ~(1 << BUZZER);
    }
    _delay_us(10);  // Delay to slow down loop
  }
  PORTB &= ~(1 << BUZZER);
}

int main() {
  // Variables used for button states
  unsigned int last_left_button_state = (PINB & (1 << L_BUTTON));
  unsigned int left_button_pressed = 0;
  unsigned int last_right_button_state = (PINB & (1 << R_BUTTON));
  unsigned int right_button_pressed = 0;
  unsigned int last_middle_button_state = (PINB & (1 << M_BUTTON));
  unsigned int middle_button_pressed = 0;
  unsigned int sensor_data[5] = {(PINC & (1<<SENSOR_0)),(PINC & (1<<SENSOR_1)),(PINC & (1<<SENSOR_2)),(PINC & (1<<SENSOR_3)),(PINC & (1<<SENSOR_4))};

  int sensor_state[5] = {0,0,0,0,0};
  int speed = 0;
  int bunkers_max = 5;
  int bunkers_claimed = 0;
  int bunker_found = 0;
  int on_black = 0;
  int scanning = 0;
  int victory = 0;
  int max_sensors_black = 0;
  int sensors_black = 0;
  int interface_stage = 0;
  char bunkers_max_string[8];
  char bunkers_claimed_string[8];
  char blanks[8] = "        ";
  int i;
  // Configure Motor pins and ports
  DDRD |= (1 << L_MOTOR_1) | (1 << L_MOTOR_2) | (1 << R_MOTOR_1);
  DDRB |= (1 << R_MOTOR_2);
  // Configure middle push-button
  DDRB &= ~(1 << M_BUTTON);
  PORTB |= (1 << M_BUTTON);
  // Configure left push-button
  DDRB &= ~(1 << L_BUTTON);
  PORTB |= (1 << L_BUTTON);
  // Configure right push-button
  DDRB &= ~(1 << R_BUTTON);
  PORTB |= (1 << R_BUTTON);

  DDRC &= ~(1<<SENSOR_0) & ~(1<<SENSOR_1) & ~(1<<SENSOR_2) & ~(1<<SENSOR_3) & ~(1<<SENSOR_4); //configure sensors
  PORTC |= (1<<SENSOR_0) | (1<<SENSOR_1) | (1<<SENSOR_2) | (1<<SENSOR_3) | (1<<SENSOR_4);

  DDRB |= (1 << BUZZER);

  initialize_LCD_driver();
  LCD_execute_command(TURN_ON_DISPLAY);
  LCD_execute_command(CLEAR_DISPLAY);

  while(interface_stage<2) {
    // Pulser for middle button
    if ((PINB & (1 << M_BUTTON)) != last_middle_button_state) {
      if ((PINB & (1 << M_BUTTON)) == 0) {
        middle_button_pressed = 1;
      }
      last_middle_button_state = (PINB & (1 << M_BUTTON));
    } else {
      middle_button_pressed = 0;
    }
    // Pulser for left button
    if ((PINB & (1 << L_BUTTON)) != last_left_button_state) {
      if ((PINB & (1 << L_BUTTON)) == 0) {
        left_button_pressed = 1;
      }
      last_left_button_state = (PINB & (1 << L_BUTTON));
    } else {
      left_button_pressed = 0;
    }
    // Pulser for right button
    if ((PINB & (1 << R_BUTTON)) != last_right_button_state) {
      if ((PINB & (1 << R_BUTTON)) == 0) {
        right_button_pressed = 1;
      }
      last_right_button_state = (PINB & (1 << R_BUTTON));
    } else {
      right_button_pressed = 0;
    }

    switch (interface_stage) {
    case 0:
    	printTopLCD("SPEED?  ");
    	printBotLCD("S  M  F ");
    	if(left_button_pressed == 1) {
      	speed = SPEED_SLOW;
    	}
    	if(middle_button_pressed == 1) {
      	speed = SPEED_MED;
    	}
    	if(right_button_pressed == 1) {
      	speed = SPEED_FAST;
    	}
    	if(speed > 0) {
        interface_stage++;
    	}
    	break;
    case 1:
    	printTopLCD("BUNKERS?");
    	itoa(bunkers_max, bunkers_max_string, 10);
    	printBotLCD(bunkers_max_string);
      LCD_print_String(blanks);

    	if (left_button_pressed == 1) {
      	bunkers_max--;
      	if(bunkers_max < 1) {
        	bunkers_max = 1;
      	}
    	}
    	if (right_button_pressed == 1) {
      	bunkers_max++;
    	}
    	if (middle_button_pressed == 1) {
      	interface_stage++;
    	}
    	break;
    }
  }

  while(bunkers_claimed < bunkers_max){
    sensor_data[0] = (PINC & (1<<SENSOR_0));
    sensor_data[1] = (PINC & (1<<SENSOR_1));
    sensor_data[2] = (PINC & (1<<SENSOR_2));
    sensor_data[3] = (PINC & (1<<SENSOR_3));
    sensor_data[4] = (PINC & (1<<SENSOR_4));

    printTopLCD("#:");
  	itoa(bunkers_claimed, bunkers_claimed_string, 10);
  	printAppendLCD(bunkers_claimed_string);
  	printAppendLCD("/");
  	printAppendLCD(bunkers_max_string);
    printAppendLCD(blanks);
  	LCD_move_cursor_to_col_row(0,1);
    for(i = 0; i<5; i++){
      if(sensor_data[i] > 0){
          sensor_state[i] = 1;
          printAppendLCD("1");
      }
      else{
        sensor_state[i] = 0;
        printAppendLCD("0");
      }
    }
    printAppendLCD(blanks);

    if(bunker_found){
      if((!sensor_state[0] && !sensor_state[1] && sensor_state[2] && !sensor_state[3] && !sensor_state[4])||
         (!sensor_state[0] && sensor_state[1] && sensor_state[2] && sensor_state[3] && !sensor_state[4])){
        moveBackward(70,SPEED_SLOW);
        moveTurnRight(230,SPEED_SLOW);
        moveStop(250);
        bunker_found = 0;
      }
      else{
        moveTurnLeft(1,SPEED_SLOW);
      }
    }
    else if(!on_black){
      if(sensor_state[0] || sensor_state[1] || sensor_state[2] || sensor_state[3] || sensor_state[4] ){
        moveStop(30);
        on_black = 1;
      }
      else{
        moveForward(1,speed);
      }
    }
    else if(on_black){
      if(!scanning){
        if(!sensor_state[0] && !sensor_state[1] && !sensor_state[2] && !sensor_state[3] && sensor_state[4] ){
          moveStop(20);
          scanning = 1;
        }
        else{
          moveTurnLeft(1,SPEED_SLOW);
        }
      }
      else if(scanning){
        if(sensor_state[0] && !sensor_state[1] && !sensor_state[2] && !sensor_state[3] && !sensor_state[4] ){
          moveTurnRight(50+(rand()%200),SPEED_SLOW);
          moveStop(20);
          if(max_sensors_black < 4){
            setBuzzer(10000);
            bunkers_claimed ++;
            bunker_found = 1;
          }
          max_sensors_black = 0;
          on_black = 0;
          scanning = 0;
        }
        else{
          moveTurnRight(1,SPEED_SLOW);
          sensors_black = sensor_state[0] + sensor_state[1] + sensor_state[2] + sensor_state[3] + sensor_state[4];
          if(max_sensors_black < sensors_black){
            max_sensors_black = sensors_black;
          }
        }
      }
    }
  }
  while(!victory){
    sensor_data[0] = (PINC & (1<<SENSOR_0));
    sensor_data[1] = (PINC & (1<<SENSOR_1));
    sensor_data[2] = (PINC & (1<<SENSOR_2));
    sensor_data[3] = (PINC & (1<<SENSOR_3));
    sensor_data[4] = (PINC & (1<<SENSOR_4));
    for(i = 0; i<5; i++){
      if(sensor_data[i] > 0){
          sensor_state[i] = 1;
      }
      else{
        sensor_state[i] = 0;
      }
    }
    if((!sensor_state[0] && !sensor_state[1] && sensor_state[2] && !sensor_state[3] && !sensor_state[4])||
       (!sensor_state[0] && sensor_state[1] && sensor_state[2] && sensor_state[3] && !sensor_state[4])){
      moveBackward(70,SPEED_SLOW);
      moveTurnRight(230,SPEED_FAST);
      moveStop(2);
      setBuzzer(10000);
      setBuzzer(10000);
      setBuzzer(10000);
      victory = 1;
    }
    else{
      moveTurnLeft(1,SPEED_SLOW);
    }
  }
  printTopLCD("VICTORY!");
  printBotLCD("By BL&RS");
  return 0;
}
