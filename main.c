#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <avr/io.h>
#include "lcd_driver.h"
#include "port_macros.h"
#define PWM_TOP 100
#define TIME_CONSTANT 14400
#define WHEEL_OFFSET 0

void stopLeftMotor(){
	PORTD |= (1<<5);
	PORTD |= (1<<6);
}

void stopRightMotor(){
	PORTD |= (1<<3);
	PORTB |= (1<<3);
}

void configLeftMotor(int direction){
	switch (direction) {
		case 0:
			PORTD |= (1<<5);
			PORTD &= ~(1<<6);
			break;
		case 1:
			PORTD &= ~(1<<5);
			PORTD |= (1<<6);
			break;
		case 2:
			PORTD &= ~(1<<5);
			PORTD |= (1<<6);
			break;
		case 3:
			PORTD |= (1<<5);
			PORTD &= ~(1<<6);
			break;
	}
}

void configRightMotor(int direction){
	switch (direction) {
		case 0:
			PORTD &= ~(1<<3);
			PORTB |= (1<<3);
			break;
		case 1:
			PORTD |= (1<<3);
			PORTB &= ~(1<<3);
			break;
		case 2:
			PORTD &= ~(1<<3);
			PORTB |= (1<<3);
			break;
		case 3:
			PORTD |= (1<<3);
			PORTB &= ~(1<<3);
			break;
	}
}

void runInstruction(int movement, int speed, int time){
	int clock = 0;
	int pwm_counter=0;
	int time_count = 0;
	for(time_count=0; time_count < time; time_count++){
		clock = 0;
		while(clock < TIME_CONSTANT){
			pwm_counter = pwm_counter + 1;
			clock++;
			if( pwm_counter >= PWM_TOP ){
				pwm_counter = 0;
			}

			if( pwm_counter < speed ){
				configLeftMotor(movement);
			}
			else{
				stopLeftMotor();
			}

			if( pwm_counter < speed - WHEEL_OFFSET){
				configRightMotor(movement);
			}
			else{
				stopRightMotor();
			}

			_delay_us(10);
	  }
	}
		stopLeftMotor();
		stopRightMotor();
}

int main(){
	unsigned int last_left_button_state = (PINB & (1<<1));
	unsigned int left_button_pressed = 0;
	unsigned int last_right_button_state = (PINB & (1<<5));
	unsigned int right_button_pressed = 0;
	unsigned int last_middle_button_state = (PINB & (1<<4));
	unsigned int middle_button_pressed = 0;

	int movement[4];
	int speed[4];
	int time[4];
	int command_count = 1;
	int stage = 0;
	int menu = 0;
	int i = 0;
	int j = 0;
	int duration = 100;
	char blanks[8]="        ";

	char row1[8];
	char row2[8];
	//Configure Motors

	DDRD |= (1<<5) | (1<<6) | (1<<3);
	DDRB |= (1<<3);
	//Configure middle push-button
	DDRB &= ~(1<<4); //configure pin as input
	PORTB |= (1<<4); //enable pull-up resistor
	//Configure left push-button
	DDRB &= ~(1<<1); //configure pin as input
	PORTB |= (1<<1); //enable pull-up resistor
	//Configure right push-button
	DDRB &= ~(1<<5); //configure pin as input
	PORTB |= (1<<5); //enable pull-up resistor


	initialize_LCD_driver(); // Initialize LCD Driver
	LCD_execute_command(TURN_ON_DISPLAY); // Turn On LCD Display
	LCD_execute_command(CLEAR_DISPLAY); // Clear LCD Display
	//stopRightMotor();
	//stopLeftMotor();

	while(stage<5){
		//Pulser for middle button
		if( (PINB & (1<<4)) != last_middle_button_state ){
			if( (PINB & (1<<4)) == 0 ){
				middle_button_pressed=1;
			}
			last_middle_button_state = (PINB & (1<<4));
		}
		else{
			middle_button_pressed=0;
		}
		//Pulser for left button
		if( (PINB & (1<<1)) != last_left_button_state ){
			if( (PINB & (1<<1)) == 0 ){
				left_button_pressed=1;
			}
			last_left_button_state = (PINB & (1<<1));
		}
		else{
			left_button_pressed=0;
		}
		//Pulser for right button
		if( (PINB & (1<<5)) != last_right_button_state ){
			if( (PINB & (1<<5)) == 0 ){
				right_button_pressed=1;
			}
			last_right_button_state = (PINB & (1<<5));
		}
		else{
			right_button_pressed=0;
		}


		//Start doing the interface
    if(stage == 0) {
   						strcpy(row1,"Commands");
							LCD_move_cursor_to_col_row(0,0);
            	LCD_print_String(row1);
            	itoa(command_count,row2,10);
            	LCD_move_cursor_to_col_row(0,1);
            	LCD_print_String(row2);

            	if(left_button_pressed == 1) {
                	command_count--;
                	if(command_count < 1) {
                    	command_count = 1;
                	}
            	}
   							 if(right_button_pressed == 1) {
                	command_count++;
                	if(command_count > 4) {
                    	command_count = 4;
                	}
            	}
   							 if(middle_button_pressed == 1) {
                	stage++;
                	LCD_execute_command(CLEAR_DISPLAY);
            	}
    	}
			else if(stage == 1) {
						strcpy(row1,"Movement");
						LCD_move_cursor_to_col_row(0,0);
						LCD_print_String(row1);
							 if(menu == 0) {
								 strcpy(row2,"CCW     ");
							 } else if (menu == 1) {
								 strcpy(row2,"CW      ");
							 } else if (menu == 2) {
								 strcpy(row2,"FORWARD ");
							 } else if (menu == 3) {
								 strcpy(row2,"REVERSE ");
   						 }
							 LCD_move_cursor_to_col_row(0,1);
							 LCD_print_String(row2);


   							 if(left_button_pressed == 1) {
   									 menu--;
   									 if(menu < 0) {
   											 menu = 3;
   									 }
   							 }
   							 if(right_button_pressed == 1) {
   									 menu++;
   									 if(menu > 3) {
   											 menu = 0;
   									 }
   							 }
   							 if(middle_button_pressed == 1) {
   								 LCD_execute_command(CLEAR_DISPLAY);
   								 movement[i] = menu;
   								 stage++;
   							 }
   			 }
				 else if(stage == 2) {
   						 strcpy(row1,"Speed");
       		 LCD_print_String(row1);
   						 LCD_move_cursor_to_col_row(0,1);
   						 strcpy(row2,"S   M  F");
   						 LCD_print_String(row2);

   						 if(left_button_pressed == 1) {
   							 speed[i] = 20;
   							 stage++;
   							 LCD_execute_command(CLEAR_DISPLAY);
   						 }
   						 if(middle_button_pressed == 1) {
   							 speed[i] = 40;
   							 stage++;
   							 LCD_execute_command(CLEAR_DISPLAY);
   						 }
   						 if(right_button_pressed == 1) {
   							 speed[i] = 60;
   							 LCD_execute_command(CLEAR_DISPLAY);
   							 stage++;
   						 }
    	}
			else if(stage == 3) {
						LCD_move_cursor_to_col_row(0,0);
						 strcpy(row1,"Time");
   					 LCD_print_String(row1);
   					 itoa(duration,row2,10);
   					 LCD_move_cursor_to_col_row(0,1);
   					 LCD_print_String(row2);
						 LCD_print_String(blanks);

   					 if(left_button_pressed == 1) {
   						 duration--;
   						 if(duration < 0) {
   							 duration = 0;
   						 }
   					 }
   					 if(right_button_pressed == 1) {
   						 duration++;
   					 }
   					 if(middle_button_pressed == 1) {
   						 time[i] = duration;
   						 LCD_execute_command(CLEAR_DISPLAY);
   						 stage++;
   					 }
   		  }
				else if(stage == 4) {
						i++;
						if(i < command_count) {
							 stage = 1;
   					}
						else {
   						 stage++;

   					}
   				 }
			 //small delay to slow down main loop
		_delay_us(10);
	}

	//Robot movement according to instructions
	for(j=0; j<command_count; j++){
		runInstruction(movement[j],speed[j],time[j]);
	}
	return 0;
}
