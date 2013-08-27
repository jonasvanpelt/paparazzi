/*
 * Author: Jonas Van Pelt
 * */

//include library for servo commands
#include "subsystems/actuators/set_servo.h"
#include "led.h"
#include "subsystems/datalink/datalink.h"

typedef union{ //message id 72
	uint8_t raw[14]; 
	struct Output_message {
			int16_t servo_1;
			int16_t servo_2;
			int16_t servo_3;
			int16_t servo_4;
			int16_t servo_5;
			int16_t servo_6;
			int16_t servo_7;
		} message;
} Input;

//when SERVO_COMMANDS message is received, drive_servos() will be called automatically
void drive_servos(){
	Input input;
	int i;

	for(i=0;i<14;i++){
		input.raw[i]=dl_buffer[2+i];
	}
		
	set_servo.servo_1=input.message.servo_1;
	set_servo.servo_2=input.message.servo_2;
	set_servo.servo_3=input.message.servo_3;
	set_servo.servo_4=input.message.servo_4;
	set_servo.servo_5=input.message.servo_5;
	set_servo.servo_6=input.message.servo_6;
	set_servo.servo_7=input.message.servo_7;
	

}
