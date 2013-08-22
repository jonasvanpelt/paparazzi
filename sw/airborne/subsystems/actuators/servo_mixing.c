/*
 * Copyright (C) 2008-2012 The Paparazzi Team
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file servo_mixing.c
 *  Servo Mixing.
 *  Handles the mapping of roll/pitch/yaw commands
 *  to actual servo commands after filtering.
 */

#include "subsystems/actuators/servo_mixing.h"
#include "paparazzi.h"

//#include <stdint.h>

struct ServoMixing servo_mixing;
struct ServoMixing servo_mixing_old;
int32_t aileron_rcscaler;
int32_t aileron_scaler;
int32_t elevator_rcscaler;
int32_t elevator_scaler;
int32_t sample_size;

void servo_mixing_init(void) {
  uint8_t i;
  for (i=0; i<SERVO_MIXING_NB_SERVO; i++) {
    servo_mixing.commands[i] = 0;
  }
  servo_mixing.trim[0] = SERVO_MIXING_TRIM_FLAPERON_LEFT;
  servo_mixing.trim[1] = SERVO_MIXING_TRIM_FLAPERON_RIGHT;
  servo_mixing.trim[2] = SERVO_MIXING_TRIM_ELEVATOR;
  servo_mixing.trim[3] = SERVO_MIXING_TRIM_SAMARA_LEFT;
  servo_mixing.trim[4] = SERVO_MIXING_TRIM_SAMARA_RIGHT;

  for (i=0; i<SERVO_MIXING_NB_SERVO; i++) {
    servo_mixing_old.commands[i] = 0;
  }
  servo_mixing_old.trim[0] = SERVO_MIXING_TRIM_FLAPERON_LEFT;
  servo_mixing_old.trim[1] = SERVO_MIXING_TRIM_FLAPERON_RIGHT;
  servo_mixing_old.trim[2] = SERVO_MIXING_TRIM_ELEVATOR;
  servo_mixing_old.trim[3] = SERVO_MIXING_TRIM_SAMARA_LEFT;
  servo_mixing_old.trim[4] = SERVO_MIXING_TRIM_SAMARA_RIGHT;

  sample_size=STABILIZATION_ATTITUDE_SAMPLE_SIZE;
  aileron_rcscaler=STABILIZATION_ATTITUDE_AILERON_RCSCALER;
  aileron_scaler=STABILIZATION_ATTITUDE_AILERON_SCALER;
  elevator_rcscaler=STABILIZATION_ATTITUDE_ELEVATOR_RCSCALER;
  elevator_scaler=STABILIZATION_ATTITUDE_ELEVATOR_SCALER;
}

__attribute__ ((always_inline)) static inline void bound_commands(void) {
  uint8_t j;
  for (j=0; j<SERVO_MIXING_NB_SERVO; j++)
    Bound(servo_mixing.commands[j],
          SERVO_MIXING_MIN_SERVO, SERVO_MIXING_MAX_SERVO);
}

void servo_mixing_run(pprz_t in_cmd[]) {
  int32_t flaps;
  int32_t tilt;
  if (in_cmd[COMMAND_RCFLAPS]>2000)
    flaps=2880;
  else
    flaps=0;
  if(in_cmd[COMMAND_RCMODE]>2000){
    servo_mixing.commands[0] = ((sample_size-1)*servo_mixing_old.commands[0] + (servo_mixing.trim[0] + (aileron_rcscaler * in_cmd[COMMAND_RCAILERON])/10 + flaps))/(sample_size);
    servo_mixing.commands[1] = ((sample_size-1)*servo_mixing_old.commands[1] + (servo_mixing.trim[1] + (aileron_rcscaler * in_cmd[COMMAND_RCAILERON])/10 - flaps))/(sample_size);
    servo_mixing.commands[2] = ((sample_size-1)*servo_mixing_old.commands[2] + (servo_mixing.trim[2] + (elevator_rcscaler * in_cmd[COMMAND_RCELEVATOR])/10))/(sample_size);
    }
  else{
    servo_mixing.commands[0] = ((sample_size-1)*servo_mixing_old.commands[0] + (servo_mixing.trim[0] + (aileron_scaler * in_cmd[COMMAND_ROLL])/10 + flaps))/(sample_size);
    servo_mixing.commands[1] = ((sample_size-1)*servo_mixing_old.commands[1] + (servo_mixing.trim[1] + (aileron_scaler * in_cmd[COMMAND_ROLL])/10 - flaps))/(sample_size);
    servo_mixing.commands[2] = ((sample_size-1)*servo_mixing_old.commands[2] + (servo_mixing.trim[2] + (elevator_scaler * in_cmd[COMMAND_PITCH])/10))/(sample_size);
    }
  servo_mixing_old.commands[0] = servo_mixing.commands[0];
  servo_mixing_old.commands[1] = servo_mixing.commands[1];
  servo_mixing_old.commands[2] = servo_mixing.commands[2];
  if(in_cmd[COMMAND_TILT]>0){
    tilt=(500 + ((9600-500) * in_cmd[COMMAND_TILT])/9600);
    }
  else{
    tilt=(500 + ((9600+500) * in_cmd[COMMAND_TILT])/9600);
    }
  if(in_cmd[COMMAND_SAMKILL]<-1000){
    servo_mixing.commands[3]= (-3*in_cmd[COMMAND_YAW] -tilt);
    servo_mixing.commands[4]= (-3*in_cmd[COMMAND_YAW] +tilt);
   }
  else if(in_cmd[COMMAND_SAMKILL]>-1000 && in_cmd[COMMAND_SAMKILL]<2000){
    servo_mixing.commands[3]= (-tilt);
    servo_mixing.commands[4]= (+tilt);
    }
  else{
    servo_mixing.commands[3]= (0);
    servo_mixing.commands[4]= (0);
    }
  bound_commands();
}

