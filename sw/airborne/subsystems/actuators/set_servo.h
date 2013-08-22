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

/** @file set_servo.h
 *  Set Servos.
 *  Handles the setting of servo outputs based on actuator commands 
 *  transmitted via the telemetry interface.
 */

#ifndef SET_SERVO_H
#define SET_SERVO_H

#include "std.h"
#include "paparazzi.h"

//struct SetServo {
//  int32_t aileron_left;
//  int32_t aileron_right;
//  int32_t flap_left;
//  int32_t flap_right;
//  int32_t rudder;
//  int32_t elevator;
//  int32_t mode;
//}

struct SetServo {
  int16_t servo_1;
  int16_t servo_2;
  int16_t servo_3;
  int16_t servo_4;
  int16_t servo_5;
  int16_t servo_6;
  int16_t servo_7;
};

extern struct SetServo set_servo;

#endif /* SET_SERVO_H */
