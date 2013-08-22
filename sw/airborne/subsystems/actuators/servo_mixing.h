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

/** @file servo_mixing.h
 *  Servo Mixing.
 *  Handles the mapping of roll/pitch/yaw commands
 *  to actual servo commands after filtering.
 */

#ifndef SERVO_MIXING_H
#define SERVO_MIXING_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

struct ServoMixing {
  int32_t commands[SERVO_MIXING_NB_SERVO];
  int32_t trim[SERVO_MIXING_NB_SERVO];
};

extern struct ServoMixing servo_mixing;
extern struct ServoMixing servo_mixing_old;

extern void servo_mixing_init(void);
extern void servo_mixing_run(pprz_t in_cmd[]);
extern int32_t aileron_rcscaler;
extern int32_t aileron_scaler;
extern int32_t elevator_rcscaler;
extern int32_t elevator_scaler;
extern int32_t sample_size;

#endif /* SERVO_MIXING_H */
