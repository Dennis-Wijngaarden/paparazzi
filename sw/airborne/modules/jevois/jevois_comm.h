/*
 * Copyright (C) Dennis van Wijngaarden
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/jevois/jevois_comm.h"
 * @author Dennis van Wijngaarden
 * Standard communication protocol for jevois camera
 */

#ifndef JEVOIS_COMM_H
#define JEVOIS_COMM_H

#include "std.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include "generated/airframe.h"
#include "mcu_periph/uart.h"
#include "modules/sonar/sonar_bebop.h"

#define NEW_MESSAGE_BYTE 0
#define FIRST_STATE_BYTE 0
#define SECOND_STATE_BYTE 1
#define SECOND_LAST_STATE_BYTE 150
#define LAST_STATE_BYTE 151

// Data structure
struct jevois_comm_data {
	int16_t x;
	int16_t y;
};

// Transmit state structure
struct jevois_state_data {
	float alt;
	float phi;
	float theta;
	float psi;
	float lat;
	float lon;
};

// Define state union from float to 4 int8 numbers
union float2bytes {
	float f;
	char b[4];
};

extern void jevois_comm_init(void);
extern void jevois_comm_event(void);
extern void jevois_setpar_value_f(char par_name[], float value);
extern void SendString(char string[]);
//extern void jevois_test(void);
extern void get_jevois_state(void);
extern void jevois_parse_state(void);
extern void SendFloat(float);

#endif

