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
 * @file "modules/jevois/jevois_comm.c"
 * @author Dennis van Wijngaarden
 * Standard communication protocol for jevois camera
 */

#include "modules/jevois/jevois_comm.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "state.h"

#include "mcu_periph/uart.h"
#include "modules/sonar/sonar_bebop.h"

#define JevoisLinkDev (&((JEVOIS_PORT).device))
#define JevoisLinkTransmit(c) JevoisLinkDev->put_byte(JevoisLinkDev->periph, 0, c)

// Set message buffer
static char jevois_msg_buf[128] __attribute__ ((aligned)); // Create a 128 bytes buffer

// Set data structure
static struct jevois_comm_data jevois_data;

uint8_t testint=200;

void jevois_comm_init()
{
}

void jevois_comm_event()
{
	static uint8_t i = 0;
	static uint8_t j = 0;

	while (uart_char_available(&(JEVOIS_PORT))) {
		uint8_t ch;
		ch = uart_getch(&(JEVOIS_PORT));
		//printf("%d\n",ch);
		if (ch != 10) {
			jevois_msg_buf[i] = ch;
			i++;
		}
		else {
			char *tok = strtok(jevois_msg_buf, " \r\n");
			//printf("%s\n",jevois_msg_buf);
			while (tok) {
				if (j == 2) {
					jevois_data.x = atoi(tok);
					//printf("%s\n",tok);
				}
				if (j == 3) {
					jevois_data.y = atoi(tok);
				}
				tok = strtok(0, " \r\n");
				j++;
			}
			//jevois_parse_msg();
			i = 0;
			j = 0;
		}
	}
}



void jevois_setpar_value_f(char par_name[], float value) {
	char valuestring[16];
	sprintf(valuestring, "%f", value);
	SendString("setpar ");
	SendString(par_name);
	SendString(" ");
	SendString(valuestring);
	JevoisLinkTransmit(13);
	JevoisLinkTransmit(10); //Parse line
}

void jevois_test(){
	jevois_setpar_value_f("alt", stateGetPositionEnu_f()->z); // Get position from NED measurements as float in meters
	//jevois_setpar_value("alt", state.alt_agl_f); // Altitude above ground as float
	//jevois_setpar_value("alt", sonar_bebop.distance); // Sonar measurement directly taken from the bebop sonar subsystem
	jevois_setpar_value_f("lat", stateGetPositionLla_f()->lat); // In radians
	jevois_setpar_value_f("lon", stateGetPositionLla_f()->lon); // In radians
	jevois_setpar_value_f("phi", stateGetNedToBodyEulers_f()->phi); //In radians
	jevois_setpar_value_f("theta", stateGetNedToBodyEulers_f()->theta); //In radians
	jevois_setpar_value_f("psi", stateGetNedToBodyEulers_f()->psi); //In radians
}


void SendString(char string[])
{
  int i = 0;

  while (string[i]) {
    JevoisLinkTransmit(string[i++]);
  }
}


