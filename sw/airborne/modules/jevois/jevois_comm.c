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

// set state data structure
static struct jevois_state_data jevois_state;

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
	//jevois_setpar_value_f("alt", stateGetPositionEnu_i()->z); // Get position from ENU measurements as integer 32 in meters
	//jevois_setpar_value("alt", state.alt_agl_f); // Altitude above ground as float
	//jevois_setpar_value("alt", sonar_bebop.distance); // Sonar measurement directly taken from the bebop sonar subsystem
	//jevois_setpar_value_f("lat", stateGetPositionLla_f()->lat); // In radians
	//jevois_setpar_value_f("lon", stateGetPositionLla_f()->lon); // In radians
	//jevois_setpar_value_f("alt", stateGetNedToBodyEulers_f()->phi); //In radians
	//JevoisLinkTransmit(5);
	//JevoisLinkTransmit(10);
	//jevois_setpar_value_f("theta", stateGetNedToBodyEulers_f()->theta); //In radians
	//jevois_setpar_value_f("psi", stateGetNedToBodyEulers_f()->psi); //In radians
}

void get_jevois_state(){
	jevois_state.alt = stateGetPositionEnu_f()->z; //m
	jevois_state.phi = stateGetNedToBodyEulers_f()->phi; //rad
	jevois_state.theta = stateGetNedToBodyEulers_f()->theta; //rad
	jevois_state.psi = stateGetNedToBodyEulers_f()->psi; //rad
	jevois_state.lat = stateGetPositionLla_f()->lat; //radians
	jevois_state.lon = stateGetPositionLla_f()->lon; //radians
}

void jevois_parse_state(){
	get_jevois_state();
	//Initiate Data package transmission
	//JevoisLinkTransmit(NEW_MESSAGE_BYTE);
	JevoisLinkTransmit(FIRST_STATE_BYTE); // 1 byte 0
	JevoisLinkTransmit(SECOND_STATE_BYTE); // 1 byte 1
	//JevoisLinkTransmit(6);
	//JevoisLinkTransmit(19);
	//JevoisLinkTransmit(13);
	//Transmit Data
	SendFloat(jevois_state.alt); //parse alt 4 bytes 2 3 4 5
	SendFloat(jevois_state.phi); //parse phi 4 bytes 6 7 8 9
	SendFloat(jevois_state.theta); //parse theta 4 bytes 10 11 12 13
	SendFloat(jevois_state.psi); //parse psi 4 bytes 14 15 16 17
	SendFloat(jevois_state.lat); //parse lat 4 bytes 18 19 20 21
	SendFloat(jevois_state.lon); //parse lon 4 bytes 22 23 24 25
	// End Data package
	JevoisLinkTransmit(SECOND_LAST_STATE_BYTE); // 1 byte 26
 	JevoisLinkTransmit(LAST_STATE_BYTE); // 1 byte 27
}

void SendFloat(float f){
	// Define state union from float to 4 int8 bytes
	union float2bytes {float f;char b[4];} f2b;
	f2b.f = f;
	for (int i=0; i<sizeof(float); i++){
		JevoisLinkTransmit(f2b.b[i]);
	}
}

void SendString(char string[])
{
  int i = 0;

  while (string[i]) {
    JevoisLinkTransmit(string[i++]);
  }
}


