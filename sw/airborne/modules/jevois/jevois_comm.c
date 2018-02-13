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

#include "pprzlink/pprz_transport.h"
#include "mcu_periph/uart.h"

// Main jevois_comm structure
static struct jevois_comm_t jevois_comm = {
		.device = (&((JEVOIS_PORT).device)),
		.msg_available = false
};

#define JevoisLinkTransmit(c) jevois_comm.device->put_byte(jevois_comm.device->periph, 0, c)

// Set message buffer
static char jevois_msg_buf[128] __attribute__ ((aligned)); // Create a 128 bytes buffer

// Set data structure
static struct jevois_comm_data jevois_data;

void jevois_comm_init()
{
	pprz_transport_init(&jevois_comm.transport);
	//uart_periph_set_baudrate(&uart2, B115200);
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

void jevois_parse_command() {
	static uint8_t i = 0;
	static uint8_t j;
	static uint8_t c;
	static uint8_t command_parsed = 0;
	static char setpar[8] = "setpar ";
	static char altitude_str[] = "altitude ";

	j = strlen(setpar);

	while(command_parsed == 0) {
		if(i < j) {
			c = setpar[i];
			JevoisLinkTransmit(c);
		}
		else{
			command_parsed = 1;
		}
	}

}


